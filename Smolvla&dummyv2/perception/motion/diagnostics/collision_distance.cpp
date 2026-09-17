// Read-only diagnostic. Queries the existing model/scene, then computes locally.
// No publishers, motion services, parameter writes or hardware access.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace std::chrono_literals;

void report_pose(std::ostream& out, const std::string& name,
                 planning_scene::PlanningScene& scene, moveit::core::RobotState state, double threshold)
{
  state.updateCollisionBodyTransforms();
  collision_detection::CollisionRequest check;
  check.group_name = "dummy_arm";
  check.distance = true;
  collision_detection::CollisionResult collision;
  scene.getCollisionEnvUnpadded()->checkSelfCollision(check, collision, state, scene.getAllowedCollisionMatrix());
  collision_detection::DistanceRequest req;
  req.group_name = "dummy_arm";
  req.enableGroup(scene.getRobotModel());
  req.acm = &scene.getAllowedCollisionMatrix();
  req.type = collision_detection::DistanceRequestType::ALL;
  req.enable_nearest_points = true;
  req.distance_threshold = 1.0;
  collision_detection::DistanceResult result;
  scene.getCollisionEnvUnpadded()->distanceSelf(req, result, state);
  std::vector<collision_detection::DistanceResultsData> distances;
  for (const auto& pair : result.distances)
    for (const auto& value : pair.second) distances.push_back(value);
  std::sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) { return a.distance < b.distance; });
  const double scale = collision.collision ? 0. : std::min(1., std::exp(-std::log(.001) / threshold * (collision.distance - threshold)));
  out << "{\"pose\":" << std::quoted(name) << ",\"collision\":" << (collision.collision ? "true" : "false")
      << ",\"servo_distance_mm\":" << collision.distance * 1000 << ",\"velocity_scale\":" << scale
      << ",\"model_angles_deg\":[";
  for (int i=0; i<6; ++i) {
    if (i) out << ',';
    out << state.getVariablePosition("Joint" + std::to_string(i+1)) * 180 / M_PI;
  }
  out << "],\"nearest_pairs\":[";
  for (size_t i=0; i<std::min(size_t(8), distances.size()); ++i) {
    const auto& d = distances[i];
    if (i) out << ',';
    out << "{\"links\":[" << std::quoted(d.link_names[0]) << ',' << std::quoted(d.link_names[1])
        << "],\"distance_mm\":" << d.distance * 1000 << ",\"points_m\":[";
    for (int point=0; point<2; ++point) {
      if (point) out << ',';
      out << '[' << d.nearest_points[point][0] << ',' << d.nearest_points[point][1] << ',' << d.nearest_points[point][2] << ']';
    }
    out << "]}";
  }
  out << "]}";
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("readonly_collision_distance_diagnostic");
    auto params = std::make_shared<rclcpp::SyncParametersClient>(node, "/servo_node");
    if (!params->wait_for_service(5s)) throw std::runtime_error("servo parameters unavailable");
    auto values = params->get_parameters({"robot_description", "robot_description_semantic", "moveit_servo.self_collision_proximity_threshold"}, 5s);
    const double threshold = values.at(2).as_double();
    if (!std::isfinite(threshold) || threshold <= 0) throw std::runtime_error("invalid collision threshold");
    auto urdf = urdf::parseURDF(values.at(0).as_string());
    if (!urdf) throw std::runtime_error("URDF parsing failed");
    auto srdf = std::make_shared<srdf::Model>();
    if (!srdf->initString(*urdf, values.at(1).as_string())) throw std::runtime_error("SRDF parsing failed");
    auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    planning_scene::PlanningScene scene(model);
    auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!client->wait_for_service(5s)) throw std::runtime_error("planning scene unavailable");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = 1023;
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, 5s) != rclcpp::FutureReturnCode::SUCCESS)
      throw std::runtime_error("planning scene read timed out");
    auto snapshot = future.get()->scene;
    scene.setPlanningSceneMsg(snapshot);
    std::ofstream report(argc > 1 ? argv[1] : "collision-distance-report.json");
    report << std::setprecision(10);
    report << "{\"hardware_io\":false,\"snapshot_is_cached\":true,\"self_proximity_mm\":" << threshold*1000 << ",\"environment_objects\":"
           << snapshot.world.collision_objects.size() << ",\"octomap_bytes\":" << snapshot.world.octomap.octomap.data.size()
           << ",\"poses\":[";
    report_pose(report, "last_gateway_feedback", scene, scene.getCurrentState(), threshold);
    std::vector<std::pair<std::string, std::array<double,6>>> poses = {
      {"ready", {0,0,90,0,45,0}}, {"rest", {0,-75,180,0,0,0}},
      {"spread_a", {30,-25,110,20,65,10}}, {"spread_b", {-30,20,80,-30,-45,40}},
      {"wrist_90", {0,0,90,0,90,0}}
    };
    for (const auto& [name, hardware] : poses) {
      auto state = scene.getCurrentState();
      for (int i=0; i<6; ++i) {
        double q = hardware[i] * M_PI / 180 * (i>=4 ? -1 : 1) - (i==2 ? M_PI/2 : 0);
        state.setVariablePosition("Joint"+std::to_string(i+1), q);
      }
      report << ',';
      report_pose(report, name, scene, state, threshold);
    }
    report << "],\"base_link2_sweep\":{";
    // Only J1/J2 change this pair's relative transform. All values are local
    // RobotState copies; never published to the running planning scene.
    collision_detection::AllowedCollisionMatrix pair_acm(model->getLinkModelNames(), true);
    pair_acm.setEntry("base_link", "link2_1_1", false);
    collision_detection::CollisionRequest pair_request;
    pair_request.group_name = "dummy_arm";
    pair_request.distance = true;
    size_t samples = 0, collisions = 0;
    double minimum_mm = 1e9, maximum_mm = 0, minimum_j1 = 0, minimum_j2 = 0;
    for (int j1=-170; j1<=170; j1+=5) {
      for (int j2=-75; j2<=90; j2+=5) {
        auto state = scene.getCurrentState();
        state.setVariablePosition("Joint1", j1 * M_PI / 180);
        state.setVariablePosition("Joint2", j2 * M_PI / 180);
        state.updateCollisionBodyTransforms();
        collision_detection::CollisionResult pair_result;
        scene.getCollisionEnvUnpadded()->checkSelfCollision(pair_request, pair_result, state, pair_acm);
        ++samples;
        if (pair_result.collision) ++collisions;
        double mm = pair_result.distance * 1000;
        if (mm < minimum_mm) { minimum_mm = mm; minimum_j1 = j1; minimum_j2 = j2; }
        maximum_mm = std::max(maximum_mm, mm);
      }
    }
    report << "\"step_deg\":5,\"samples\":" << samples << ",\"colliding_samples\":" << collisions
           << ",\"min_distance_mm\":" << minimum_mm << ",\"max_distance_mm\":" << maximum_mm
           << ",\"min_at_hardware_j1_j2\":[" << minimum_j1 << ',' << minimum_j2 << "]}}\n";
    std::cout << "Read-only collision report written. No robot commands sent.\n";
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
