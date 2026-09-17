"""Real isolated MoveIt + production guard + simulated USB controller. No hardware I/O."""
import os
os.environ['ROS_DOMAIN_ID'] = '196'
import json
import http.client
from http.server import ThreadingHTTPServer
import math
from pathlib import Path
import subprocess
import sys
import threading
import time
import rclpy
from moveit_servo_gateway import GatewayNode, handler_for
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from rclpy.parameter_client import AsyncParameterClient
from joint_contract import model_limits
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent))
from perception_service.robot import RobotClient, COMMAND_LIMITS
from perception_service.live_control import LiveControl


def main():
    log = open(ROOT/'../runtime/control-loop-offline.log','w')
    process = subprocess.Popen([sys.executable,str(ROOT/'run_safe_launch.py')],stdout=log,stderr=log)
    rclpy.init()
    node = GatewayNode()
    thread = threading.Thread(target=rclpy.spin,args=(node,),daemon=True)
    thread.start()
    actual = [0.,0.,90.,0.,45.,0.]
    target = actual.copy()
    sent, events = [], []
    def bridge(method,path,body=None,timeout=4):
        if path == '/command' and body['command'] == '#GETJPOS':
            return {'raw_response':'ok '+' '.join(f'{q:.4f}' for q in actual)}
        if path == '/command' and body['command'] == '#CMDMODE 2':
            return {'raw_response':'ok Set command mode to [2]'}
        if path == '/motion':
            target[:] = body['positions']
            sent.append({'gap':max(abs(a-b) for a,b in zip(target,actual)), 'target':target.copy(),
                         'feedback_gap':max(abs(a-b) for a,b in zip(
                             target, robot.status()['state']['positions'])),
                         'at':time.monotonic()})
            return {'accepted':True}
        if path == '/command' and body['command'] in {'!DISABLE','!HAND_DIS'}:
            return {'raw_response':'Disabled ok' if body['command']=='!DISABLE' else 'ok hand disable'}
        if path == '/stop': return {'stop':{'acknowledged':True}}
        raise AssertionError((path,body))
    def gateway(method,path,body,timeout):
        if path == '/health': return node.health()
        if path == '/feedback':
            node.set_feedback(body['positions'], body.get('age_ms',0))
            return {'accepted':True,'trajectory':node.latest_trajectory()}
        if path == '/command': node.set_command(body); return {'accepted':True}
        if path == '/trajectory': return node.latest_trajectory()
        if path == '/arm':
            okay,reason = node.arm()
            assert okay,reason
            return {'armed':True}
        if path == '/disarm': node.disarm(); return {'armed':False}
        raise AssertionError(path)
    robot = RobotClient(start_sampler=False,event_callback=events.append)
    robot._request = bridge
    robot._enabled_latch = 'confirmed'
    control = LiveControl(robot,request=gateway,start_worker=False,event_callback=events.append)
    pauses = []
    pause_motion = control._pause_motion
    def capture_pause(reason):
        if control.status()['paused_reason'] != reason:
            pauses.append({'reason': reason, 'trajectory': node.latest_trajectory(),
                           'feedback_age': time.monotonic()-node.feedback_at,
                           'command_age': time.monotonic()-node.command_at})
        pause_motion(reason)
    control._pause_motion = capture_pause
    server = ThreadingHTTPServer(('127.0.0.1',0),handler_for(node))
    http_thread = threading.Thread(target=server.serve_forever,daemon=True)
    http_thread.start()
    try:
        deadline = time.monotonic()+20
        while node.health()['status'] != 'ready' and time.monotonic()<deadline:
            time.sleep(.1)
        parameters = AsyncParameterClient(node, '/servo_node')
        assert parameters.wait_for_services(timeout_sec=5), 'Servo parameter service unavailable'
        future = parameters.get_parameters(['robot_description'])
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.02)
        assert future.done(), 'Servo model query timed out'
        loaded_model = ET.fromstring(future.result().values[0].string_value)
        loaded_limits = []
        for axis, expected in enumerate(model_limits(), 1):
            limit = loaded_model.find(f"joint[@name='Joint{axis}']/limit")
            bounds = [float(limit.get('lower')), float(limit.get('upper'))]
            assert all(abs(a-b) < 1e-9 for a, b in zip(bounds, expected)), (axis, bounds, expected)
            loaded_limits.append(bounds)
        robot.read_state()
        session = control.arm({'confirmation':'ENABLE_LIVE_CONTROL'})['session_id']
        # Let this newly launched isolated ROS graph receive its initial robot
        # state before simulating the first key press (not a hardware delay).
        warmup = time.monotonic()+1.0
        while time.monotonic() < warmup:
            robot.read_state()
            control.update({'keys': [], 'session_id': session})
            control._cycle()
            time.sleep(.025)
        # Exercise the real HTTP validator too; direct node calls would miss a
        # gateway still rejecting the new speeds. This is an ephemeral test port.
        def check_http_speed(field, value, expected):
            connection = http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=2)
            command = control._zero_command()
            command[field] = value
            connection.request('POST','/command',json.dumps(command),headers={
                'Content-Type':'application/json','Host':'127.0.0.1:8801'})
            response = connection.getresponse()
            result = response.read().decode()
            connection.close()
            assert response.status == expected,(field,value,response.status,result)
        check_http_speed('linear_x',.1,200)
        check_http_speed('linear_x',.101,422)
        check_http_speed('linear_z',.1,200)
        check_http_speed('linear_z',-.1,200)
        check_http_speed('angular_z',.24,200)
        check_http_speed('angular_z',.241,422)
        node.set_command(control._zero_command())
        stages = [('w',{'keys':['w']},False,1.5), ('a',{'keys':['a']},False,1.5),
                  ('up',{'keys':['arrowup']},False,1.5), ('down',{'keys':['arrowdown']},False,1.5),
                  ('look',{'yaw':.4},False,1.5),('release',{'stop_mode':'release'},False,.8),
                  # Allow Servo's post-arm filter startup before the existing
                  # 3-second no-progress timer (which starts beyond 0.35 deg).
                  ('stalled',{'keys':['w']},True,5.0)]
        outcomes = []
        for name,body,jammed,duration in stages:
            if name not in {'w', 'release'}:
                # Each direction starts at the same simulated pose. Without
                # the old 6-degree cap, chaining long stages can reach a joint
                # bound and intentionally latch the remaining stages paused.
                control.disarm('offline_stage_reset', disable_robot=False)
                actual[:] = [0., 0., 90., 0., 45., 0.]
                target[:] = actual
                robot._last_good_feedback = None  # Teleport only the simulator.
                robot.read_state()
                session = control.arm({'confirmation':'ENABLE_LIVE_CONTROL'})['session_id']
                settle_until = time.monotonic() + .6
                while time.monotonic() < settle_until:
                    robot.read_state()
                    control.update({'keys': [], 'session_id': session})
                    control._cycle()
                    time.sleep(.025)
            before = actual.copy()
            target_before = list(target)
            sent_start = len(sent)
            end = time.monotonic()+duration
            previous = time.monotonic()
            next_input = next_feedback = 0.
            while time.monotonic()<end:
                now = time.monotonic()
                dt = min(.1,now-previous)
                previous = now
                if not jammed:
                    for i in range(6):
                        actual[i] += max(-60*dt,min(60*dt,target[i]-actual[i]))
                if now >= next_feedback:
                    robot.read_state()
                    next_feedback = now+.2  # Same 5 Hz feedback target as the service.
                if now >= next_input:
                    control.update({**body,'session_id':session})
                    next_input = now+.025
                control._cycle()
                time.sleep(.005)
            outcomes.append({'stage':name,'moved_deg':max(abs(a-b) for a,b in zip(actual,before)),
                             'paused':control.status()['paused_reason'],
                             'targets': len(sent)-sent_start,
                             'target_unchanged': list(target) == target_before,
                             'distinct_targets': len({tuple(s['target']) for s in sent[sent_start:]})})
        print(json.dumps({'stages': outcomes, 'pauses': pauses}), flush=True)
        assert all(o['moved_deg']>.1 for o in outcomes[:5]),outcomes
        # At 5 Hz, reusing one position step per feedback would yield <= 9
        # distinct targets in each 1.5 s stage. Require continuous advancement.
        assert all(o['distinct_targets'] > 12 for o in outcomes[:5]), outcomes
        assert outcomes[-1]['paused'] == 'motion_no_progress',outcomes
        assert control.is_armed() and robot.status()['enabled_latch']=='confirmed'
        assert any(e.get('action')=='hold' for e in events)
        release_events = [e for e in events if e.get('type') == 'robot_release_stop']
        assert not release_events, release_events
        release_stage = next(o for o in outcomes if o['stage'] == 'release')
        assert release_stage['targets'] == 0 and release_stage['target_unchanged'], release_stage
        assert any(e.get('action') == 'release' and e.get('hardware_command_sent') is False for e in events)
        assert node.health()['max_target_lead_deg'] is None
        assert node.health()['target_lead_limit_enabled'] is False
        assert control.status()['max_target_gap_deg'] is None
        assert max(s['feedback_gap'] for s in sent) > 10, 'stalled fixture must exercise disabled lead cap'
        assert all(math.isfinite(q) and low-1e-6 <= q <= high+1e-6
                   for s in sent for q, (low, high) in zip(s['target'], COMMAND_LIMITS))
        # Prove the tighter proximity setting still halts for real geometry
        # overlap, using a synthetic obstacle ONLY in isolated ROS domain 196.
        update_body = {'keys':[], 'session_id':session}
        control.update(update_body)
        publisher = node.create_publisher(PlanningScene,'/planning_scene',1)
        obstacle_scene = PlanningScene()
        obstacle_scene.is_diff = True
        obstacle = CollisionObject()
        obstacle.header.frame_id = 'base_link'
        obstacle.id = 'offline_collision_test'
        obstacle.operation = CollisionObject.ADD
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [2.,2.,2.]
        pose = Pose()
        pose.orientation.w = 1.
        obstacle.primitives = [shape]
        obstacle.primitive_poses = [pose]
        obstacle_scene.world.collision_objects = [obstacle]
        deadline = time.monotonic()+3
        collision_stopped = False
        while time.monotonic()<deadline:
            publisher.publish(obstacle_scene)
            robot.read_state()
            control.update({'keys':['w'],'session_id':session})
            control._cycle()
            if str(control.status()['paused_reason']).startswith('moveit_servo_5'):
                collision_stopped = True
                break
            time.sleep(.1)
        assert collision_stopped, control.status()
        report = {'stages':outcomes,'targets':len(sent),
                  'max_target_feedback_gap_deg':max(s['feedback_gap'] for s in sent),
                  'max_target_simulated_pose_gap_deg':max(s['gap'] for s in sent),
                  'hold_events':sum(e.get('action')=='hold' for e in events),
                  'release_sends_no_target_verified':True,
                  'loaded_model_limits_rad':loaded_limits,
                  'firmware_joint_limits_verified':True,
                  'linear_speed_m_s':control.LINEAR_SPEED_M_S,
                  'angular_speed_rad_s':control.ANGULAR_SPEED_RAD_S,
                  'http_speed_limits_verified':True,'collision_stop_verified':collision_stopped,
                  'hardware_io':False,'result':'passed'}
        (ROOT/'../runtime/control-loop-offline.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report),flush=True)
    finally:
        control.disarm('offline_test',disable_robot=False)
        control.close(); robot.close()
        server.shutdown(); server.server_close(); http_thread.join(timeout=2)
        process.send_signal(2)
        try: process.wait(timeout=8)
        except subprocess.TimeoutExpired: process.terminate()
        rclpy.shutdown(); thread.join(timeout=2); log.close()

if __name__ == '__main__': main()
