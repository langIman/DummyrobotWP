$ErrorActionPreference = 'Stop'
$distro = if ($env:DUMMY_MOVEIT_WSL_DISTRO) { $env:DUMMY_MOVEIT_WSL_DISTRO } else { 'SmolVLA-Lab-Recovered' }
$rosDistro = if ($env:DUMMY_MOVEIT_ROS_DISTRO) { $env:DUMMY_MOVEIT_ROS_DISTRO } else { 'jazzy' }
$motionRoot = $PSScriptRoot
$motionPath = $motionRoot -replace "'", "'\\''"
$motionWsl = (& wsl.exe -d $distro -- bash -lc "wslpath -a '$motionPath'").Trim()
if ($LASTEXITCODE -ne 0) { throw "Could not start WSL distribution: $distro" }
if (-not $motionWsl) { throw 'Could not access the MoveIt workspace through WSL.' }
$command = @'
set -e
test -f '/opt/ros/__ROS_DISTRO__/setup.bash'
source '/opt/ros/__ROS_DISTRO__/setup.bash'
source '__MOTION_ROOT__/.ros/install-resources/setup.bash'
python3 '__MOTION_ROOT__/run_safe_launch.py' &
launch_pid=$!
cleanup() {
  kill "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
set +e
python3 '__MOTION_ROOT__/moveit_servo_gateway.py'
gateway_status=$?
trap - EXIT INT TERM
cleanup
exit "$gateway_status"
'@
$command = $command.Replace('__ROS_DISTRO__', $rosDistro).Replace('__MOTION_ROOT__', $motionWsl)
& wsl.exe -d $distro -- bash -lc $command
