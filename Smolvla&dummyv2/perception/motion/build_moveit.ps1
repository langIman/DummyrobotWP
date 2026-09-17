$ErrorActionPreference = 'Stop'
$distro = if ($env:DUMMY_MOVEIT_WSL_DISTRO) { $env:DUMMY_MOVEIT_WSL_DISTRO } else { 'SmolVLA-Lab-Recovered' }
$rosDistro = if ($env:DUMMY_MOVEIT_ROS_DISTRO) { $env:DUMMY_MOVEIT_ROS_DISTRO } else { 'jazzy' }
$motionRoot = $PSScriptRoot
$sourceRoot = (Resolve-Path (Join-Path $motionRoot '..\..\..\dummy_moveit_ws')).Path
$resourceCmake = (Resolve-Path (Join-Path $motionRoot 'dummy_moveit_config.resources.cmake')).Path
$motionPath = $motionRoot -replace "'", "'\\''"
$sourcePath = $sourceRoot -replace "'", "'\\''"
$resourceCmakePath = $resourceCmake -replace "'", "'\\''"
$motionWsl = (& wsl.exe -d $distro -- bash -lc "wslpath -a '$motionPath'").Trim()
if ($LASTEXITCODE -ne 0) { throw "Could not start WSL distribution: $distro" }
$sourceWsl = (& wsl.exe -d $distro -- bash -lc "wslpath -a '$sourcePath'").Trim()
if ($LASTEXITCODE -ne 0) { throw 'Could not access the read-only MoveIt source workspace through WSL.' }
$resourceCmakeWsl = (& wsl.exe -d $distro -- bash -lc "wslpath -a '$resourceCmakePath'").Trim()
if ($LASTEXITCODE -ne 0) { throw 'Could not access the resource build template through WSL.' }
if (-not $motionWsl -or -not $sourceWsl -or -not $resourceCmakeWsl) { throw 'Could not translate one or more WSL paths.' }
$command = @"
set -e
test -f '/opt/ros/$rosDistro/setup.bash'
source '/opt/ros/$rosDistro/setup.bash'
mkdir -p '$motionWsl/.ros/shadow/dummy-ros2_description' '$motionWsl/.ros/shadow/dummy_moveit_config'
cp -a '$sourceWsl/dummy-ros2_description/.' '$motionWsl/.ros/shadow/dummy-ros2_description/'
cp -a '$sourceWsl/dummy_moveit_config/.' '$motionWsl/.ros/shadow/dummy_moveit_config/'
cp '$resourceCmakeWsl' '$motionWsl/.ros/shadow/dummy_moveit_config/CMakeLists.txt'
mkdir -p '$motionWsl/.ros/build-resources' '$motionWsl/.ros/install-resources' '$motionWsl/.ros/log'
colcon --log-base '$motionWsl/.ros/log' build \
  --base-paths '$motionWsl/.ros/shadow' \
  --build-base '$motionWsl/.ros/build-resources' \
  --install-base '$motionWsl/.ros/install-resources' \
  --packages-up-to dummy_moveit_config
"@
& wsl.exe -d $distro -- bash -lc $command
if ($LASTEXITCODE -ne 0) { throw "MoveIt build failed with exit code $LASTEXITCODE" }
