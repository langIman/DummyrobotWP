# DummyV2 感知与数据采集工作台

Windows 本地双相机感知服务、DummyV2 控制台和键盘遥操作采集页面。
当前版本为 **0.8.18**，本阶段不运行 SmolVLA 推理。

实现、启动说明、接口和历史记录见 [perception/README.md](perception/README.md)。

## 当前控制方式

- WASD：沿末端自身前、左、后、右平移。
- ↑/↓：沿末端自身上下平移。
- 8/2/4/6：向末端自身上下左右转向。
- 0 / `.`：按住闭合 / 张开夹爪。
- 使能按钮同时使能六轴与夹爪；退出实机控制时一起失能。

平移设定为每轴 5 cm/s，转向上限 0.24 rad/s，公共运动速度参数 50，准备位速度参数 12。
保留约 105 mm 的抓取点补偿；该安装偏移为实测估值，并非精确 TCP 标定。

## 运行依赖与仓库范围

此目录保存感知服务、网页、MoveIt 适配层、测试和诊断文档。
当前部署还依赖同一工作区下的 `windows_dummy_bridge` 串口桥接，以及
`dummy_moveit_ws` 的机器人模型和运动学资源；这两个外部目录不包含在此目录中。
Windows 启动器使用本机路径，迁移机器时需核对路径、WSL 发行版与设备配置。

在 `langIman/DummyrobotWP` 仓库中，本项目保存在 `Smolvla&dummyv2/` 子目录，配套
桥接保存在仓库根目录的 `windows_dummy_bridge/`。恢复目前部署结构时，将仓库克隆到
`D:\DummyRobot_workplace\dummy_moveit_ws`，再将仓库中的 `Smolvla&dummyv2` 和
`windows_dummy_bridge` 两个目录分别复制到 `D:\DummyRobot_workplace` 下，成为
与 `dummy_moveit_ws` 同级的目录；随后按照感知服务和桥接各自的 README 安装环境。

录制数据、运行日志、虚拟环境、MoveIt 构建产物、实机固件备份和本机 `config.json`
不纳入版本控制。初次部署请参考 `perception/config.example.json` 和启动说明。

服务地址为 `http://127.0.0.1:8770/`，采集页为 `/collect`。
启动不会自动使能或运动；实机操作使用页面中的使能和长按确认。
