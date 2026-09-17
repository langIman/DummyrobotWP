# Windows 串口接收卡死修复

## 已确认原因

人工操作机械臂复现后，22:07:04 的运动事务在 `motion_backend.exchange` 中
给 `Serial.timeout` 赋值时阻塞。抓栈显示 Windows `GetCommState` 没有返回；
此时 `write` 已经返回，但运动确认尚未读到。桥接等待超时后结束工作进程，
控制服务失去反馈和确认，退出控制；后续 STOP/DISABLE 因串口打开失败而未发送。

该证据确认了不必要的串口重配置路径，不足以判断底层不返回是 Windows 驱动还是
设备固件问题。软件退出控制不等于硬件已经成功失能。

原始证据：`runtime/serial-diagnostic-20260916-220245/stack-96141.txt`、
`transport-captured.jsonl` 和 `FINDINGS.md`。

## 修改范围

- `windows_dummy_bridge/device.py`：打开串口前固定 `timeout=0`，保持原有写超时。
- `windows_dummy_bridge/motion_backend.py`：按可读字节数读取；空闲时最多休眠 1 ms，
  用单调时钟限制整个回复等待，不在热循环修改串口属性。
- `windows_dummy_bridge/bridge.py`：健康接口暴露 `nonblocking_poll_v1`，便于确认
  实际运行进程已加载新实现。
- 不修改固件、MoveIt 模型、速度、角度差保护或使能规则。不自动重发运动命令。

## 验证

174 项离线测试通过。新增测试覆盖串口一次性非阻塞初始化、延迟分段确认、
完整换行、静默超时、残留回复隔离、部分回复保留、部分写入和零等待；
假串口禁止任何运行时 `timeout` 赋值，防止同类重配置问题回归。

桥接与感知服务已重启，健康接口确认新读取策略生效。重连控制 USB 后，
实机 `#GETJPOS` 成功返回。

重连后连续只读测试 150.01 秒，请求频率 50 Hz，实际 47.18 Hz，共 7,078 次；
全部获得完整六轴回复，零错误、零超时。HTTP 往返中位耗时 5.94 ms，
P95 25.62 ms，最大 48.08 ms。修复前同样 150 秒、请求 50 Hz 的测试实际为
36.49 Hz，中位耗时 20.62 ms。此处是独立角度读取吞吐，不是实机运动输出频率。
报告：`runtime/serial-diagnostic-20260916-222225/report.json`。

修复后 5 秒活动栈采样未出现串口属性重配置路径：
`runtime/serial-read-profile-after-fix.txt`。该采样没有包含全部休眠样本，
不能直接与此前包含休眠的样本总数比较 CPU 占比。

实机运动仍由操作者手动验收；自动测试及读取压力测试不发送使能或运动命令。
