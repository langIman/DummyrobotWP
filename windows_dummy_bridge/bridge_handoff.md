# Dummy Windows bridge — 服务器交接 V4

更新：2026-09-09，Asia/Shanghai。按用户授权的 Windows桥接简化Prompt.md 实现普通通信通道，并提供 CMD 启动和中文通信网页。

## a. 文件、启动和实际部署

根目录：`D:\DummyRobot_workplace\windows_dummy_bridge`。可信仓库：`D:\DummyRobot_workplace\DummyRobot`。

| 文件 | 用途 |
|---|---|
| bridge.py | V4 HTTP、USB 子进程、格式校验、日志和网页服务 |
| device.py | 按已知身份打开 USB CDC，无初始化写入 |
| motion_backend.py | 单次写入、原始回复收集，无权限或运动状态机 |
| dashboard.html | 中文通信观察台 |
| start.cmd / launch.ps1 | 一键启动桥接、SSH 隧道并打开网页 |
| stop.cmd / stop_bridge.ps1 | 仅关闭本项目通信进程，释放 USB，不发送 STOP |
| status.cmd | 查询 health |
| start_bridge.ps1 / start_tunnel.ps1 | 分别启动桥接与隧道 |
| unlock_bridge.ps1 | 旧入口仅提示 V4 无需解锁 |
| verify_remote.py | 服务器侧仅 GET health/capabilities/state |
| requirements.txt | pyserial==3.5 |
| runtime/transport_trace.jsonl | V4 原始事务日志，2MiB，3 个轮转备份 |
| archive/v3_20260909_225811/ | V3 源码、文档、旧测试和现场工具备份 |

双击 start.cmd 或 CMD 执行：

```bat
cd /d D:\DummyRobot_workplace\windows_dummy_bridge
start.cmd
```

可加 -NoBrowser 或 -LocalOnly。PowerShell 运行 `.\launch.ps1`，或依次 `.\start_bridge.ps1`、`.\start_tunnel.ps1`。前台运行支持 `.\start_bridge.ps1 -Foreground`。关闭窗口不会关后台；stop.cmd 仅关闭通信。

**真实部署快照：2026-09-09 23:12 北京时间，V4 已启动。** 启动器 bridge PID 17936、实际监听 Python 子进程 PID 26692、SSH PID 27116；以实际进程和 runtime PID 为准。远端 health 实例 `76cf7dd3-a414-43e6-b72f-61f06e0eb165` 与本地一致，protocol_version=4、mode=transport。证据：`runtime/v4_deployment_verification.json`。

本轮 USB 枚举仅 COM15/COM16 蓝牙端口，未发现已知 Dummy，所以 state 为 error、positions=null、transaction.error 为 open_error / OSError: device_not_found。HTTP/SSH 可用不等于真机已接通；接好设备后下一次显式 /state 会再次尝试打开，无需重启或解锁。

网页 http://127.0.0.1:8765/ 每秒 GET /monitor，被动显示最近 100 条事件、最近角度和时间。点击“读取角度”才 GET /state。关闭网页不影响机械臂。网页不宣称检测到真实使能或实时 SSH 状态，隧道验证结果在启动窗口。收发记录区分 TX 尝试、实际 bytes_written、sent、原始回复和错误。

## b. 环境和 USB / SSH

- Windows Python 3.13.5，基础解释器 D:\anaconda\python.exe，独立 .venv；依赖 pyserial 3.5。
- 115200 baud、8N1、ASCII，USB CDC Microsoft usbser；未更换驱动/刷固件。历史驱动版本 10.0.22621.5415。
- 已知 VID/PID 1209:0D32、序列号 347933853335，历史 COM7；按身份查找，允许端口号变化。打开前 DTR/RTS=false，不主动重置或初始化。
- 仅实现 CDC ASCII。另有 Native REF/WINUSB 和 Fibre SDK，但没有接入本版 HTTP，任意 ASCII 不等于全部 USB/Fibre 方法。
- 历史已确认 #GETJPOS、!START、!STOP、> 六轴目标。当前固件 CMDMODE 回复曾为 `ok Set command mode to [2]`，仓库为 `Set command mode to [2]`；V4 /command 原样返回，不自行切模式。
- 源码 J2 下限 -73 与用户纠正实际 -75 的差异只作说明；V4 不在桥接检查该范围，也未修改固件。

服务器入口 http://127.0.0.1:18765，Windows http://127.0.0.1:8765。复用 SSH alias frp-era.com（langIman、61537、已有密钥配置），不读取或输出私钥。

```text
ssh -N -T -o BatchMode=yes -o ConnectTimeout=10 -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -R 127.0.0.1:18765:127.0.0.1:8765 frp-era.com
```

只监听回环；隧道脚本检查远端监听和 health 实例一致。SSH 错误见 runtime/tunnel.stderr.log，不修改 sshd 权限或 VS Code 会话。SSH 保活只维持网络，不关联运动权限。

## c. HTTP 契约

UTF-8 JSON。POST Content-Type: application/json，正文最多 65536 bytes；拒绝 chunked、NaN/Infinity。无 token、session、seq、TTL，也不要求预先 GET /health。拒绝外站浏览器 Origin 和非回环 Host；允许 localhost/127.0.0.1 的 8765、18765，无 CORS。

### GET /health、GET /capabilities

均不操作硬件。

```json
{"status":"alive","protocol_version":4,"mode":"transport","instance_id":"<uuid>","bridge_time_ms":1788966728865,"device_enabled":null}
```

无 motion_enabled/session_id/armed 字段。device_enabled=null 表示未知。
capabilities 返回 protocol_version=4、transport=usb_cdc_ascii、raw_ascii=true、multiline=true、fibre_supported=false、endpoints。limits_informational 仅说明，bridge_range_enforcement=false，不作执行条件。default_speed=100。

### GET /state

只发送 `#GETJPOS\n`，读取 500ms，不先 START。成功格式示例（不是本轮实测）：

```json
{"usb_status":"responsive","feedback_status":"device_reported","positions":[0,-75,180,0,0,0],"unit":"degree","coordinate_space":"hardware_joint","joints":["J1","J2","J3","J4","J5","J6"],"received_at_ms":1788966728865,"device_sample_at_ms":null,"sample_age_ms":null,"device_enabled":null}
```

另含 port/device_id/generation/bridge_time_ms/transaction。无反馈时 positions=null、feedback_status=unavailable，不使用目标/零伪造。HTTP 200 包含 USB 状态，必须检查 usb_status。received_at_ms 为主机窗口结束时间，无硬件采样时间，不能保证设备侧数据新鲜度。

### POST /command

```json
{"command":"#GETJPOS","read_timeout_ms":500}
```

无命令白名单；支持多行 ASCII，原文保留，非空且末尾没有 LF 时仅补一个 LF。空 command 扩展用于收取缓冲回复，不发送字节。不会调用 Python eval/exec、系统 shell 或 Fibre。

read_timeout_ms 为整数 0..2000，默认 500；在完整窗口内收集拼接/无换行的实际回复。命令最多 16384 bytes（含补换行），单次新回复最多 65536 bytes；到上限返回 response_limit_reached=true，剩余数据留在串口缓冲区。这些仅为 I/O 资源预算。

```json
{"sent":true,"bytes_written":9,"write_status":"complete","raw_response":"ok ...\r\n","raw_response_hex":"...","pending_response":"","pending_response_hex":"","read_timed_out":false,"read_wait_skipped":false,"reply_correlation":"unavailable_in_ascii_protocol"}
```

bytes_written 来自实际 write 返回值。sent=true 仅表示全部写入，不等于固件接受或运动到达。写成功但无新回复：sent=true、raw_response=""、read_timed_out=true，HTTP 200，不造 ok。

read_timeout_ms=0 只写不等待新回复，read_wait_skipped=true；后续回复仍可留在缓冲区。下一次发送前，将已到达字节放 pending_response/pending_response_hex，不静默丢弃；新写入后读到的字节放 raw_response。空 command 同样区分旧缓冲和后续数据。ASCII 无请求 ID，晚到旧回复仍可能混入下一窗口，无法严格关联；建议按顺序调用，使用非零窗口，写入结果不确定时不要自动重发。

raw_response/pending_response 用 Latin-1 一一映射字节到 JSON 字符，ASCII 文本完全一致，不 trim、不替换换行；hex 字段为逐字节证据。

### POST /motion

```json
{"positions":[1,-75,180,0,0,0],"speed":100,"unit":"degree","coordinate_space":"hardware_joint"}
```

只校验六个有限数值、有限 speed、单位/坐标。speed 省略为 100，不限定范围。编码 `>j1,j2,j3,j4,j5,j6,speed\n`，最多 15 位有效数字。仅写目标，不自动查询角度、START、CMDMODE；需要时调用方显式 /command 初始化。

500ms 内收到完整行 ok 才返回 accepted=true，execution_complete=false；未确认 accepted=false，并返回原始事务。固件 ok 只是处理确认，部分固件内部拒绝目标仍可能回 ok，服务器须以实际反馈判定运动结果。

不实现 no_op 特判，零位移仍按原协议发送，不附带 STOP。目标替换方式、实际速度与限位由当前固件及模式决定。

### POST /stop {}

只发 `!STOP\n`，读取 500ms；完整行 Stopped ok 才令 acknowledged=true，否则 false。不锁定、不失能，也不更改下一次请求权限。

```json
{"sent":true,"bytes_written":6,"raw_response":"Stopped ok\r\n","stop":{"acknowledged":true}}
```

另含完整事务字段。确认不是机械停止时间保证。

### 网页和废弃接口

GET / 网页；GET /monitor 返回 health、最近 state、最近 100 条事件、serial_busy、passive=true，不打开串口。
/session、/unlock、/heartbeat、/lock、/validate-motion 返回 HTTP 410 endpoint_removed_in_v4，无副作用。旧 /lock 绝不映射 STOP，服务器删除 V3 授权探测。

### 错误码

| HTTP | 内容 |
|---|---|
| 200 | 有事务结果；仍需检查 sent/accepted/acknowledged/usb_status |
| 400 | json_object_required、invalid_json、invalid_content_length、chunked_not_supported |
| 403 | loopback_host_required、foreign_browser_origin |
| 404 | not_found |
| 410 | endpoint_removed_in_v4 |
| 413 | body_size_invalid、command_too_large |
| 415 | json_required |
| 422 | ascii_text_required、read_timeout_ms_out_of_range、six_finite_axes_required、finite_speed_required、unit_or_coordinate_mismatch、empty_object_required、unknown_fields |
| 503 | io_busy_not_sent，或事务 error：open_error/pending_read_error/write_error/read_error/partial_write/worker_unavailable/pending_buffer_full |
| 500 | internal_error，写入可能未知，不自动重试 |

校验/排队/打开失败均 sent=false、bytes_written=0。部分写入报告实际计数和 write_status=partial；写超时或子进程失联可能已发出部分字节，sent=null、bytes_written=null、write_status=unknown。写成功而读异常时保留 sent=true 及已收到字节。父进程硬超时不能恢复未交付的子进程字节，response_capture_complete=false 明示不完整，不能把空 raw_response 当作确认无数据。

## d. 坐标

Windows 仅硬件角度 degree，J1→J6，不改方向/零点。服务器便捷 ROS 入口只转换一次：

```text
hardware_deg = (ros_rad + [0,0,pi/2,0,0,0]) * [1,1,1,1,-1,-1] * 180/pi
ros_rad = hardware_deg * [1,1,1,1,-1,-1] * pi/180 - [0,0,pi/2,0,0,0]
```

服务器 --space hardware 直接传硬件度数；/command 不转换。来源是 dummy_moveit_ws/dummy_controller/dummy_controller/dummy_arm_controller.py。反馈超出名义范围也不裁剪；不重新现场测映射。

## e. 并发、停止和断线

单一 USB 子进程，写/读串行；最多一个等待请求等待 2.75 秒，更多或超时返回 503 io_busy_not_sent，未发送。不批量积压目标、不替换请求、不重连重放。USB 写超时 250ms，读最大 2000ms，父进程 3200ms 硬期限；必要时退出卡住的通信子进程，无隐含硬件命令。

启动不打开串口、不查询/控制。退出、HTTP 结束/超时、客户端 Ctrl+C、SSH 断开、读超时、USB 故障均不自动 START/STOP/DISABLE/归零。普通 STOP 只在 /stop 或 /command 显式指定时发送。断线不保证机械停止，固件可能继续执行旧目标。

下一次显式硬件请求可重新开设备，但不补发失败命令。health/capabilities/monitor 不触发连接。

## f. 验证和未验证项

- Python/PowerShell 静态语法检查完成。首次启动健康检查发现 handler 工厂缺少返回值，已修复后重新启动，当前 HTTP 正常。
- 真实 V4 两端 health 实例匹配、远端回环监听、capabilities=4 已验证，证据见 runtime/v4_deployment_verification.json。
- 网页已显示真实收发和 device_not_found；被动刷新不写 USB。
- 当前 Dummy USB 未枚举，本轮反馈 unavailable。未发送运动、START、CMDMODE、STOP，未做 V4 真机写入、断线、吞吐测试或新增 mock 测试。
- 原 V3 会话/限位测试与现场工具已归档，verify_remote.py 已改为 V4 只读检查；旧记录不作为 V4 测试结果。
- Fibre 未接入，不宣称支持所有 USB/Fibre 方法。

## 摄像头模块（Windows）

camera_backend.py 在独立进程中读取 OpenCV UVC 设备 index 0；V4 网页显示画面并可修改配置，接口为 GET /camera/status、GET /camera/frame、GET /camera/stream、GET /camera/config 和 POST /camera/config。/camera/stream 是最高 30 FPS 的 multipart MJPEG 长连接，/camera/frame 保留给控制节点按需取单帧。采集进程与 USB 串口进程隔离，摄像头错误或重配置不会影响 Dummy。依赖 opencv-python-headless==4.10.0.84（含 numpy）。2026-09-14 已确认正常图像，未发送 Dummy 控制命令。

配置请求示例：

```json
{"width":1280,"height":720,"fps":30,"jpeg_quality":85}
```

允许分辨率为 1280×720、1920×1080，FPS 为 30、60、120，JPEG 质量为 40–95 整数。字段可部分提交；成功后仅重启摄像头子进程并原子写入 camera_config.json。HTTP 422 表示未知字段、不支持的分辨率/FPS或质量越界；HTTP 503 表示摄像头未能采用配置，旧配置仍保留。


摄像头修复记录（2026-09-14）：
- status/frame 曾同时读取同一 multiprocessing Pipe 导致 AssertionError；现在同一摄像头进程的请求及关闭使用独立 RLock 串行化，不占用 Dummy USB 锁。
- 已完成常用模式矩阵测试。该驱动要求依次设置 width、height、fps，最后设置 MJPG；若在 MJPG 后再设置尺寸或帧率，会切回 YUY2 并产生全黑帧。桥接固定请求 1280×720 @ 30 FPS，正式采集进程稳定实测约 30 FPS。
- GET /camera/status 返回 requested_mode、driver_mode、actual_mode、property_order、dimensions、frame_id、captured_at_ms 和 frame_age_ms；actual_mode.measured_fps 是约 4 秒滑动窗口实测值，驱动报告的 FPS 不等于真实帧率。
- GET /camera/frame 返回 image/jpeg，附 X-Frame-Id、X-Captured-At-Ms。无有效帧返回 HTTP 503 JSON；读失败清除旧图，超过 2 秒的缓存图不作为有效帧返回。
- 并发执行 status/frame/health 的 8 个只读请求全部成功，4 张 JPEG 均解码为 1280×720、非黑图；网页截图已确认真实画面。网页预览约 1 次/秒，与相机实际采集频率不同。
- 与桥接相同的 JPEG 编码负载下，1280×720 请求 15/30/60/120 时约为 30/30/60/87 FPS；1920×1080 时约为 30/30/39/37 FPS。请求 15 FPS 会被设备实际按约 30 FPS 输出；高帧率模式曝光更短、画面明显更暗。640×360 和 640×480 请求会被驱动改成 1280×720，需由消费端缩放。吞吐原始记录见 runtime/camera_throughput_1789395542734/report.json，属性顺序记录见 runtime/camera_order_1789394956420/report.json。
- 网页与 API 已实测切换到 1920×1080 后获得非黑帧，再恢复 1280×720；完整桥接重启后成功加载保存配置。非法 640×480 请求返回 HTTP 422，未覆盖原配置。
- 桥接及 SSH 隧道已重启加载修复。本轮没有向 Dummy 发送 START、运动或 STOP。
