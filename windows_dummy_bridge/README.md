# Dummy Windows 桥接 V4

2026-09-16 串口卡死修复：串口打开时一次性设置非阻塞读取，接收循环仅查询可读字节，
无数据时最多休眠 1 ms，等待总时长仍由请求的单调时钟截止时间限制。接收期间不再赋值
`Serial.timeout`，避免每次读取触发 Windows `GetCommState/SetCommState` 重配置。
`GET /health` 的 `serial_read_strategy=nonblocking_poll_v1` 表示此修复已经加载。
保留完整回复校验、旧回复单独记录、部分写入报错，以及运动命令超时不自动重发。
此修复针对已抓栈确认的阻塞路径，不代表任何 USB 或设备故障都能自动恢复。

2026-09-16：`#GETJPOS` 收到完整换行和六轴有限数值后立即返回；未完成的回复仍等待
指定超时。结构化运动继续等有效确认，其他 ASCII 命令的等待语义不变。响应的
`read_ended_on_joint_reply` 和 `serial_elapsed_ms` 可用于核验读取耗时。

## 启动

双击 **start.cmd**，自动启动桥接、SSH 隧道并打开通信网页。CMD 中也可直接执行：

```bat
cd /d <仓库目录>\windows_dummy_bridge
start.cmd
```

无需解锁、会话或心跳续租。重复启动复用现有 V4。关闭启动窗口不关闭后台进程。

| 命令 | 用途 |
|---|---|
| start.cmd | 一键启动并打开网页 |
| start.cmd -LocalOnly | 只启动本地桥接和网页 |
| start.cmd -NoBrowser | 启动但不打开浏览器 |
| status.cmd | 查看 bridge health |
| stop.cmd | 关闭本项目通信进程并释放 USB，不发送机械臂 STOP |
| .\start_bridge.ps1 | PowerShell：单独启动桥接；支持 -Foreground |
| .\start_tunnel.ps1 | PowerShell：单独启动 SSH 隧道 |
| .\unlock_bridge.ps1 | 旧入口只提示 V4 无需解锁，无其他作用 |

网页：**http://127.0.0.1:8765/**。显示最近 100 条串口事务、错误、原始十六进制和最近读取的六轴角度。自动刷新只访问日志；点击“读取角度”才发送一次 #GETJPOS。不是连续传感器采样。

## 常用 PowerShell 请求

```powershell
Invoke-RestMethod http://127.0.0.1:8765/health
Invoke-RestMethod http://127.0.0.1:8765/capabilities | ConvertTo-Json -Depth 8
Invoke-RestMethod http://127.0.0.1:8765/state | ConvertTo-Json -Depth 8
Invoke-RestMethod -Method Post http://127.0.0.1:8765/command -ContentType application/json -Body '{"command":"#GETJPOS","read_timeout_ms":500}'
```

确实需要停止机械臂时显式执行：

```powershell
Invoke-RestMethod -Method Post http://127.0.0.1:8765/stop -ContentType application/json -Body '{}'
```

STOP 不锁定、不失能。退出桥接、SSH 断线、HTTP 超时均不自动 STOP，已被固件收到的目标可能继续执行。/motion 不自动 START 或 CMDMODE；调用方通过 /command 显式发送。桥接只校验六轴有限数值及单位，不增加角度或速度范围限制。完整契约见 [bridge_handoff.md](bridge_handoff.md)。

日志：`runtime/transport_trace.jsonl`，2MiB 轮转，保留 3 个备份。V3 源码和旧测试归档于 `archive/v3_20260909_225811/`；旧 token/session 文件无效，V4 不读取。

## 环境与本次部署

Python 3.13.5 独立 `.venv`，依赖 pyserial 和 opencv-python-headless，无需 Docker/WSL/ROS。复用原 USB 驱动和 SSH config 别名 frp-era.com。桥接不可与 DummyStudio/舞蹈程序同时占用串口。

2026-09-09 23:12（北京时间）已实际启动 V4，服务器 health/capabilities 验证版本及实例一致。当前仅枚举到 COM15/COM16 蓝牙端口，未发现 Dummy USB；state 如实返回 device_not_found、positions=null。接好设备后点击网页“读取角度”即可重新尝试连接。本轮未发运动、START、CMDMODE 或 STOP。

已有虚拟环境可直接启动；缺失时重建：

```powershell
D:\anaconda\python.exe -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

摄像头已并入 V4：camera_backend.py 在独立进程中读取 Windows UVC 摄像头 0，网页可修改分辨率、请求帧率和 JPEG 质量；/camera/status 查询状态、/camera/frame 返回单张 JPEG、/camera/stream 提供最高 30 FPS 的连续 MJPEG 预览。摄像头故障不会影响 Dummy 串口。2026-09-14 已修复黑帧和管道并发问题，网页确认有正常 1280×720 图像。

网页支持 1280×720 / 1920×1080、30 / 60 / 120 FPS，以及 40–95 的 JPEG 质量。点击“应用配置”只重启摄像头子进程，不重启串口桥接；成功后写入 camera_config.json，下次启动继续使用。也可直接调用：

```powershell
Invoke-RestMethod http://127.0.0.1:8765/camera/config
Invoke-RestMethod -Method Post http://127.0.0.1:8765/camera/config -ContentType application/json -Body '{"width":1280,"height":720,"fps":30,"jpeg_quality":85}'
```


摄像头修复记录（2026-09-14）：
- status/frame 曾同时读取同一 multiprocessing Pipe 导致 AssertionError；现在同一摄像头进程的请求及关闭使用独立 RLock 串行化，不占用 Dummy USB 锁。
- 已完成常用模式矩阵测试。该驱动要求依次设置 width、height、fps，最后设置 MJPG；若在 MJPG 后再设置尺寸或帧率，会切回 YUY2 并产生全黑帧。桥接固定请求 1280×720 @ 30 FPS，正式采集进程稳定实测约 30 FPS。
- GET /camera/status 返回 requested_mode、driver_mode、actual_mode、property_order、dimensions、frame_id、captured_at_ms 和 frame_age_ms；actual_mode.measured_fps 是约 4 秒滑动窗口实测值，驱动报告的 FPS 不等于真实帧率。
- GET /camera/frame 返回 image/jpeg，附 X-Frame-Id、X-Captured-At-Ms。无有效帧返回 HTTP 503 JSON；读失败清除旧图，超过 2 秒的缓存图不作为有效帧返回。
- GET /camera/stream 返回 multipart/x-mixed-replace MJPEG，网页使用单个长连接连续显示，预览最高 30 FPS；摄像头配置为 60/120 FPS 时仍只推送最新帧，避免无意义地占满本机和隧道带宽。
- 并发执行 status/frame/health 的 8 个只读请求全部成功，4 张 JPEG 均解码为 1280×720、非黑图；网页截图已确认真实画面。网页预览约 1 次/秒，与相机实际采集频率不同。
- 与桥接相同的 JPEG 编码负载下，1280×720 请求 15/30/60/120 时约为 30/30/60/87 FPS；1920×1080 时约为 30/30/39/37 FPS。请求 15 FPS 会被设备实际按约 30 FPS 输出；高帧率模式曝光更短、画面明显更暗。640×360 和 640×480 请求会被驱动改成 1280×720，需由消费端缩放。吞吐原始记录见 runtime/camera_throughput_1789395542734/report.json，属性顺序记录见 runtime/camera_order_1789394956420/report.json。
- GET /camera/config 返回当前及支持配置；POST /camera/config 接受 width、height、fps、jpeg_quality，允许省略未修改字段。未知字段或无效数值返回 HTTP 422，配置失败返回 HTTP 503，并保留上一个已保存配置。
- 桥接及 SSH 隧道已重启加载修复。本轮没有向 Dummy 发送 START、运动或 STOP。
