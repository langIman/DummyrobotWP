# Dummy Windows 桥接 V4

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

Python 3.13.5 独立 `.venv`，唯一依赖 pyserial==3.5，无需 Docker/WSL/ROS。复用原 USB 驱动和 SSH config 别名 frp-era.com。桥接不可与 DummyStudio/舞蹈程序同时占用串口。

2026-09-09 23:12（北京时间）已实际启动 V4，服务器 health/capabilities 验证版本及实例一致。当前仅枚举到 COM15/COM16 蓝牙端口，未发现 Dummy USB；state 如实返回 device_not_found、positions=null。接好设备后点击网页“读取角度”即可重新尝试连接。本轮未发运动、START、CMDMODE 或 STOP。

已有虚拟环境可直接启动；缺失时重建：

```powershell
D:\anaconda\python.exe -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```
