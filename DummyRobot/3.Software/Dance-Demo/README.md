# Dummy Robot 低速舞蹈示例

这个程序通过 USB 串口控制六个关节。它会先校验开机读数，逐轴做小幅自检，等待人工确认后执行一轮大幅六轴舞蹈，最后回到折叠参考姿态并发送 `!DISABLE`。

## 运行前

1. 关闭 DummyStudio，它会占用同一个串口。
2. 断开 12 V 电源，把机械臂手动摆到实际的折叠收纳姿态，再通电。
3. 固定底座并清空机械臂四周至少 50 cm 的运动范围，手放在 12 V 插头旁，随时准备断电。
4. 电机已经发热或故障灯按组重复闪烁时不要运行。

双击 `run_dance.bat`，按提示先输入 `FOLD`。程序会只使能约 1.2 秒后自动失能；确认这段时间没有两连闪或三连闪，再输入 `TEST`。六轴逐轴自检完成且现场确认正常后，最后输入 `RUN`。

任何时候按 `Ctrl+C` 都会尝试发送 `!STOP` 和 `!DISABLE`。如果机械臂碰撞、剧烈抖动或持续堵转，直接拔掉 12 V 电源，不要等待软件响应。

## 命令行选项

只查看动作、不连接机器人：

```powershell
.\.venv\Scripts\python.exe .\3.Software\Dance-Demo\dummy_dance.py --dry-run
```

指定串口、速度和循环次数：

```powershell
.\.venv\Scripts\python.exe .\3.Software\Dance-Demo\dummy_dance.py --port COM7 --speed 30 --cycles 1
```

正式舞蹈的默认速度为 30，允许范围为 1 到 30；逐轴自检和最后折叠固定不超过 4。循环次数限制为 1 到 3。
