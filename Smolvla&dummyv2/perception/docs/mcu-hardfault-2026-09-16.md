# 23:00 实机 HardFault：浮点格式化工作区损坏

## 结论

本次通信中断捕获到了主控 HardFault。异常指令在 newlib 浮点转字符串所用的 Bigint 分配路径中，访问了损坏的空闲链表指针。当前任务为 OledTask；被挂起的 UsbServerTask 同时处于六轴角度浮点格式化路径，两者使用同一个 `_reent` 工作区。

**已证实：主控异常、出错指令、损坏的指针、OLED 与 USB 格式化重入并共享工作区。**

**高度怀疑：未隔离的 newlib 工作区在多任务抢占中造成损坏。** 静态快照不能还原最早写坏指针的那一次写操作，因此不能宣称已经排除其他越界写、堆管理并发或栈问题，也不能把所有此前故障都归为同一次原因。

本轮只有被动日志观察、ST-LINK HotPlug 读取和离线分析；没有使能、运动、写内存、暂停内核、复位或烧录。诊断完成不等于固件已经修复。

## 现场与时间线

现场目录：`runtime/mcu-fault-20260916-230024-930333/`。

- 事务 47052：`!HAND_DIS` 正常确认，反馈 -249.032806°。
- 事务 47054：`#GETJPOS` 3.52 ms 正常返回 `ok 15.20 47.08 79.32 -4.26 42.49 38.80`。
- 事务 47056：下一次 `#GETJPOS` 超时，无回复。
- 事务 47058：复读 worker 超过截止时间，HTTP 事务耗时 4535.4 ms；自动抓取触发于 2026-09-16 23:00:24.913（北京时间）。
- 保存两份 128 KiB SRAM、一份 64 KiB CCM、两份异常寄存器记录，以及桥接／服务日志。两份 SRAM 的关键损坏值及当前任务一致。

Flash 反汇编使用备份 `firmware_backups/dummy-mainboard-20260915-175818/flash-read1.bin`，SHA-256：

`F52D9209FE2EECA72B6EB0235E6C5446C2A326B2A0F1A6F4ECAF6C590076C5AF`

本次复现前已读取实机 Flash 并确认与该备份哈希相同。参考源码与实机版本并非完全相同，以下实机地址由该二进制及 RAM 推导，不依赖参考源码符号地址。

## CPU 证据

| 项目 | 值 | 含义 |
| --- | --- | --- |
| ICSR | `0x04446803` | 当前活动异常号为 3：HardFault |
| CFSR | `0x00008200` | 精确数据总线错误，BFAR 有效 |
| HFSR | `0x40000000` | 可配置故障升级为 HardFault |
| BFAR | `0x32333937` | 被访问的非法地址 |
| 异常帧位置 | `0x1000E408` | 保存的 R0 与 BFAR 相等 |
| 保存的 PC | `0x080345E4` | `ldr r2, [r0]`，直接访问上述地址 |
| 保存的 LR | `0x08030961` | 调用点 `0x0803095C`，调用 `0x080345D4` |

MMFAR 恰好也是相同数值，但 MMARVALID 未置位，不把它作为有效证据。

实机 HardFault 向量指向 `0x0800611C`，该处为跳回自身的死循环。ST-LINK 输出的 `Core is Running` 仅表示内核没有被调试器暂停，不表示业务程序还在正常运行。

异常帧是从内存扫描定位的，没有通过暂停 CPU 读取 PSP；其指令、R0、BFAR、当前任务栈位置与调用点一致，因而具有很高置信度。浮点扩展现场存在，不能把基本异常帧之后的 32 字节直接当作普通调用栈。

## 损坏位置与函数识别

实机 `0x080345D4` 的函数逐步执行：从工作区 `+0x44` 读取空闲链表、按 `k` 取节点、读取节点的 next；链表不存在时分配 33 个指针；新节点大小按 `1 << k` 计算。其相邻释放函数也将节点插回对应链表，与 newlib `Balloc/Bfree` 源码结构吻合。函数名是依据反汇编与源代码匹配识别，不是来自该固件的 ELF 调试符号。

RAM 链路：

```text
全局工作区指针 0x200007C0 -> 0x200007C8
工作区 + 0x44           -> 0x20007CB8（freelist）
freelist[0]             -> 0x32333937（非法节点地址）
指令 0x080345E4         -> 读取该非法地址，触发异常
```

`0x32333937` 的小端字节恰好是 ASCII `7932`。这与数字文本污染链表相符，但仅凭四个字节不能证明是哪次格式化写入的，也不是“内存耗尽”的证明。

## 为什么定位到 OLED 与 USB 的并发格式化

1. PendSV 实机代码从 `0x20002B38` 读取当前任务指针；两份 SRAM 均为 `0x1000E818`。
2. 该 TCB 名为 `OledTask`，栈起点 `0x1000E040`，异常帧处于它的栈内。
3. OLED 栈内保存的返回点 `0x08013429` 对应调用 `0x08001A74`。调用前加载 Flash 字符串 `IMU:%.3f/%.3f`。该函数进一步调用 `0x0802F9CC`（与 Print.printf / vsnprintf 路径相符）。栈缓冲区还保留 `IMU:0.035`。
4. 格式化内部调用 `0x080306F0`，再调用上述 Bigint 分配函数；与 `vfprintf -> dtoa -> Balloc` 路径相符。
5. `UsbServerTask` 的 TCB 位于 `0x1000CF58`，保存的任务栈顶为 `0x1000C8AC`。上下文中的 EXC_RETURN 为 `0xFFFFFFED`；按 FreeRTOS 的软件保存区及浮点区布局恢复后，保存的 PC 为 `0x0802C5F2`，位于同一格式化函数内。
6. USB 栈保存字符串 `ok %.2f %.2f %.2f %.2f %.2f %.2f`，同时保存工作区指针 `0x200007C8`；OLED 栈也保存此指针。证明本次两条格式化调用尚未结束且共享状态。
7. 实机 vsnprintf 路径从全局 `0x200007C0` 取工作区；已识别的任务切换函数只切换当前 TCB，没有切换该工作区。

参考源码也存在相同风险：`Core/Inc/FreeRTOSConfig.h` 未启用 `configUSE_NEWLIB_REENTRANT`，其 FreeRTOS 头文件默认值为 0；`UserApp/main.cpp` 的 OLED 任务打印浮点数，`Bsp/communication/ascii_processor.hpp` 的 Respond 使用 snprintf。此处用于印证设计风险，不假定参考版本与实机完全相同。

## 对当前控制故障的解释

主控在 HardFault 中循环，不能继续处理 USB 请求；电脑侧继而遇到无回复、worker 截止时间、重连失败。网页不能靠提高速度、改变奇异阈值或增加重试修复已经崩溃的主控。

这也解释为什么软件随后发送 STOP／DISABLE 不一定收到确认。**界面退出控制不等于硬件已经失能**；故障时驱动的保持／使能状态不能从本快照推断。现场已经保存，无须为了保留这次证据继续维持故障状态。

本次没有收到此前的 C++ terminate 文本，不能直接断言此前 terminate 与这次 HardFault 的具体异常类型相同。

## 修复路线（尚未实施或烧录）

优先在与实机相匹配的固件工程中修复 newlib 的多任务使用方式：每任务独立 reent 工作区，补齐 malloc/free 的互斥与必要的初始化／回收；检查任务栈余量、堆边界和中断上下文中的库调用。仅启用一个 FreeRTOS 宏不能替代完整的堆线程安全检查。

另一种有针对性的简化方案是移除 OLED 的 `%f` 格式化，用不依赖浮点字符串转换的实现显示数值，并审计其他线程的格式化调用。它能移除本次已证实的一组重入路径，但在其他路径未审计前不能称为完整修复。

修复验收应先在不发送运动命令的情况下，让屏幕刷新与高频角度读取持续并发，监测回复完整性及故障寄存器，再由现场人员进行运动验证。通过有限时长测试只能提高置信度，不能证明永不发生。

目前没有编译、烧录新固件，也没有改动现有运动参数。需要匹配的实机源码／构建信息来避免用参考版本覆盖已有功能；不能为了试这个问题直接盲刷不匹配的版本。

## 离线复核与依据

已经对两份 SRAM、异常指令字节、当前 TCB、OLED 调用点、USB 保存上下文、六轴格式字符串、共享工作区指针做交叉断言检查，全部通过。原始现场及哈希保存在该目录的 `manifest.json`。

- [ST PM0214：Cortex-M4 异常与故障寄存器](https://www.st.com/resource/en/programming_manual/pm0214-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf)
- [newlib 官方文档：Reentrancy](https://sourceware.org/newlib/libc.html#Reentrancy)
- [newlib 源码镜像：mprec.c](https://raw.githubusercontent.com/mirror/newlib-cygwin/master/newlib/libc/stdlib/mprec.c)
- [newlib 源码镜像：dtoa.c](https://raw.githubusercontent.com/mirror/newlib-cygwin/master/newlib/libc/stdlib/dtoa.c)

源码镜像用于识别算法和结构；不把其当前版本当作实机所用库的确切版本。
