# FSRemote

## 1. 模块作用 / Purpose

FSRemote 是 FS-i6X 遥控器搭配 iA6B 接收机的 XRobot 模块。构造时配置 UART 为
100000 bit/s、8 个有效数据位、偶校验、2 个停止位，并创建 SBUS 接收线程。
模块解码 10 个原始通道，向 CMD 提交摇杆控制量，并通过 `GetEvent()` 提供四个拨杆的位置变化事件。

FSRemote is an XRobot Module for the FS-i6X transmitter and iA6B receiver. It configures
100000 bit/s, 8E2, starts a receive thread, decodes ten channels, feeds stick commands to
CMD and exposes switch position events.

接收采用逐字节流式组帧，支持 UART 分段到达、连续帧和错误帧后的重新同步。
读取超时为 4 ms；超时或 UART 错误会放弃未完成帧。
完整帧携带 `frame_lost` 或 `failsafe` 时立即提交零控制量和离线标志；
连续 100 ms 无正常帧也提交离线命令，期限由 `offline_timeout_ms` 配置。
期限检查发生在接收线程中，存在读取超时和任务调度带来的延迟。

The byte-stream parser handles fragmented and consecutive frames and searches for a new
header after malformed frames. A 4 ms read timeout or UART error discards an incomplete
frame. Receiver fault flags immediately submit zero offline CMD. Absence of healthy
frames also causes offline status after the configured timeout, subject to read timeout
and scheduling latency.

首帧及离线恢复后的首帧只建立拨杆基准；之后每次档位变化触发一次事件。
离线期间保留最近完整帧以便诊断；控制决策应使用 `GetSnapshot().online`，不能仅检查保留的通道值。
`GetSnapshot()` 与 `GetSBUSData()` 使用互斥锁复制数据，只能在线程上下文调用。
事件回调同步运行在接收线程中，可以读取快照，但应及时返回。
实例、UART 和 CMD 必须具有系统生命周期，UART 接收端由本模块独占。

The first healthy frame after startup or loss establishes the switch baseline. Later
changes emit events once per transition. Raw data is retained offline for diagnostics;
use snapshot `online` before acting on it. Getters lock and copy a coherent snapshot
from thread context. Event callbacks run synchronously in the receive thread. The
Module, UART and CMD require system lifetime; FSRemote exclusively owns UART reception.

## 2. 协议与通道 / Protocol and Channels

帧为 25 字节：`0x0F` + 22 字节通道数据 + 1 字节标志 + `0x00`。
通道按低位优先打包，每通道 11 位；本模块读取 CH1～CH10。
标志字节 bit 2 为丢帧，bit 3 为 failsafe，bit 4～7 必须为零；bit 0/1 不使用。
帧格式正确时 `DecodeFrame()` 返回 `OK`，包含故障标志的帧也会成功解码；
`ToCMD()` 再依据故障标志决定返回在线或离线命令。
长度、帧头尾或保留位错误返回 `CHECK_ERR`，空指针返回 `PTR_NULL`，错误时输出保持原值。
SBUS 没有 CRC，格式检查不能检测所有线路数据损坏。

Frames contain 25 bytes with header `0x0F`, 22 packed channel bytes, flags and footer
`0x00`. DecodeFrame validates framing and reserved bits, extracts ten 11-bit channels
and preserves fault flags. A well-formed fault frame still decodes as `OK`; ToCMD turns
it into offline commands. SBUS has no CRC, so framing cannot detect every corrupt frame.

默认映射来自随附的《FS-i6X遥控器通道映射》，使用实测端点 **240 / 1024 / 1807**。
手册 PDF 第 9 页的出厂辅助通道表不同，遥控器需按下表重新分配辅助通道。

The mapping follows the supplied measured channel map, with endpoints 240/1024/1807.
It differs from the auxiliary channel example on manual page 9; configure the transmitter
to match the table below.

| 通道 | 枚举 | 功能 / Function | 方向 / Direction |
| --- | --- | --- | --- |
| CH1 / ch[0] | RIGHT_X | 右杆左右 / Right horizontal | 左高右低 / Left high |
| CH2 / ch[1] | RIGHT_Y | 右杆上下 / Right vertical | 上高下低 / Up high |
| CH3 / ch[2] | LEFT_Y | 左杆上下 / Left vertical | 上高下低 / Up high |
| CH4 / ch[3] | LEFT_X | 左杆左右 / Left horizontal | 右高左低 / Right high |
| CH5 / ch[4] | SWC | 三档拨杆 / Three-position switch | 上低、中中、下高 / Up low |
| CH6 / ch[5] | SWA | 两档拨杆 / Two-position switch | 上高下低 / Up high |
| CH7 / ch[6] | SWB | 两档拨杆 / Two-position switch | 上高下低 / Up high |
| CH8 / ch[7] | SWD | 两档拨杆 / Two-position switch | 上高下低 / Up high |
| CH9 / ch[8] | VRA | 旋钮 / Knob | 顺时针增加 / Clockwise increases |
| CH10 / ch[9] | VRB | 旋钮 / Knob | 顺时针增加 / Clockwise increases |

`sbus.sw[0..3]` 依次为 SWA、SWB、SWC、SWD，表示原始低/中/高 `-1/0/+1`，
中位判定范围为 824～1224（含边界）。SWC 物理上位对应 `sw[C] == -1`，下位对应 `+1`。
事件采用物理位置，所以 SWC 的原始低位触发 `SwitchPos::SWC_UP`。

`sbus.sw` stores A/B/C/D raw low/mid/high levels, with the middle band 824..1224 inclusive.
SWC raw low means physical up. Event names always follow physical position.

| 拨杆 / Switch | 上 / Up | 下 / Down | 中 / Middle |
| --- | --- | --- | --- |
| SWA | SWA_UP = 0 | SWA_DOWN = 1 | SWA_MID = 2 |
| SWC | SWC_UP = 3 | SWC_DOWN = 4 | SWC_MID = 5 |
| SWB | SWB_UP = 6 | SWB_DOWN = 7 | SWB_MID = 8 |
| SWD | SWD_UP = 9 | SWD_DOWN = 10 | SWD_MID = 11 |

两档拨杆稳定状态只有上下两位；三个 MID 事件保留编号，正常两档状态不产生它们。
事件 ID 0～5 对应原 DR16 左拨杆=SWA、右拨杆=SWC；新增 SWB/SWD 事件不代表键鼠事件。

Two-position switches normally produce only up/down events. IDs 0..5 retain the DR16
switch layout; SWB/SWD IDs are additional switch events, not keyboard or mouse events.

### CMD 映射 / CMD Mapping

| 输出 / Output | 通道 / Channel | 正方向 / Positive direction |
| --- | --- | --- |
| chassis.x | CH4 | 右 / Right |
| chassis.y | CH3 | 上 / Up |
| chassis.z | CH1 | 左转 / Left turn |
| gimbal.yaw | CH1 | 左转 / Left turn |
| gimbal.pit | CH2 | 上抬 / Up |

控制量在中位两侧分别缩放并限幅为 `[-1, 1]`。SWB/SWD 和旋钮通过快照供应用层使用，
`self_define` 为 `CMD::ChasStat::NONE`，`launcher.isfire` 为 `false`；
模块不自动赋予拨杆开火或加速含义。FS-i6X 链路不传输鼠标和键盘。

Commands scale each side of center separately and clamp to [-1, 1]. Auxiliary switches
and knobs remain accessible through snapshots. Custom chassis mode remains NONE and
firing remains false. FS-i6X carries no keyboard or mouse data.

## 3. 构造接口 / Constructor

```cpp
FSRemote(LibXR::UART& uart, CMD& cmd,
         const Param& param = {.task_stack_depth_uart = 2048,
                               .thread_priority_uart = LibXR::Thread::Priority::HIGH,
                               .offline_timeout_ms = 100});
```

依赖先于参数：`uart` 为已注册的接收 UART，`cmd` 为先构造的 CMD 实例。
栈深单位为字节；离线期限必须大于零；UART 配置失败进入 LibXR `REQUIRE` 错误处理。

Dependencies precede configuration: `uart` is the registered receiver UART; `cmd` is an
earlier CMD instance. Stack depth is in bytes; timeout must be positive. UART setup
failure enters the LibXR REQUIRE failure handler.

```cpp
const auto state = fsremote.GetSnapshot();
if (state.online)
{
  const bool swb_up = state.sbus.sw[FSRemote::B] > 0;
  const float vra = FSRemote::NormalizeChannel(state.sbus.ch[FSRemote::VRA]);
  // Pass swb_up and vra to application logic.
}
```

## 4. Topic

无独立 Topic；通过 `cmd.FeedRC(data)` 提交 `CMD::Data`，由 CMD 发布其配置的控制 Topic。
该单参数接口使用 CMD 的默认遥控输入槽，不与该槽的其他接收模块同时运行。
默认声明公共索引中的 `QDU-Robomaster/CMD@same-or-dev`，无需引用专属输入源枚举。
本机 `starpang888/CMD` 也提供相同的单参数接口；使用该 fork 时需在发布索引中登记并调整 manifest。

No independent Topic. FSRemote calls the single-argument CMD FeedRC API, which selects
the default RC input slot. CMD publishes its configured control Topics. Reserve that
input slot for this receiver. The manifest selects publicly indexed QDU-Robomaster/CMD;
the local starpang888 fork offers the same API. A fork requires its own index entry and
manifest identity. No fork-specific input-source enumerator is referenced.

## 5. 配置示例 / Configuration Example

下面是实例片段；`cmd` 应已在前面构造，`usart3` 必须与 BSP 中实际的 `XR_REGISTER` 名称一致。
模块尚未发布到索引，因此示例使用占位标识 `your-owner/FSRemote`，发布时替换为实际仓库 ID。

This is an instance excerpt. Construct `cmd` earlier and use the UART's actual registered
name. Replace placeholder `your-owner/FSRemote` with the repository ID when publishing.

```yaml
modules:
  - module: your-owner/FSRemote
    id: fsremote
    args:
      - uart: usart3
      - cmd: cmd
      - param:
          task_stack_depth_uart: 2048
          thread_priority_uart: LibXR::Thread::Priority::HIGH
          offline_timeout_ms: 100
```

事件绑定源对象为 `fsremote.GetEvent()`，例如源 ID 使用
`FSRemote::SwitchPos::SWC_UP`；EventBinder 可绑定四个拨杆的事件。

Use `fsremote.GetEvent()` as the event source and event IDs such as
`FSRemote::SwitchPos::SWC_UP` with EventBinder.

## 6. 依赖与硬件 / Dependencies and Hardware

- LibXR（对照本地 2.0.1 系列接口）、C++20、支持线程和互斥锁的系统后端。
- `QDU-Robomaster/CMD@same-or-dev`，构造 FSRemote 前完成 CMD 构造。BSP 的
  `Modules/sources.yaml` 需包含 `https://qdu-robomaster.github.io/qdu-future-modules/index.yaml`；
  模块 CI 已声明此来源。
- FS-i6X + 支持 SBUS 输出的 iA6B；在接收机设置中选择串行总线 `s.bus`，启用 10 通道，
  按第 2 节配置辅助通道并长按 CANCEL 保存。
- iA6B 使用 SBUS 输出口的 GND、5V、SBUS 三根线，接收机和主控共地。
  C 板 DBUS 接口连接 USART3，并使用板上反相电路。普通 UART RX 直连接收机的反相 SBUS
  信号需要额外反相电路或 MCU 支持的 RX 反相功能。
- UART 接收队列建议至少 128 字节；模块运行前由 BSP 配好 RX DMA、引脚、NVIC 及队列。
  SBUS 使用 8 个有效数据位加偶校验；STM32F4 HAL 通常以 9 位字长包含校验位。
- 接收机失控保护需输出 SBUS 的 failsafe 标志或停止输出。若接收机失联后持续发送正常标志的
  保持值，软件无法仅凭通道值判断射频失联，应在实机检查遥控器关机后的输出。

Requires LibXR threading/mutex support, C++20, CMD and an SBUS-capable iA6B configured for
ten channels. Use the board's inverted DBUS/USART3 input, or supply appropriate inversion
for a plain UART input. Configure DMA, pins, interrupts and an RX queue of at least
128 bytes in the BSP. STM32F4 HAL uses a nine-bit word length for eight payload bits plus
parity. Receiver failsafe must set SBUS fault flags or stop transmitting; healthy-looking
hold frames alone cannot prove RF loss.

实现依据：随附 `FS-i6X遥控器通道映射.md`，以及《FS-i6x搭配iA6B接收端使用文档》
第 4～7 页的 SBUS 格式、状态位与第 8～9 页的遥控器设置说明。
#   F S R e m o t e  
 