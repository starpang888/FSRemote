#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: FS-i6X/iA6B SBUS 遥控模块，提供通道快照、拨杆事件和 CMD 控制输入 / FS-i6X/iA6B SBUS remote Module with channel snapshots, switch events and CMD input
depends:
  - id: QDU-Robomaster/CMD
    ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "CMD.hpp"
#include "event.hpp"
#include "mutex.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

/**
 * @brief FS-i6X + iA6B 的 SBUS 接收模块。实例及 UART/CMD 必须存活至系统停止。
 *        SBUS receiver for FS-i6X + iA6B. Instance, UART and CMD need system lifetime.
 * @note 独占 UART 接收端；接收、CMD 提交及事件回调均发生在接收线程。
 *       Exclusively owns UART RX; reception, CMD submission and events run in its thread.
 */
class FSRemote
{
 public:
  static constexpr std::size_t FRAME_LENGTH = 25;
  static constexpr std::size_t CHANNEL_COUNT = 10;
  static constexpr uint16_t CHANNEL_MIN = 240;
  static constexpr uint16_t CHANNEL_MID = 1024;
  static constexpr uint16_t CHANNEL_MAX = 1807;
  static constexpr uint16_t SWITCH_DEADBAND = 200;

  /// 通道下标，与实测映射一致 / Channel indices matching the measured mapping.
  enum Channel : uint8_t
  {
    RIGHT_X = 0,
    RIGHT_Y = 1,
    LEFT_Y = 2,
    LEFT_X = 3,
    SWC = 4,
    SWA = 5,
    SWB = 6,
    SWD = 7,
    VRA = 8,
    VRB = 9
  };

  /// sw[] 的顺序 / Order of entries in sw[].
  enum Switch : uint8_t
  {
    A = 0,
    B = 1,
    C = 2,
    D = 3
  };

  /**
   * @brief 按物理上下位置触发的事件；0..5 沿用 DR16 左=SWA、右=SWC 的布局。
   *        Events follow physical positions; 0..5 retain DR16 left=SWA, right=SWC layout.
   */
  enum class SwitchPos : uint8_t
  {
    SWA_UP = 0,
    SWA_DOWN = 1,
    SWA_MID = 2,
    SWC_UP = 3,
    SWC_DOWN = 4,
    SWC_MID = 5,
    SWB_UP = 6,
    SWB_DOWN = 7,
    SWB_MID = 8,
    SWD_UP = 9,
    SWD_DOWN = 10,
    SWD_MID = 11
  };

  /// 完整帧的原始通道及状态 / Raw channels and status of a complete frame.
  struct SBUSData
  {
    std::array<uint16_t, CHANNEL_COUNT> ch{};  ///< 原始 11 位值 / Raw 11-bit values.
    std::array<int8_t, 4> sw{};  ///< A/B/C/D 原始低/中/高=-1/0/+1 / Raw low/mid/high.
    bool frame_lost = false;     ///< SBUS bit 2 / SBUS bit 2.
    bool failsafe = false;       ///< SBUS bit 3 / SBUS bit 3.
  };

  /// 线程安全的单次读取结果 / Coherent snapshot for application threads.
  struct Snapshot
  {
    SBUSData sbus{};  ///< 最近完整帧，离线时保留 / Last complete frame, retained offline.
    CMD::Data
        control{};  ///< 当前命令；离线时清零 / Current command, zeroed when offline.
    bool online = false;  ///< 可用帧且未超时 / Usable frame received and not timed out.
    LibXR::MillisecondTimestamp
        last_valid_frame{};  ///< 最近正常帧时间 / Last healthy frame.
  };

  /// 接收线程配置 / Receive thread configuration.
  struct Param
  {
    uint32_t task_stack_depth_uart;  ///< 栈大小，字节 / Stack size in bytes.
    LibXR::Thread::Priority thread_priority_uart;  ///< 线程优先级 / Thread priority.
    uint32_t
        offline_timeout_ms;  ///< 无正常帧的离线期限 / Timeout without healthy frames.
  };

  /**
   * @brief 配置 100000 bit/s、8E2，提交初始离线命令并启动接收线程。
   *        Configure 100000 bit/s, 8E2, submit initial offline CMD and start reception.
   * @param uart 已由 BSP 注册的 UART，须有硬件 SBUS 反相及有效接收队列。
   *             Registered UART with hardware SBUS inversion and a working RX queue.
   * @param cmd 已构造的 CMD；FeedRC(data) 使用其默认遥控输入槽。
   *            Existing CMD; FeedRC(data) selects its default RC input slot.
   * @param param 线程和超时参数，offline_timeout_ms 必须大于 0。
   *              Thread and timeout settings; offline_timeout_ms must be positive.
   */
  FSRemote(LibXR::UART& uart, CMD& cmd,
           const Param& param = {.task_stack_depth_uart = 2048,
                                 .thread_priority_uart = LibXR::Thread::Priority::HIGH,
                                 .offline_timeout_ms = 100})
      : uart_(uart), cmd_(cmd), param_(param), sem_(0), read_op_(sem_, READ_TIMEOUT_MS)
  {
    REQUIRE(uart_.read_port_ != nullptr);
    REQUIRE(param_.offline_timeout_ms > 0);
    const auto RESULT = uart_.SetConfig({100000, LibXR::UART::Parity::EVEN, 8, 2});
    REQUIRE(RESULT == LibXR::ErrorCode::OK);
    snapshot_.control = MakeOfflineCMD();
    cmd_.FeedRC(snapshot_.control);
    thread_.Create(this, ThreadMain, "fsremote_rx", param_.task_stack_depth_uart,
                   param_.thread_priority_uart);
  }

  FSRemote(const FSRemote&) = delete;
  FSRemote& operator=(const FSRemote&) = delete;

  /**
   * @brief 在线程上下文复制一致快照；不能在 ISR 调用。
   *        Copy a coherent snapshot from thread context; do not call from an ISR.
   * @return 原始通道、命令、在线状态和最后正常帧时间 / Channels, CMD, status and
   * timestamp.
   */
  Snapshot GetSnapshot() const
  {
    LibXR::Mutex::LockGuard guard(snapshot_mutex_);
    return snapshot_;
  }

  /// 获取原始数据；应用层使用前应检查快照 online / Raw data; check snapshot online first.
  SBUSData GetSBUSData() const { return GetSnapshot().sbus; }

  /// 获取事件对象，回调不得阻塞接收线程 / Events; callbacks must not block the RX thread.
  LibXR::Event& GetEvent() { return event_; }

  /**
   * @brief 纯解码，无实例副作用；检查长度、帧头尾及保留标志位。
   *        Decode without instance side effects; validate length, framing and flag bits.
   * @param raw_data 原始缓冲 / Raw buffer.
   * @param size 缓冲长度，必须为 25 / Buffer length, exactly 25.
   * @param output 成功时写入（含丢帧/失控标志）；失败时保持原值。
   *               Written on success, including fault flags; unchanged on failure.
   * @return OK=帧格式正确（仍须检查 flags）；PTR_NULL=空指针；CHECK_ERR=格式错误。
   *         OK=well-formed frame (check flags too), PTR_NULL=null, CHECK_ERR=bad format.
   */
  static constexpr LibXR::ErrorCode DecodeFrame(const uint8_t* raw_data, std::size_t size,
                                                SBUSData& output)
  {
    if (raw_data == nullptr)
    {
      return LibXR::ErrorCode::PTR_NULL;
    }
    if (size != FRAME_LENGTH || raw_data[0] != 0x0F || raw_data[24] != 0x00 ||
        (raw_data[23] & 0xF0U) != 0)
    {
      return LibXR::ErrorCode::CHECK_ERR;
    }

    SBUSData parsed{};
    for (std::size_t channel = 0; channel < CHANNEL_COUNT; ++channel)
    {
      const std::size_t BIT = channel * 11;
      const std::size_t BYTE = 1 + BIT / 8;
      const uint32_t PACKED = static_cast<uint32_t>(raw_data[BYTE]) |
                              (static_cast<uint32_t>(raw_data[BYTE + 1]) << 8) |
                              (static_cast<uint32_t>(raw_data[BYTE + 2]) << 16);
      parsed.ch[channel] = static_cast<uint16_t>((PACKED >> (BIT % 8)) & 0x07FFU);
    }
    constexpr std::array<Channel, 4> SWITCH_CHANNELS = {SWA, SWB, SWC, SWD};
    for (std::size_t i = 0; i < parsed.sw.size(); ++i)
    {
      const auto VALUE = parsed.ch[SWITCH_CHANNELS[i]];
      parsed.sw[i] = VALUE > CHANNEL_MID + SWITCH_DEADBAND   ? 1
                     : VALUE < CHANNEL_MID - SWITCH_DEADBAND ? -1
                                                             : 0;
    }
    parsed.frame_lost = (raw_data[23] & 0x04U) != 0;
    parsed.failsafe = (raw_data[23] & 0x08U) != 0;
    output = parsed;
    return LibXR::ErrorCode::OK;
  }

  /**
   * @brief 按 240/1024/1807 分段归一化并限幅至 [-1, 1]，可用于旋钮。
   *        Normalize around 240/1024/1807 and clamp to [-1, 1], including knobs.
   * @param value 原始通道值 / Raw channel value.
   * @return 中位为 0 的控制量 / Control value centered at zero.
   */
  static constexpr float NormalizeChannel(uint16_t value)
  {
    const int32_t OFFSET = static_cast<int32_t>(value) - CHANNEL_MID;
    const float SPAN = static_cast<float>(OFFSET >= 0 ? CHANNEL_MAX - CHANNEL_MID
                                                      : CHANNEL_MID - CHANNEL_MIN);
    return std::clamp(static_cast<float>(OFFSET) / SPAN, -1.0f, 1.0f);
  }

  /**
   * @brief 转换摇杆至 CMD；丢帧/失控时返回全零离线命令。SWB/SWD/旋钮留给应用层。
   *        Convert sticks to CMD; fault flags yield zero offline CMD. Aux inputs stay
   * raw.
   * @param data DecodeFrame 的结果 / Result from DecodeFrame.
   * @return x=左杆左右，y=左杆上下，z/yaw=右杆左右，pit=右杆上下；发射为 false。
   *         x=left horizontal, y=left vertical, z/yaw=right horizontal, pit=right
   * vertical.
   */
  static constexpr CMD::Data ToCMD(const SBUSData& data)
  {
    CMD::Data output = MakeOfflineCMD();
    if (data.frame_lost || data.failsafe)
    {
      return output;
    }
    output.chassis.x = NormalizeChannel(data.ch[LEFT_X]);
    output.chassis.y = NormalizeChannel(data.ch[LEFT_Y]);
    output.chassis.z = NormalizeChannel(data.ch[RIGHT_X]);
    output.gimbal.yaw = NormalizeChannel(data.ch[RIGHT_X]);
    output.gimbal.pit = NormalizeChannel(data.ch[RIGHT_Y]);
    output.chassis_online = true;
    output.gimbal_online = true;
    return output;
  }

 private:
  static constexpr uint32_t READ_TIMEOUT_MS = 4;

  static constexpr CMD::Data MakeOfflineCMD()
  {
    CMD::Data output{};
    output.ctrl_source = CMD::ControlSource::CTRL_SOURCE_RC;
    return output;
  }

  static void ThreadMain(FSRemote* remote)
  {
    while (true)
    {
      uint8_t byte = 0;
      const auto RESULT = remote->uart_.Read({&byte, 1}, remote->read_op_);
      // Check elapsed time before a queued frame can restore the previous state.
      remote->CheckOffline();
      if (RESULT == LibXR::ErrorCode::OK)
      {
        remote->PushByte(byte);
      }
      else
      {
        remote->rx_used_ = 0;
        if (RESULT != LibXR::ErrorCode::TIMEOUT)
        {
          LibXR::Thread::Sleep(1);
        }
      }
    }
  }

  void PushByte(uint8_t byte)
  {
    if (rx_used_ == 0 && byte != 0x0F)
    {
      return;
    }
    rx_frame_[rx_used_++] = byte;
    if (rx_used_ != FRAME_LENGTH)
    {
      return;
    }

    SBUSData parsed{};
    if (DecodeFrame(rx_frame_.data(), rx_frame_.size(), parsed) == LibXR::ErrorCode::OK)
    {
      rx_used_ = 0;
      AcceptFrame(parsed);
      return;
    }

    // A header can occur inside packed channel data; keep the next candidate suffix.
    std::size_t next = 1;
    while (next < FRAME_LENGTH && rx_frame_[next] != 0x0F)
    {
      ++next;
    }
    rx_used_ = FRAME_LENGTH - next;
    for (std::size_t i = 0; i < rx_used_; ++i)
    {
      rx_frame_[i] = rx_frame_[next + i];
    }
  }

  void AcceptFrame(const SBUSData& parsed)
  {
    const auto CONTROL = ToCMD(parsed);
    const bool ONLINE = !parsed.frame_lost && !parsed.failsafe;
    std::array<int8_t, 4> previous_switches{};
    bool emit_events = false;
    {
      LibXR::Mutex::LockGuard guard(snapshot_mutex_);
      previous_switches = snapshot_.sbus.sw;
      emit_events = snapshot_.online && ONLINE;
      snapshot_.sbus = parsed;
      snapshot_.control = CONTROL;
      snapshot_.online = ONLINE;
      if (ONLINE)
      {
        snapshot_.last_valid_frame = LibXR::Timebase::GetMilliseconds();
      }
    }

    // Release the snapshot lock before callbacks; callbacks may read GetSnapshot().
    if (emit_events)
    {
      constexpr std::array<SwitchPos, 4> BASE = {SwitchPos::SWA_UP, SwitchPos::SWB_UP,
                                                 SwitchPos::SWC_UP, SwitchPos::SWD_UP};
      for (std::size_t i = 0; i < parsed.sw.size(); ++i)
      {
        if (parsed.sw[i] != previous_switches[i])
        {
          const int8_t POSITION =
              i == C ? static_cast<int8_t>(-parsed.sw[i]) : parsed.sw[i];
          const uint32_t OFFSET = POSITION > 0 ? 0 : (POSITION < 0 ? 1 : 2);
          event_.Active(static_cast<uint32_t>(BASE[i]) + OFFSET);
        }
      }
    }
    cmd_.FeedRC(CONTROL);
  }

  void CheckOffline()
  {
    bool became_offline = false;
    {
      LibXR::Mutex::LockGuard guard(snapshot_mutex_);
      if (snapshot_.online &&
          (LibXR::Timebase::GetMilliseconds() - snapshot_.last_valid_frame)
                  .ToMillisecond() >= param_.offline_timeout_ms)
      {
        snapshot_.online = false;
        snapshot_.control = MakeOfflineCMD();
        became_offline = true;
      }
    }
    if (became_offline)
    {
      cmd_.FeedRC(MakeOfflineCMD());
    }
  }

  LibXR::UART& uart_;
  CMD& cmd_;
  const Param param_;
  LibXR::Semaphore sem_;
  LibXR::ReadOperation read_op_;
  LibXR::Thread thread_;
  LibXR::Event event_;
  mutable LibXR::Mutex snapshot_mutex_;
  Snapshot snapshot_{};
  std::array<uint8_t, FRAME_LENGTH> rx_frame_{};
  std::size_t rx_used_ = 0;
};
