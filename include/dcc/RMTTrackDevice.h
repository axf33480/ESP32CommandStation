/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

#ifndef _RMT_TRACK_DEVICE_H_
#define _RMT_TRACK_DEVICE_H_

#include <driver/rmt.h>
#include <driver/uart.h>
#include <esp_vfs.h>

#include <dcc/Packet.hxx>
#include <dcc/PacketFlowInterface.hxx>
#include <dcc/RailCom.hxx>
#include <openlcb/EventHandlerTemplates.hxx>
#include <os/OS.hxx>
#include <utils/macros.h>
#include <utils/StringPrintf.hxx>

#include "dcc/can_ioctl.h"
#include "dcc/MonitoredHBridge.h"

class RMTTrackDevice : public dcc::PacketFlowInterface
                     , public openlcb::BitEventInterface
{
public:
  RMTTrackDevice(openlcb::SimpleCanStack *
               , dcc::RailcomHubFlow *
               , const esp32cs::TrackOutputConfig &
               , const esp32cs::TrackOutputConfig &
               , const rmt_channel_t=RMT_CHANNEL_0
               , const gpio_num_t=(gpio_num_t)OPS_SIGNAL_PIN
               , const uint8_t=OPS_PREAMBLE_BITS
               , const gpio_num_t=(gpio_num_t)OPS_ENABLE_PIN
               , const gpio_num_t=(gpio_num_t)OPS_THERMAL_PIN
               , const adc1_channel_t=OPS_CURRENT_SENSE_ADC
               , const rmt_channel_t=RMT_CHANNEL_1
               , const gpio_num_t=(gpio_num_t)PROG_SIGNAL_PIN
               , const uint8_t=PROG_PREAMBLE_BITS
               , const gpio_num_t=(gpio_num_t)PROG_ENABLE_PIN
               , const adc1_channel_t=PROG_CURRENT_SENSE_ADC
               , const gpio_num_t=(gpio_num_t)RAILCOM_BRAKE_ENABLE_PIN
               , const gpio_num_t=(gpio_num_t)RAILCOM_ENABLE_PIN
               , const gpio_num_t=(gpio_num_t)RAILCOM_SHORT_PIN
               , const uart_port_t=(uart_port_t)RAILCOM_UART
               , const gpio_num_t=(gpio_num_t)RAILCOM_UART_RX_PIN);

  // VFS interface helper
  ssize_t write(int, const void *, size_t);

  // VFS interface helper
  int open(const char *, int, int);

  // VFS interface helper
  int close(int);

  // VFS interface helper
  int ioctl(int, int, va_list);

  // RMT callback for transmit completion. This will be called via the ISR
  // context but not from an IRAM restricted context.
  void ops_rmt_transmit_complete();

  // RMT callback for transmit completion. This will be called via the ISR
  // context but not from an IRAM restricted context.
  void prog_rmt_transmit_complete();

  // needed for Turnouts (temporary)
  void send(Buffer<dcc::Packet> *, unsigned);

  // enables the transmission of packets to the OPS track.
  void enable_ops_output();

  // disables the transmission of packets to the OPS track.
  void disable_ops_output();

  // enables the transmission of packets to the PROG track.
  void enable_prog_output();

  // disables the transmission of packets to the PROG track.
  void disable_prog_output();

  // generates a json payload for the current hbridge status.
  std::string generate_status_json();

  // BitEventInterface method to return current track output status.
  openlcb::EventState get_current_state() override
  {
    return is_enabled() ? EventState::VALID : EventState::INVALID;
  }

  // BitEventInterface method to enable/disable track output.
  void set_state(bool new_value) override
  {
    if (new_value)
    {
      enable_ops_output();
    }
    else
    {
      disable_ops_output();
    }
  }

  // BitEventInterface method
  Node *node() override
  {
    return stack_->node();
  }

  // returns true if either of the track outputs are active.
  bool is_enabled()
  {
    return opsSignalActive_ || progSignalActive_;
  }

  // retrive status of the track signal and current usage.
  std::string getStateAsDCCpp();

  std::string get_info_screen_data();

private:
  // maximum number of RMT memory blocks (256 bytes each, 4 bytes per data bit)
  // this will result in a max payload of 192 bits which is larger than any
  // known DCC packet with the addition of up to 50 preamble bits.
  static constexpr uint8_t MAX_RMT_MEMORY_BLOCKS = 3;

  // maximum number of bits that can be transmitted as one packet.
  static constexpr uint8_t MAX_DCC_PACKET_BITS = (RMT_MEM_ITEM_NUM * MAX_RMT_MEMORY_BLOCKS);

  // number of microseconds to wait after the end of packet to start the RailCom
  // processing.
  static constexpr uint8_t RAILCOM_PACKET_END_DELAY_USEC = 1;

  // number of microseconds to wait after enabling the BRAKE pin on the h-bridge
  // before disabling h-bridge output.
  static constexpr uint8_t RAILCOM_BRAKE_ENABLE_DELAY_USEC = 1;

  // number of microseconds to wait after enabling the RailCom detection circuit.
  static constexpr uint8_t RAILCOM_ENABLE_TRANSIENT_DELAY_USEC = 1;

  // number of microseconds to wait for railcom data on channel 1.
  static constexpr TickType_t RAILCOM_MAX_READ_DELAY_CH_1 = 177
    - RAILCOM_PACKET_END_DELAY_USEC
    - RAILCOM_BRAKE_ENABLE_DELAY_USEC
    - RAILCOM_ENABLE_TRANSIENT_DELAY_USEC;

  // number of microseconds to wait for railcom data on channel 2.
  static constexpr TickType_t RAILCOM_MAX_READ_DELAY_CH_2 = 454 - RAILCOM_MAX_READ_DELAY_CH_1;

  // number of microseconds to wait after disabling the BRAKE pin on the h-bridge
  // before returning to normal operations. The h-bridge output will be ENABLED
  // prior to the BRAKE pin being disabled.
  static constexpr uint8_t RAILCOM_BRAKE_DISABLE_DELAY_USEC = 10;

  openlcb::SimpleCanStack *stack_{nullptr};

  const gpio_num_t opsSignalPin_;
  const rmt_channel_t opsRMTChannel_;
  const uint32_t opsPreambleBits_;
  const gpio_num_t opsOutputEnablePin_;
  DeviceBuffer<dcc::Packet> *opsPacketQueue_;
  Notifiable* opsWritableNotifiable_{nullptr};
  rmt_item32_t opsEncodedPacket_[MAX_DCC_PACKET_BITS];
  uint32_t opsEncodedLength_{0};
  int8_t opsPacketRepeatCount_{0};
  uintptr_t opsPacketFeedbackKey_{0};
  bool opsSignalActive_{false};
  std::unique_ptr<MonitoredHBridge> opsHBridge_;

  const gpio_num_t progSignalPin_;
  const rmt_channel_t progRMTChannel_;
  const uint32_t progPreambleBits_;
  const gpio_num_t progOutputEnablePin_;
  DeviceBuffer<dcc::Packet> *progPacketQueue_;
  rmt_item32_t progEncodedPacket_[MAX_DCC_PACKET_BITS];
  uint32_t progEncodedLength_{0};
  int8_t progPacketRepeatCount_{0};
  Notifiable* progWritableNotifiable_{nullptr};
  bool progSignalActive_{false};
  std::unique_ptr<MonitoredHBridge> progHBridge_;

  const gpio_num_t railComBrakeEnablePin_;
  const gpio_num_t railComEnablePin_;
  const gpio_num_t railComShortPin_;
  const uart_port_t railComUartPort_;

  RailcomHubFlow *railComHub_;
  Buffer<dcc::RailcomHubData> *railComFeedback_{nullptr};
  std::function<void(void)> railcomReader_{nullptr};
  bool railcomEnabled_{false};

  dcc::Packet idlePacket_{dcc::Packet::DCC_IDLE()};
  Atomic packetQueueLock_;
  uint32_t packetQueueOverrunCount_{0};
  bool devOpened_{false};
  bool infoDataFirst_{false};

  void initRMTDevice(const char *, rmt_channel_t, gpio_num_t, uint8_t);
  void encode_next_ops_packet();
  void encode_next_prog_packet();
  void send_last_railcom_response_buffer(bool=false);
  void get_next_railcom_response_buffer(uintptr_t);
  void read_railcom_response();
  void read_railcom_response_noop();

  DISALLOW_COPY_AND_ASSIGN(RMTTrackDevice);
};

#endif // _RMT_TRACK_DEVICE_H_
