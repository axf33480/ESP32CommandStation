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

#include "RMTTrackDevice.h"

#include <dcc/DccDebug.hxx>
#include <soc/gpio_struct.h>

#include <StatusDisplay.h>

///////////////////////////////////////////////////////////////////////////////
// RMT clock tick divider value. With the APB frequency of 80Mhz we can use a
// divider of 80 to have an approximate 1usec tick frequency which simplifies
// and improves accuracy on the DCC packet timing.
///////////////////////////////////////////////////////////////////////////////
static constexpr uint8_t RMT_CLOCK_DIVIDER = 80;

///////////////////////////////////////////////////////////////////////////////
// RMT direct register access helper macros. These are used in the tx complete
// ISR handler(s) rather than IDF RMT APIs which can crash from inside the ISR
// callback due to FreeRTOS function usage.
///////////////////////////////////////////////////////////////////////////////
#define RMT_SET_FIFO(mode) RMT.apb_conf.fifo_mask = mode;
#define RMT_SET_DATA(channel, index) RMTMEM.chan[channel].data32[index].val
#define RMT_CONFIG(channel) RMT.conf_ch[channel].conf1

///////////////////////////////////////////////////////////////////////////////
// The NMRA DCC Signal is sent as a square wave with each half having
// identical timing (or nearly identical). Packet Bytes have a minimum of 11
// preamble ONE bits in order to be considered valid by the decoder. For
// RailCom support it is recommended to have at least 16 preamble bits. For the
// Programming Track it is required to have a longer preamble of at least 22
// bits. Packet data follows immediately after the preamble bits, between the
// packet bytes a DCC ZERO is sent. After the last byte of packet data a DCC
// ONE is sent.
//
// DCC ZERO:
//    ----------------
//    |      96      |
// ---|     usec     |      96      ---
//                   |     usec     |
//                   ----------------
// DCC ONE:
//    --------
//    |  58  |      
// ---| usec |  58  ---
//           | usec |
//           --------
//
// Waveforms above are not exact to scale.
//
///////////////////////////////////////////////////////////////////////////////
static constexpr uint32_t DCC_ZERO_BIT_PULSE_USEC = 96;
static constexpr uint32_t DCC_ONE_BIT_PULSE_USEC = 58;

///////////////////////////////////////////////////////////////////////////////
// DCC ZERO bit pre-encoded in RMT format, sent as HIGH then LOW.
///////////////////////////////////////////////////////////////////////////////
static constexpr rmt_item32_t DCC_RMT_ZERO_BIT =
{{{
    DCC_ZERO_BIT_PULSE_USEC       // number of microseconds for TOP half
  , 1                             // of the square wave.
  , DCC_ZERO_BIT_PULSE_USEC       // number of microseconds for BOTTOM half
  , 0                             // of the square wave.
}}};

///////////////////////////////////////////////////////////////////////////////
// DCC ONE bit pre-encoded in RMT format, sent as HIGH then LOW.
///////////////////////////////////////////////////////////////////////////////
static constexpr rmt_item32_t DCC_RMT_ONE_BIT =
{{{
    DCC_ONE_BIT_PULSE_USEC        // number of microseconds for TOP half
  , 1                             // of the square wave.
  , DCC_ONE_BIT_PULSE_USEC        // number of microseconds for BOTTOM half
  , 0                             // of the square wave.
}}};

///////////////////////////////////////////////////////////////////////////////
// Marklin Motorola bit timing (WIP)
// https://people.zeelandnet.nl/zondervan/digispan.html
// http://www.drkoenig.de/digital/motorola.htm
///////////////////////////////////////////////////////////////////////////////
static constexpr uint32_t MARKLIN_ZERO_BIT_PULSE_HIGH_USEC = 182;
static constexpr uint32_t MARKLIN_ZERO_BIT_PULSE_LOW_USEC = 26;
static constexpr uint32_t MARKLIN_ONE_BIT_PULSE_HIGH_USEC = 26;
static constexpr uint32_t MARKLIN_ONE_BIT_PULSE_LOW_USEC = 182;
static constexpr uint32_t MARKLIN_PREAMBLE_BIT_PULSE_HIGH_USEC = 104;
static constexpr uint32_t MARKLIN_PREAMBLE_BIT_PULSE_LOW_USEC = 104;

///////////////////////////////////////////////////////////////////////////////
// Marklin Motorola ZERO bit pre-encoded in RMT format, sent as HIGH then LOW.
///////////////////////////////////////////////////////////////////////////////
static constexpr rmt_item32_t MARKLIN_RMT_ZERO_BIT =
{{{
    MARKLIN_ZERO_BIT_PULSE_HIGH_USEC  // number of microseconds for TOP half
  , 1                                 // of the square wave.
  , MARKLIN_ZERO_BIT_PULSE_LOW_USEC   // number of microseconds for BOTTOM half
  , 0                                 // of the square wave.
}}};

///////////////////////////////////////////////////////////////////////////////
// Marklin Motorola ONE bit pre-encoded in RMT format, sent as HIGH then LOW.
///////////////////////////////////////////////////////////////////////////////
static constexpr rmt_item32_t MARKLIN_RMT_ONE_BIT =
{{{
    MARKLIN_ZERO_BIT_PULSE_HIGH_USEC  // number of microseconds for TOP half
  , 1                                 // of the square wave.
  , MARKLIN_ZERO_BIT_PULSE_LOW_USEC   // number of microseconds for BOTTOM half
  , 0                                 // of the square wave.
}}};

///////////////////////////////////////////////////////////////////////////////
// Marklin Motorola preamble bit pre-encoded in RMT format, both top and bottom
// half of the wave are LOW.
///////////////////////////////////////////////////////////////////////////////
static constexpr rmt_item32_t MARKLIN_RMT_PREAMBLE_BIT =
{{{
    MARKLIN_PREAMBLE_BIT_PULSE_HIGH_USEC // number of microseconds for TOP half
  , 0                                    // of the square wave.
  , MARKLIN_PREAMBLE_BIT_PULSE_LOW_USEC  // number of microseconds for BOTTOM
  , 0                                    // half of the square wave.
}}};

///////////////////////////////////////////////////////////////////////////////
// Declare ISR flags for the RMT driver ISR.
//
// NOTE: ESP_INTR_FLAG_IRAM is *NOT* included in this bitmask so that we do not
// have a dependency on execution from IRAM and the related software
// limitations of execution from there.
///////////////////////////////////////////////////////////////////////////////
static constexpr uint32_t RMT_ISR_FLAGS =
(
    ESP_INTR_FLAG_LOWMED              // ISR is implemented in C code
  | ESP_INTR_FLAG_SHARED              // ISR is shared across multiple handlers
);

///////////////////////////////////////////////////////////////////////////////
// Bit mask constants used as part of the packet translation layer.
///////////////////////////////////////////////////////////////////////////////
static constexpr DRAM_ATTR uint8_t PACKET_BIT_MASK[] =
{
  0x80, 0x40, 0x20, 0x10, //
  0x08, 0x04, 0x02, 0x01  //
};

#if CONFIG_OPS_RAILCOM
///////////////////////////////////////////////////////////////////////////////
// RailCom UART threshold defaults (copied from esp-idf uart.c)
///////////////////////////////////////////////////////////////////////////////
static constexpr uint8_t UART_FULL_THRESH_DEFAULT = 120; // 120 bytes
static constexpr uint8_t UART_TOUT_THRESH_DEFAULT = 10;  // 10 bit times

///////////////////////////////////////////////////////////////////////////////
// RailCom UART ISR bitmask to enable/disable RX.
///////////////////////////////////////////////////////////////////////////////
static constexpr uint32_t UART_RX_INT_EN_MASK = UART_RXFIFO_FULL_INT_ENA
                                              | UART_RXFIFO_TOUT_INT_ENA;

///////////////////////////////////////////////////////////////////////////////
// RailCom UART ISR bitmask to clear the RX of data.
///////////////////////////////////////////////////////////////////////////////
static constexpr uint32_t UART_RX_INT_CLR_MASK = UART_RXFIFO_TOUT_INT_CLR_M
                                               | UART_RXFIFO_FULL_INT_CLR_M;

///////////////////////////////////////////////////////////////////////////////
// RailCom UART ISR flags.
//
// For the RailCom UART the CS does not use the ESP-IDF default UART ISR
// handler since it is not possible to call uart_read_bytes() from inside the
// ISR callback for RMT TX complete (which is when the RC code is active).
// Instead we register our own ISR to capture the data and push it into the
// waiting dcc::Feedback packet.
///////////////////////////////////////////////////////////////////////////////
static constexpr uint32_t RAILCOM_ISR_FLAGS = ESP_INTR_FLAG_LOWMED;

///////////////////////////////////////////////////////////////////////////////
// UART device handles for direct hardware access.
//
// These should only be accessed from inside the ISR context for RailCom.
///////////////////////////////////////////////////////////////////////////////
static DRAM_ATTR uart_dev_t* const UART[UART_NUM_MAX] =
{
  &UART0, &UART1, &UART2
};

///////////////////////////////////////////////////////////////////////////////
// Inline method to retrieve the current byte from the hardware uart fifo
// register.
//
// Note: This has *NO* error checking and should only be used with previously
// validated parameters when execution speed is critical.
///////////////////////////////////////////////////////////////////////////////
static inline uint8_t _uart_read(uart_port_t uart_num)
{
  return UART[uart_num]->fifo.rw_byte;
}

///////////////////////////////////////////////////////////////////////////////
// Inline method to check the hardware UART RX FIFO for data.
//
// Note: This has *NO* error checking and should only be used with previously
// validated parameters when execution speed is critical.
///////////////////////////////////////////////////////////////////////////////
static inline bool _uart_available(uart_port_t uart_num)
{
  return UART[uart_num]->status.rxfifo_cnt != 0;
}

///////////////////////////////////////////////////////////////////////////////
// Inline method to clear the UART RX buffer.
//
// Note: This has *NO* error checking and should only be used with previously
// validated parameters when execution speed is critical.
//
// Note: Due to a hardware issue we can not reset the fifo directly but instead
// need to read data from it until the fifo is empty. This is a known rom bug.
///////////////////////////////////////////////////////////////////////////////
static inline void _uart_flush(uart_port_t uart_num)
{
  while(_uart_available(uart_num))
  {
    _uart_read(uart_num);
  }
}
#endif // CONFIG_OPS_RAILCOM


///////////////////////////////////////////////////////////////////////////////
// Inline method for setting the output state of a gpio pin via it's hardware
// register. gpio_set_level should normally be used instead.
//
// Note: This method has minimal error checking and should only be used with
// previously validated parameters when execution speed is critical.
///////////////////////////////////////////////////////////////////////////////
static inline void _set_gpio_state(gpio_num_t pin, bool state)
{
  if (state)
  {
    if (pin < 32)
    {
      GPIO.out_w1ts = (1 << pin);
    }
    else if (pin < 40)
    {
      GPIO.out1_w1ts.data = (1 << (pin & 31));
    }
  }
  else
  {
    if (pin < 32)
    {
      GPIO.out_w1tc = (1 << pin);
    }
    else if (pin < 40)
    {
      GPIO.out1_w1tc.data = (1 << (pin & 31));
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Inline method to retrieve the current state of a gpio pin from the hardware
// register. gpio_get_level should normally be used instead.
//
// Note: This method has minimal error checking and should only be used with
// previously validated parameters when execution speed is critical.
///////////////////////////////////////////////////////////////////////////////
static inline bool _get_gpio_state(gpio_num_t pin)
{
  if (pin < 32)
  {
    return (GPIO.in >> pin) & 0x1;
  }
  else if (pin < 40)
  {
    return (GPIO.in1.data >> (pin & 31)) & 0x1;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////
// ESP32 VFS ::write() impl for the RMTTrackDevice
///////////////////////////////////////////////////////////////////////////////
static ssize_t rmt_track_write(void* ctx, int fd, const void * data, size_t size)
{
  return reinterpret_cast<RMTTrackDevice *>(ctx)->write(fd, data, size);
}

///////////////////////////////////////////////////////////////////////////////
// ESP32 VFS ::open() impl for the RMTTrackDevice
///////////////////////////////////////////////////////////////////////////////
static int rmt_track_open(void* ctx, const char * path, int flags, int mode)
{
  return reinterpret_cast<RMTTrackDevice *>(ctx)->open(path, flags, mode);
}

///////////////////////////////////////////////////////////////////////////////
// ESP32 VFS ::close() impl for the RMTTrackDevice
///////////////////////////////////////////////////////////////////////////////
static int rmt_track_close(void* ctx, int fd)
{
  return reinterpret_cast<RMTTrackDevice *>(ctx)->close(fd);
}

///////////////////////////////////////////////////////////////////////////////
// ESP32 VFS ::ioctl() impl for the RMTTrackDevice
///////////////////////////////////////////////////////////////////////////////
static int rmt_track_ioctl(void* ctx, int fd, int cmd, va_list args)
{
  return reinterpret_cast<RMTTrackDevice *>(ctx)->ioctl(fd, cmd, args);
}

///////////////////////////////////////////////////////////////////////////////
// RMT TX complete ISR callback.
//
// This is called automatically by the RMT peripheral when it reaches the end
// of TX data.
///////////////////////////////////////////////////////////////////////////////
static void rmt_tx_complete_isr_callback(rmt_channel_t channel, void *ctx)
{
  RMTTrackDevice *track = reinterpret_cast<RMTTrackDevice *>(ctx);
  // Check if the channel 0 TX END has been triggered
  // if so the OPS TX has completed and we need to clear the event
  if (channel == RMT_CHANNEL_0)
  {
    track->ops_rmt_transmit_complete();
  }

  // Check if the channel 1 TX END has been triggered
  // if so the PROG TX has completed and we need to clear the event
  if (channel == RMT_CHANNEL_3)
  {
    track->prog_rmt_transmit_complete();
  }
}

#if CONFIG_OPS_RAILCOM
static void railcom_uart_isr(void *ctx)
{
  RMTTrackDevice *track = reinterpret_cast<RMTTrackDevice *>(ctx);
  track->railcom_data_received();
}
#endif // CONFIG_OPS_RAILCOM

///////////////////////////////////////////////////////////////////////////////
// Helper macro for encoding a dcc::Packet into an RMT payload
//
// When a packet has remaining repeats the packet data is not re-encoded.
// This will allocate a new RailCom feedback when a new packet is encoded based
// on the alloc_railcom function passed in.
///////////////////////////////////////////////////////////////////////////////
#define ENCODE_PACKET(queue, lock, preambleCount, encodedPacket             \
                    , encodedLength, repeatCount, notifiable                \
                    , alloc_railcom)                                        \
  /* decrement the repeat count and if we still have at least one repeat
     remaining skip reencoding the packet */                                \
  if (--repeatCount >= 0)                                                   \
  {                                                                         \
    return;                                                                 \
  }                                                                         \
  /* attempt to fetch a packet from the queue or use an idle packet */      \
  dcc::Packet packet{dcc::Packet::DCC_IDLE()};                              \
  {                                                                         \
    AtomicHolder l(&lock);                                                  \
    queue->get(&packet, 1);                                                 \
  }                                                                         \
  /* Check if we have a pending writer and notify it if needed. */          \
  Notifiable* n = nullptr;                                                  \
  std::swap(n, notifiable);                                                 \
  if (n)                                                                    \
  {                                                                         \
    n->notify_from_isr();                                                   \
  }                                                                         \
  /* encode the preamble bits */                                            \
  for (encodedLength = 0; encodedLength < preambleCount; encodedLength++)   \
  {                                                                         \
    encodedPacket[encodedLength].val = DCC_RMT_ONE_BIT.val;                 \
  }                                                                         \
  /* start of payload marker */                                             \
  encodedPacket[encodedLength++].val = DCC_RMT_ZERO_BIT.val;                \
  /* encode the packet bits */                                              \
  for (uint8_t dlc = 0; dlc < packet.dlc; dlc++)                            \
  {                                                                         \
    for(uint8_t bit = 0; bit < 8; bit++)                                    \
    {                                                                       \
      encodedPacket[encodedLength++].val =                                  \
        packet.payload[dlc] & PACKET_BIT_MASK[bit] ?                        \
          DCC_RMT_ONE_BIT.val : DCC_RMT_ZERO_BIT.val;                       \
    }                                                                       \
    /* end of byte marker */                                                \
    encodedPacket[encodedLength++].val = DCC_RMT_ZERO_BIT.val;              \
  }                                                                         \
  /* set the last bit of the encoded payload to be an end of packet 
      marker */                                                             \
  encodedPacket[encodedLength - 1].val = DCC_RMT_ONE_BIT.val;               \
  /* add an extra ONE bit to the end to prevent mangling of the last bit
      by the RMT */                                                         \
  encodedPacket[encodedLength++].val = DCC_RMT_ONE_BIT.val;                 \
  /* record the repeat count */                                             \
  repeatCount = packet.packet_header.rept_count + 1;                        \
  alloc_railcom(packet.feedback_key);

///////////////////////////////////////////////////////////////////////////////
// RMTTrackDevice constructor.
//
// This creates a VFS interface for the packet queue which can be used by
// the LocalTrackIf implementation.
//
// The VFS mount point is /dev/track. This must be opened by the caller before
// the LocalTrackIf is able to route dcc::Packets to the track.
//
// This also allocates two h-bridge monitoring state flows, these will check
// for short circuits and disable the track output from the h-bridge
// independently from the RMT signal being generated.
///////////////////////////////////////////////////////////////////////////////
RMTTrackDevice::RMTTrackDevice(openlcb::Node *node
                             , Service *service
                             , const esp32cs::TrackOutputConfig &opsCfg
                             , const esp32cs::TrackOutputConfig &progCfg
#if CONFIG_OPS_RAILCOM
                             , dcc::RailcomHubFlow *railComHub
#endif // CONFIG_OPS_RAILCOM
                             )
                             : BitEventInterface(openlcb::Defs::CLEAR_EMERGENCY_OFF_EVENT
                                               , openlcb::Defs::EMERGENCY_OFF_EVENT)
                             , node_(node)
                             , opsPacketQueue_(DeviceBuffer<dcc::Packet>::create(CONFIG_OPS_PACKET_QUEUE_SIZE))
                             , opsHBridge_(node
                                         , service
                                         , (adc1_channel_t)CONFIG_OPS_ADC
                                         , opsOutputEnablePin_
#if defined(CONFIG_OPS_THERMAL_PIN)
                                         , (gpio_num_t)CONFIG_OPS_THERMAL_PIN
#endif // CONFIG_OPS_THERMAL_PIN
                                         , CONFIG_OPS_HBRIDGE_LIMIT_MILLIAMPS
                                         , CONFIG_OPS_HBRIDGE_MAX_MILLIAMPS
                                         , CONFIG_OPS_TRACK_NAME, CONFIG_OPS_HBRIDGE_TYPE_NAME
                                         , opsCfg)
                             , progPacketQueue_(DeviceBuffer<dcc::Packet>::create(CONFIG_PROG_PACKET_QUEUE_SIZE))
                             , progHBridge_(node
                                          , service
                                          , (adc1_channel_t)CONFIG_PROG_ADC
                                          , progOutputEnablePin_
                                          , CONFIG_PROG_HBRIDGE_MAX_MILLIAMPS
                                          , CONFIG_PROG_TRACK_NAME
                                          , CONFIG_PROG_HBRIDGE_TYPE_NAME
                                          , progCfg)
#if CONFIG_OPS_RAILCOM
                             , railComHub_(railComHub)
#endif // CONFIG_OPS_RAILCOM
{
  // register the VFS handler as the LocalTrackIf uses this to route DCC
  // packets to the track.
  esp_vfs_t vfs;
  memset(&vfs, 0, sizeof(vfs));
  vfs.flags = ESP_VFS_FLAG_CONTEXT_PTR;
  vfs.ioctl_p = &rmt_track_ioctl;
  vfs.open_p = &rmt_track_open;
  vfs.close_p = &rmt_track_close;
  vfs.write_p = &rmt_track_write;
  ESP_ERROR_CHECK(esp_vfs_register("/dev/track", &vfs, this));

  // TODO: move the RMT init and RailCom UART init to be called from dedicated
  // task and block here until it completes.

  Singleton<StatusDisplay>::instance()->status("RMT Init");
  //xTaskCreatePinnedToCore(rmt_init, CONFIG_OPS_TRACK_NAME, 2048, this, 5, nullptr, APP_CPU_NUM);
  //xTaskCreatePinnedToCore(rmt_init, CONFIG_PROG_TRACK_NAME, 2048, this, 5, nullptr, APP_CPU_NUM);
  initRMTDevice(CONFIG_OPS_TRACK_NAME, opsRMTChannel_, opsSignalPin_
              , opsPreambleBits_);
  initRMTDevice(CONFIG_PROG_TRACK_NAME, progRMTChannel_, progSignalPin_
              , progPreambleBits_);

  // hook into the RMT ISR to have callbacks when TX completes
  rmt_register_tx_end_callback(rmt_tx_complete_isr_callback, this);

#if CONFIG_OPS_RAILCOM
  LOG(INFO
    , "[OPS] Initializing RailCom detector using UART %d on data pin: %d, "
      "enable pin: %d, h-bridge enable pin:%d, h-bridge brake pin: %d"
    , railComUartPort_, CONFIG_OPS_RAILCOM_UART_RX_PIN, railComEnablePin_
    , opsOutputEnablePin_, railComBrakeEnablePin_);
  gpio_pad_select_gpio(railComEnablePin_);
  ESP_ERROR_CHECK(gpio_set_direction(railComEnablePin_, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(gpio_pulldown_en(railComEnablePin_));
  ESP_ERROR_CHECK(gpio_set_level(railComEnablePin_, 0));
  gpio_pad_select_gpio(railComBrakeEnablePin_);
  ESP_ERROR_CHECK(gpio_set_direction(railComBrakeEnablePin_, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(gpio_pullup_en(railComBrakeEnablePin_));
  ESP_ERROR_CHECK(gpio_set_level(railComBrakeEnablePin_, 1));
  uart_config_t uart =
  {
    .baud_rate           = 250000L,
    .data_bits           = UART_DATA_8_BITS,         // 8 bit bytes
    .parity              = UART_PARITY_DISABLE,      // no partity
    .stop_bits           = UART_STOP_BITS_1,         // one stop bit
    .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE, // no flow control
    .rx_flow_ctrl_thresh = 0,                        // unused
    .use_ref_tick        = false                     // unused
  };
  ESP_ERROR_CHECK(uart_param_config(railComUartPort_, &uart));
  ESP_ERROR_CHECK(uart_set_pin(railComUartPort_, UART_PIN_NO_CHANGE
                              , CONFIG_OPS_RAILCOM_UART_RX_PIN
                              , UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(
    uart_driver_install(railComUartPort_    // hardware UART index
                      , UART_FIFO_LEN + 1   // RX buffer size
                      , 0                   // TX buffer size (disabled)
                      , 0                   // event queue size (disabled)
                      , NULL                // event queue handle (unused)
                      , 0));                // ISR flags (defaults)

  // disable the default IDF UART ISR for RailCom port
  ESP_ERROR_CHECK(uart_isr_free(railComUartPort_));
  uart_intr_config_t uart_intr =
  {
    .intr_enable_mask = 0                          // none enabled by default
  , .rx_timeout_thresh = UART_TOUT_THRESH_DEFAULT
  , .txfifo_empty_intr_thresh = 0                  // unused
  , .rxfifo_full_thresh = UART_FULL_THRESH_DEFAULT
  };
  ESP_ERROR_CHECK(uart_isr_register(railComUartPort_, railcom_uart_isr, this
                                  , RAILCOM_ISR_FLAGS, nullptr));
  ESP_ERROR_CHECK(uart_intr_config(railComUartPort_, &uart_intr));
  LOG(INFO, "[OPS] RailCom detector configured");
  railcomReader_ = std::bind(&RMTTrackDevice::read_railcom_response, this);
  railcomEnabled_ = true;
#endif // CONFIG_OPS_RAILCOM

  // add a callback to update the initial state of the track output rather than
  // call it here since the h-bridge code may not be fully "up" yet.
  service->executor()->add(new CallbackExecutable([this]
  {
    update_status_display();
  }));

}

///////////////////////////////////////////////////////////////////////////////
// ESP VFS callback for ::write()
//
// This will write *ONE* dcc::Packet to either the OPS or PROG packet queue. If
// there is no space in the packet queue the packet will be rejected and errno
// set to ENOSPC. If the track output is not enabled the packet will be
// discarded without an error being reported.
//
// NOTE: At this time Marklin packets will be actively rejected.
///////////////////////////////////////////////////////////////////////////////
ssize_t RMTTrackDevice::write(int fd, const void * data, size_t size)
{
  if (size != sizeof(dcc::Packet))
  {
    errno = EINVAL;
    return -1;
  }
  const dcc::Packet *sourcePacket{(dcc::Packet *)data};

  if (sourcePacket->packet_header.is_marklin)
  {
    // drop marklin packets as unsupported for now.
    errno = ENOTSUP;
    return -1;
  }

  // Check if it is a PROG track packet
  if (sourcePacket->packet_header.send_long_preamble && progSignalActive_)
  {
    AtomicHolder l(&progPacketQueueLock_);
    dcc::Packet* writePacket;
    if (progPacketQueue_->space() &&
        progPacketQueue_->data_write_pointer(&writePacket))
    {
      memcpy(writePacket, data, size);
      progPacketQueue_->advance(1);
      return 1;
    }
  }
  else if (opsSignalActive_)
  {
    AtomicHolder l(&opsPacketQueueLock_);
    dcc::Packet* writePacket;
    if (opsPacketQueue_->space() &&
        opsPacketQueue_->data_write_pointer(&writePacket))
    {
      memcpy(writePacket, data, size);
      opsPacketQueue_->advance(1);
      return 1;
    }
  }
  else
  {
#if CONFIG_DCC_RMT_LOG_LEVEL == VERBOSE
    dcc::Packet pkt;
    memcpy(&pkt, data, size);
    LOG(CONFIG_DCC_RMT_LOG_LEVEL, "[RMT] Discarding packet as track is not ENABLED:\n%s",
        dcc::packet_to_string(pkt).c_str());
#endif // CONFIG_DCC_RMT_LOG_LEVEL == VERBOSE
    // discard packet
    return 1;
  }

  // packet queue is full!
  errno = ENOSPC;
  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// ESP VFS callback for ::open()
//
// This is currently implemented to always return the fd of zero unless the
// device has already been opened in which case errno will be set to EINVAL.
///////////////////////////////////////////////////////////////////////////////
int RMTTrackDevice::open(const char * path, int flags, int mode)
{
  if (devOpened_)
  {
    LOG_ERROR("[RMT] Attempt to open %s after it has already been opened"
            , path);
    errno = -EINVAL;
    return -1;
  }
  devOpened_ = true;
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// ESP VFS callback for ::close()
//
// Note: no error will be raised if the track device has not been opened.
///////////////////////////////////////////////////////////////////////////////
int RMTTrackDevice::close(int fd)
{
  if (!devOpened_)
  {
    LOG_ERROR("[RMT] Attempt to close fd %d without it being open", fd);
  }
  devOpened_ = false;
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// ESP VFS callback for ::ioctl()
//
// When the cmd is CAN_IOC_WRITE_OPS_ACTIVE the OPS packet queue will be
// queried. When the cmd is CAN_IOC_WRITE_PROG_ACTIVE the PROG packet queue
// will be queried.
//
// If there is space in the queue the args will be converted to a Notifiable
// object which will be notified of space available or to requeue themselves.
///////////////////////////////////////////////////////////////////////////////
int RMTTrackDevice::ioctl(int fd, int cmd, va_list args)
{
  // Attempt to write a Packet to the queue
  if (IOC_TYPE(cmd) == CAN_IOC_MAGIC &&
      IOC_SIZE(cmd) == NOTIFIABLE_TYPE &&
      cmd == CAN_IOC_WRITE_OPS_ACTIVE)
  {
    Notifiable* n = reinterpret_cast<Notifiable*>(va_arg(args, uintptr_t));
    HASSERT(n);
    {
      AtomicHolder l(&opsPacketQueueLock_);
      if (!opsPacketQueue_->space())
      {
        // stash the notifiable so we can call it later when there is space
        std::swap(n, opsWritableNotifiable_);
      }
    }
    if (n)
    {
      n->notify();
    }
    return 0;
  }

  // Attempt to write a Packet to the queue
  if (IOC_TYPE(cmd) == CAN_IOC_MAGIC &&
      IOC_SIZE(cmd) == NOTIFIABLE_TYPE &&
      cmd == CAN_IOC_WRITE_PROG_ACTIVE)
  {
    Notifiable* n = reinterpret_cast<Notifiable*>(va_arg(args, uintptr_t));
    HASSERT(n);
    {
      AtomicHolder l(&progPacketQueueLock_);
      if (!progPacketQueue_->space())
      {
        // stash the notifiable so we can call it later when there is space
        std::swap(n, progWritableNotifiable_);
      }
    }
    if (n)
    {
      n->notify();
    }
    return 0;
  }
  // Unknown ioctl operation
  errno = EINVAL;
  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// RMT transmit complete for the OPS RMT channel.
//
// When RailCom is enabled this will poll for RailCom data before transmission
// of the next dcc::Packet from the queue.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::ops_rmt_transmit_complete()
{
#if CONFIG_OPS_RAILCOM
  // Process any pending RailCom data
  railcomReader_();
#endif // CONFIG_OPS_RAILCOM

  // If the OPS signal is active generate the next packet for TX
  if (opsSignalActive_)
  {
    // retrieve the next DCC packet to 
    encode_next_ops_packet();

    // send the packet to the RMT, note not using memcpy for the packet as this
    // directly accesses hardware registers.
    RMT_SET_FIFO(RMT_DATA_MODE_MEM);
    for(uint32_t index = 0; index < opsEncodedLength_; index++)
    {
      RMT_SET_DATA(opsRMTChannel_, index) =  opsEncodedPacket_[index].val;
    }
    // RMT marker for "end of data"
    RMT_SET_DATA(opsRMTChannel_, opsEncodedLength_) = 0;
    // start transmit
    RMT_CONFIG(opsRMTChannel_).mem_rd_rst = 1;
    RMT_CONFIG(opsRMTChannel_).mem_owner = RMT_MEM_OWNER_TX;
    RMT_CONFIG(opsRMTChannel_).tx_start = 1;
  }
}

///////////////////////////////////////////////////////////////////////////////
// RMT transmit complete for the PROG RMT channel.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::prog_rmt_transmit_complete()
{
  // If the PROG signal is active generate the next packet for TX
  if (progSignalActive_)
  {
    // retrieve the next DCC packet to 
    encode_next_prog_packet();

    // send the packet to the RMT, note not using memcpy for the packet as this
    // directly accesses hardware registers.
    RMT_SET_FIFO(RMT_DATA_MODE_MEM);
    for(uint32_t index = 0; index < progEncodedLength_; index++)
    {
      RMT_SET_DATA(progRMTChannel_, index) =  progEncodedPacket_[index].val;
    }
    // RMT marker for "end of data"
    RMT_SET_DATA(progRMTChannel_, progEncodedLength_) = 0;
    // start transmit
    RMT_CONFIG(progRMTChannel_).mem_rd_rst = 1;
    RMT_CONFIG(progRMTChannel_).mem_owner = RMT_MEM_OWNER_TX;
    RMT_CONFIG(progRMTChannel_).tx_start = 1;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Transfers a dcc::Packet to the OPS packet queue.
//
// NOTE: At this time Marklin packets will be actively discarded.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::send(Buffer<dcc::Packet> *b, unsigned prio)
{
  // if it is not a Marklin Motorola packet put it in the queue, otherwise
  // discard.
  if (!b->data()->packet_header.is_marklin)
  {
    AtomicHolder l(&opsPacketQueueLock_);
    dcc::Packet* writePacket;
    if (opsPacketQueue_->space() &&
        opsPacketQueue_->data_write_pointer(&writePacket))
    {
      memcpy(writePacket, b->data(), b->size());
      opsPacketQueue_->advance(1);
    }
  }
  b->unref();
}

///////////////////////////////////////////////////////////////////////////////
// Enables the OPS track output if not already enabled.
//
// Note: This will queue up a single DCC ONE bit to the RMT to start the
// processing of the OPS packet queue.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::enable_ops_output()
{
  if (!opsSignalActive_)
  {
    opsSignalActive_ = true;
    LOG(INFO, "[RMT] Starting RMT for OPS");

    opsHBridge_.enable();

    update_status_display();

    // send one bit to kickstart the signal, remaining data will come from the
    // packet queue. We intentionally do not wait for the RMT TX complete here.
    rmt_write_items(opsRMTChannel_, &DCC_RMT_ONE_BIT, 1, false);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Disables the OPS track output if enabled.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::disable_ops_output()
{
  if (opsSignalActive_)
  {
    opsSignalActive_ = false;
    LOG(INFO, "[RMT] Shutting down RMT for OPS");

    opsHBridge_.disable();

    update_status_display();
  }
}

///////////////////////////////////////////////////////////////////////////////
// Enables the PROG track output if not already enabled.
//
// Note: This will queue up a single DCC ONE bit to the RMT to start the
// processing of the PROG packet queue.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::enable_prog_output()
{
  if (!progSignalActive_)
  {
    LOG(INFO, "[RMT] Starting RMT for PROG");
    progSignalActive_ = true;

    progHBridge_.enable();

    update_status_display();

    // send one bit to kickstart the signal, remaining data will come from the
    // packet queue. We intentionally do not wait for the RMT TX complete here.
    rmt_write_items(progRMTChannel_, &DCC_RMT_ONE_BIT, 1, false);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Disables the PROG track output if enabled.
//
// Note: The PROG packet queue will be drained if there are any remaining
// dcc::Packet waiting for transmission.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::disable_prog_output()
{
  if (progSignalActive_)
  {
    progSignalActive_ = false;
    LOG(INFO, "[RMT] Shutting down RMT for PROG");

    progHBridge_.disable();
    update_status_display();

    {
      // if we have any pending packets, consume them now so we do not send
      // them to the track.
      AtomicHolder l(&progPacketQueueLock_);
      if (!progPacketQueue_->pending())
      {
        LOG(INFO, "[RMT] Discarding %d pending packets for PROG"
          , progPacketQueue_->pending());
        progPacketQueue_->consume(progPacketQueue_->pending());
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Generates a JSON array containing the h-bridge status.
///////////////////////////////////////////////////////////////////////////////
string RMTTrackDevice::generate_status_json()
{
  return StringPrintf("[%s,%s]", opsHBridge_.getStateAsJson().c_str()
                    , progHBridge_.getStateAsJson().c_str());
}

///////////////////////////////////////////////////////////////////////////////
// Generates a status payload for the DCCppProtocol interface.
///////////////////////////////////////////////////////////////////////////////
string RMTTrackDevice::get_state_for_dccpp()
{
  return opsHBridge_.get_state_for_dccpp();
}

///////////////////////////////////////////////////////////////////////////////
// Retrieves the status data for the StatusDisplay from the h-bridges.
///////////////////////////////////////////////////////////////////////////////
string RMTTrackDevice::get_status_screen_data()
{
  infoDataFirst_ = !infoDataFirst_;
  if (infoDataFirst_)
  {
    return opsHBridge_.getStatusData();
  }
  return progHBridge_.getStatusData();
}

///////////////////////////////////////////////////////////////////////////////
// Updates the status display with the current state of the track outputs.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::update_status_display()
{
  auto status = Singleton<StatusDisplay>::instance();
  status->track_power("%s:%s %s:%s", CONFIG_OPS_TRACK_NAME
                    , opsHBridge_.isEnabled() ? "On" : "Off"
                    , CONFIG_PROG_TRACK_NAME
                    , progHBridge_.isEnabled() ? "On" : "Off");
}

///////////////////////////////////////////////////////////////////////////////
// Initializes the RMT peripheral for one output channel.
//
// The memory blocks used is calculated based on the preamble bit count and the
// maximum payload for a dcc::Packet. The maximum number of memory blocks is
// capped at three which is enough for 192 bits of total data. Of these 192
// bits up to six are used for packet byte markers and three are used for the
// packet start, end markers and one sacrificial bit to ensure the RMT does not
// stretch the final bit of the payload.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::initRMTDevice(const char *name
                                 , rmt_channel_t channel
                                 , gpio_num_t pin
                                 , uint8_t preambleCount)
{
  uint16_t maxBitCount = preambleCount                    // preamble bits
                        + 1                               // packet start bit
                        + (dcc::Packet::MAX_PAYLOAD * 8)  // payload bits
                        +  dcc::Packet::MAX_PAYLOAD       // end of byte bits
                        + 1                               // end of packet bit
                        + 1;                              // RMT extra bit
  HASSERT(maxBitCount <= MAX_DCC_PACKET_BITS);

  uint8_t memoryBlocks = (maxBitCount / RMT_MEM_ITEM_NUM) + 1;
  HASSERT(memoryBlocks <= MAX_RMT_MEMORY_BLOCKS);

  LOG(INFO, "[%s] Using RMT(%d), pin: %d, memory: %d blocks (%d/%d bits), "
            "clk-div: %d, DCC bit timing: zero: %duS, one: %duS"
          , name, channel, pin, memoryBlocks, maxBitCount
          , (memoryBlocks * RMT_MEM_ITEM_NUM), RMT_CLOCK_DIVIDER
          , DCC_ZERO_BIT_PULSE_USEC, DCC_ONE_BIT_PULSE_USEC);
  rmt_config_t opsRMTConfig =
  {
    .rmt_mode = RMT_MODE_TX,
    .channel = channel,
    .clk_div = RMT_CLOCK_DIVIDER,
    .gpio_num = pin,
    .mem_block_num = memoryBlocks,
    {
      .tx_config =
      {
        .loop_en = false,
        .carrier_freq_hz = 0,
        .carrier_duty_percent = 0,
        .carrier_level = rmt_carrier_level_t::RMT_CARRIER_LEVEL_LOW,
        .carrier_en = false,
        .idle_level = rmt_idle_level_t::RMT_IDLE_LEVEL_LOW,
        .idle_output_en = false
      }
    }
  };
  ESP_ERROR_CHECK(rmt_config(&opsRMTConfig));
  ESP_ERROR_CHECK(rmt_driver_install(channel, 0, RMT_ISR_FLAGS));
}

///////////////////////////////////////////////////////////////////////////////
// Encodes the next OPS track dcc::Packet from the packet queue.
//
// Optionally will allocate and send RailCom feedback.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::encode_next_ops_packet()
{
#if CONFIG_OPS_RAILCOM
  if (railcomEnabled_)
  {
    send_railcom_response_buffer(opsPacketRepeatCount_ > 0);
    ENCODE_PACKET(opsPacketQueue_, opsPacketQueueLock_, opsPreambleBits_
                , opsEncodedPacket_, opsEncodedLength_
                , opsPacketRepeatCount_, opsWritableNotifiable_
                , alloc_railcom_response_buffer)
  }
  else
  {
    ENCODE_PACKET(opsPacketQueue_, opsPacketQueueLock_, opsPreambleBits_
                , opsEncodedPacket_, opsEncodedLength_
                , opsPacketRepeatCount_, opsWritableNotifiable_
                , noop_alloc_railcom_response_buffer)
  }
#else
  ENCODE_PACKET(opsPacketQueue_, opsPacketQueueLock_, opsPreambleBits_
              , opsEncodedPacket_, opsEncodedLength_
              , opsPacketRepeatCount_, opsWritableNotifiable_
              , noop_alloc_railcom_response_buffer)
#endif // CONFIG_OPS_RAILCOM
}

///////////////////////////////////////////////////////////////////////////////
// Encodes the next PROG track dcc::Packet from the packet queue.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::encode_next_prog_packet()
{
  ENCODE_PACKET(progPacketQueue_, progPacketQueueLock_, progPreambleBits_
              , progEncodedPacket_ , progEncodedLength_
              , progPacketRepeatCount_, progWritableNotifiable_
              , noop_alloc_railcom_response_buffer)
}

#if CONFIG_OPS_RAILCOM
///////////////////////////////////////////////////////////////////////////////
// Sends the last allocated dcc::RailcomHubData to the RailComHubFlow.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::send_railcom_response_buffer(bool need_more)
{
  if (railComFeedback_)
  {
    // send the feedback to the hub
    railComHub_->send(railComFeedback_);
    railComFeedback_ = nullptr;

    // if we need another feedback packet allocate it now with the same key
    if (need_more)
    {
      alloc_railcom_response_buffer(railcomFeedbackKey_);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Allocates a new dcc::RailcomHubData to receive RailCom data.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::alloc_railcom_response_buffer(uintptr_t key)
{
  railComFeedback_ = railComHub_->alloc();
  if (railComFeedback_)
  {
    railComFeedback_->data()->reset(key);
    railcomFeedbackKey_ = key;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Reads RailCom feedback data from the OPS h-bridge
//
// TODO: this needs to be reworked to directly access the UART RX FIFO rather
// than uart_read_bytes since this is called within an ISR context.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::read_railcom_response()
{
  // Ensure the signal pin is LOW before starting RailCom detection
  _set_gpio_state(opsSignalPin_, false);
  ets_delay_us(RAILCOM_PACKET_END_DELAY_USEC);

  // Enable the BRAKE pin on the h-bridge to force it into coast mode
  _set_gpio_state(railComBrakeEnablePin_, true);

  // Disable the h-bridge output after a short delay to allow the h-bridge to
  // sink remaining current.
  ets_delay_us(RAILCOM_BRAKE_ENABLE_DELAY_USEC);
  _set_gpio_state(opsOutputEnablePin_, false);

  // enable the UART RX ISR
  SET_PERI_REG_MASK(UART_INT_CLR_REG(railComUartPort_), UART_RX_INT_EN_MASK);
  SET_PERI_REG_MASK(UART_INT_ENA_REG(railComUartPort_), UART_RX_INT_EN_MASK);

  // enable the RailCom receiver circuitry
  _set_gpio_state(railComEnablePin_, true);

  // Allow the RailCom detector to stabilize before we try and read data. Any
  // data received during this stabilization period will be discarded.
  ets_delay_us(RAILCOM_ENABLE_TRANSIENT_DELAY_USEC);

  // Tell the RailCom detector that we are ready for channel 1 data
  railcomReaderCh1_ = true;

  // Give the RailCom ISR time to read channel 1
  ets_delay_us(RAILCOM_MAX_READ_DELAY_CH_1);
  railcomReaderCh1_ = false;

  // Tell the RailCom detector that we are ready for channel 2 data
  ets_delay_us(RAILCOM_ENABLE_TRANSIENT_DELAY_USEC);
  railcomReaderCh2_ = true;

  // Give the RailCom ISR time to read channel 2
  ets_delay_us(RAILCOM_MAX_READ_DELAY_CH_2);
  railcomReaderCh2_ = false;

  // Disable the UART RX ISR
  CLEAR_PERI_REG_MASK(UART_INT_ENA_REG(railComUartPort_), UART_RX_INT_EN_MASK);

  // Disable the RailCom detector circuitry
  _set_gpio_state(railComEnablePin_, false);
  _set_gpio_state(opsOutputEnablePin_, true);

  ets_delay_us(RAILCOM_BRAKE_DISABLE_DELAY_USEC);
  _set_gpio_state(railComBrakeEnablePin_, false);
}

void RMTTrackDevice::railcom_data_received()
{
  while (_uart_available(railComUartPort_))
  {
    if (_get_gpio_state(railComShortPin_))
    {
      ets_printf("RailCom Short Detected!!!");
    }
    if (!railComFeedback_)
    {
      // If railcom is disabled *OR* we don't have a dcc::RailcomHubData
      // to hold the data flush the UART.
      _uart_flush(railComUartPort_);
    }
    else
    {
      dcc::RailcomHubData *data = railComFeedback_->data();
      if (railcomReaderCh1_)
      {
        // If we are reading channel 1 retrieve a byte of data and add it to
        // the dcc::Feedback packet.
        data->add_ch1_data(_uart_read(railComUartPort_));
      }
      else if (railcomReaderCh2_)
      {
        // If we are reading channel 2 retrieve a byte of data and add it to
        // the dcc::Feedback packet.
        data->add_ch2_data(_uart_read(railComUartPort_));
      }
      else
      {
        // we are either in beween channels or we have reached capacity for
        // the current channel. Discard the received data.
        _uart_flush(railComUartPort_);
      }
    }
  }
  // clear the ISR mask
  UART[railComUartPort_]->int_clr.val = UART_RX_INT_CLR_MASK;
}

///////////////////////////////////////////////////////////////////////////////
// NO-OP RailCom reader that is used when RailCom is not enabled or for the
// PROG track.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::read_railcom_response_noop()
{
}
#endif // CONFIG_OPS_RAILCOM