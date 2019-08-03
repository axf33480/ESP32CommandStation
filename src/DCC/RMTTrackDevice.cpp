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

#include "ESP32CommandStation.h"

/////////////////////////////////////////////////////////////////////////////////////
// DCC packet queue sizes for the RMT driver
/////////////////////////////////////////////////////////////////////////////////////
static constexpr uint8_t OPS_QUEUE_LEN = 25;
static constexpr uint8_t PROG_QUEUE_LEN = 10;

/////////////////////////////////////////////////////////////////////////////////////
// OPS track h-bridge settings
/////////////////////////////////////////////////////////////////////////////////////
#define L298          0
#define LMD18200      1
#define POLOLU        2
#define BTS7960B_5A   3
#define BTS7960B_10A  4

#if OPS_HBRIDGE_TYPE == L298
#define OPS_HBRIDGE_MAX_MILIAMPS 2000
#define OPS_HBRIDGE_LIMIT_MILIAMPS 2000
#define OPS_HBRIDGE_TYPE_NAME "L298"
#elif OPS_HBRIDGE_TYPE == LMD18200
#define OPS_HBRIDGE_MAX_MILIAMPS 3000
#define OPS_HBRIDGE_LIMIT_MILIAMPS 3000
#define OPS_HBRIDGE_TYPE_NAME "LMD18200"
#elif OPS_HBRIDGE_TYPE == POLOLU
#define OPS_HBRIDGE_MAX_MILIAMPS 2500
#define OPS_HBRIDGE_LIMIT_MILIAMPS 2500
#define OPS_HBRIDGE_TYPE_NAME "POLOLU"
#elif OPS_HBRIDGE_TYPE == BTS7960B_5A
#define OPS_HBRIDGE_MAX_MILIAMPS 43000
#define OPS_HBRIDGE_LIMIT_MILIAMPS 5000
#define OPS_HBRIDGE_TYPE_NAME "BTS7960B"
#elif OPS_HBRIDGE_TYPE == BTS7960B_10A
#define OPS_HBRIDGE_MAX_MILIAMPS 43000
#define OPS_HBRIDGE_LIMIT_MILIAMPS 10000
#define OPS_HBRIDGE_TYPE_NAME "BTS7960B"
#else
#error "Unrecognized OPS_HBRIDGE_TYPE"
#endif

/////////////////////////////////////////////////////////////////////////////////////
// PROG track h-bridge settings
/////////////////////////////////////////////////////////////////////////////////////
#if PROG_HBRIDGE_TYPE == L298
#define PROG_HBRIDGE_MAX_MILIAMPS 2000
#define PROG_HBRIDGE_TYPE_NAME "L298"
#elif PROG_HBRIDGE_TYPE == LMD18200
#define PROG_HBRIDGE_MAX_MILIAMPS 3000
#define PROG_HBRIDGE_TYPE_NAME "LMD18200"
#elif PROG_HBRIDGE_TYPE == POLOLU
#define PROG_HBRIDGE_MAX_MILIAMPS 2500
#define PROG_HBRIDGE_TYPE_NAME "POLOLU"
#elif PROG_HBRIDGE_TYPE == BTS7960B_5A
#define PROG_HBRIDGE_MAX_MILIAMPS 43000
#define PROG_HBRIDGE_TYPE_NAME "BTS7960B"
#elif PROG_HBRIDGE_TYPE == BTS7960B_10A
#define PROG_HBRIDGE_MAX_MILIAMPS 43000
#define PROG_HBRIDGE_TYPE_NAME "BTS7960B"
#else
#error "Unrecognized PROG_HBRIDGE_TYPE"
#endif

// APB/REF clock divider to use for the RMT module
static constexpr uint8_t RMT_CLOCK_DIVIDER = 80;

// number of microseconds for each half of the DCC signal for a zero
static constexpr uint32_t DCC_ZERO_BIT_PULSE_USEC = 96;

// number of microseconds for each half of the DCC signal for a one
static constexpr uint32_t DCC_ONE_BIT_PULSE_USEC = 58;

// pre-encoded DCC ZERO bit.
static constexpr rmt_item32_t DCC_RMT_ZERO_BIT =
{{{
    DCC_ZERO_BIT_PULSE_USEC       // number of microseconds for TOP half
  , 1                             // of the square wave.
  , DCC_ZERO_BIT_PULSE_USEC       // number of microseconds for BOTTOM half
  , 0                             // of the square wave.
}}};

// pre-encoded DCC ONE bit.
static constexpr rmt_item32_t DCC_RMT_ONE_BIT =
{{{
    DCC_ONE_BIT_PULSE_USEC        // number of microseconds for TOP half
  , 1                             // of the square wave.
  , DCC_ONE_BIT_PULSE_USEC        // number of microseconds for BOTTOM half
  , 0                             // of the square wave.
}}};

// Declare ISR flags for the RMT driver ISR and feeder ISR
static constexpr uint32_t RMT_ISR_FLAGS = (
    ESP_INTR_FLAG_LEVEL2              // ISR is implemented in C code
  | ESP_INTR_FLAG_SHARED              // ISR is shared across multiple handlers
);

static constexpr DRAM_ATTR uint8_t PACKET_BIT_MASK[] =
{
  0x80, 0x40, 0x20, 0x10, //
  0x08, 0x04, 0x02, 0x01  //
};

static ssize_t rmt_track_write(void* ctx, int fd, const void * data, size_t size)
{
  return reinterpret_cast<RMTTrackDevice *>(ctx)->write(fd, data, size);
}

static int rmt_track_open(void* ctx, const char * path, int flags, int mode)
{
  return reinterpret_cast<RMTTrackDevice *>(ctx)->open(path, flags, mode);
}

static int rmt_track_close(void* ctx, int fd)
{
  return reinterpret_cast<RMTTrackDevice *>(ctx)->close(fd);
}

static int rmt_track_ioctl(void* ctx, int fd, int cmd, va_list args)
{
  return reinterpret_cast<RMTTrackDevice *>(ctx)->ioctl(fd, cmd, args);
}

static void rmt_tx_complete_isr_callback(void *ctx)
{
  RMTTrackDevice *track = reinterpret_cast<RMTTrackDevice *>(ctx);
  // Check if the channel 0 TX threshold has been triggered
  // if so the OPS TX has completed and we need to clear the event
  if (RMT.int_st.ch0_tx_thr_event)
  {
    RMT.int_clr.ch0_tx_thr_event = 1;
    track->ops_rmt_transmit_complete();
  }

  // Check if the channel 1 TX threshold has been triggered
  // if so the PROG TX has completed and we need to clear the event
  if (RMT.int_st.ch1_tx_thr_event)
  {
    RMT.int_clr.ch1_tx_thr_event = 1;
    track->prog_rmt_transmit_complete();
  }
}

#define WRITE_PACKET_TO_QUEUE(queue, lock, data, size)  \
      Packet* packet;                                   \
      AtomicHolder l(&lock);                            \
      if (queue->space() &&                             \
          queue->data_write_pointer(&packet))           \
      {                                                 \
        memcpy(packet, data, size);                     \
        queue->advance(1);                              \
      }                                                 \
      else                                              \
      {                                                 \
        packetQueueOverrunCount_++;                     \
      }

#define ENCODE_PACKET(queue, preambleCount, encodedPacket                       \
                    , encodedLength, repeatCount, notifiable)                   \
    /* decrement the repeat count and if we still have at least one repeat
       remaining skip reencoding the packet */                                  \
    if (--repeatCount >= 0)                                                     \
    {                                                                           \
      return;                                                                   \
    }                                                                           \
    /* attempt to fetch a packet from the queue or use an idle packet */        \
    Packet *packet = nullptr;                                                   \
    {                                                                           \
      AtomicHolder l(&packetQueueLock_);                                        \
      if ((opsPacketQueue_->pending() &&                                        \
          !opsPacketQueue_->get(packet, 1)) ||                                  \
          !packet)                                                              \
      {                                                                         \
        /* no packet retrieved, default to the IDLE packet. */                  \
        packet = &idlePacket_;                                                  \
      }                                                                         \
      /* Check if we have a pending writer and notify it if needed. */          \
      Notifiable* n = nullptr;                                                  \
      std::swap(n, notifiable);                                                 \
      if (n)                                                                    \
      {                                                                         \
        n->notify_from_isr();                                                   \
      }                                                                         \
    }                                                                           \
    /* encode the preamble bits */                                              \
    for (encodedLength = 0; encodedLength < preambleCount; encodedLength++)     \
    {                                                                           \
      encodedPacket[encodedLength].val = DCC_RMT_ONE_BIT.val;                   \
    }                                                                           \
    /* encode the packet bits */                                                \
    for (uint8_t dlc = 0; dlc < packet->dlc; dlc++)                             \
    {                                                                           \
      for(uint8_t bit = 0; bit < 8; bit++)                                      \
      {                                                                         \
        encodedPacket[encodedLength++].val =                                    \
          packet->payload[dlc] & PACKET_BIT_MASK[bit] ?                         \
            DCC_RMT_ONE_BIT.val : DCC_RMT_ZERO_BIT.val;                         \
      }                                                                         \
      /* end of byte marker */                                                  \
      encodedPacket[encodedLength++].val = DCC_RMT_ZERO_BIT.val;                \
    }                                                                           \
    /* set the last bit of the encoded payload to be an end of packet marker */ \
    encodedPacket[encodedLength - 1].val = DCC_RMT_ONE_BIT.val;                 \
    /* add an extra ONE bit to the end to prevent mangling of the last bit by
       the RMT */                                                               \
    encodedPacket[encodedLength++].val = DCC_RMT_ONE_BIT.val;                   \
    /* record the repeat count */                                               \
    repeatCount = packet->packet_header.rept_count + 1;

RMTTrackDevice::RMTTrackDevice(SimpleCanStack *stack
                             , RailcomHubFlow *railComHub
                             , const TrackOutputConfig &opsCfg
                             , const TrackOutputConfig &progCfg
                             , const rmt_channel_t opsChannel
                             , const gpio_num_t opsSignalPin
                             , const uint8_t opsPreambleBits
                             , const gpio_num_t opsOutputEnablePin
                             , const gpio_num_t opsThermalPin
                             , const adc1_channel_t opsSenseChannel
                             , const rmt_channel_t progChannel
                             , const gpio_num_t progSignalPin
                             , const uint8_t progPreambleBits
                             , const gpio_num_t progOutputEnablePin
                             , const adc1_channel_t progSenseChannel
                             , const gpio_num_t railComBrakeEnablePin
                             , const gpio_num_t railComEnablePin
                             , const gpio_num_t railComShortPin
                             , const uart_port_t railComUART
                             , const gpio_num_t railComReceivePin)
                             : BitEventInterface(Defs::CLEAR_EMERGENCY_OFF_EVENT
                                               , Defs::EMERGENCY_OFF_EVENT)
                             , stack_(stack)
                             , opsSignalPin_(opsSignalPin)
                             , opsRMTChannel_(opsChannel)
                             , opsPreambleBits_(opsPreambleBits)
                             , opsOutputEnablePin_(opsOutputEnablePin)
                             , opsPacketQueue_(DeviceBuffer<Packet>::create(OPS_QUEUE_LEN))
                             , progSignalPin_(progSignalPin)
                             , progRMTChannel_(progChannel)
                             , progPreambleBits_(progPreambleBits)
                             , progOutputEnablePin_(progOutputEnablePin)
                             , progPacketQueue_(DeviceBuffer<Packet>::create(PROG_QUEUE_LEN))
                             , railComBrakeEnablePin_(railComBrakeEnablePin)
                             , railComEnablePin_(railComEnablePin)
                             , railComShortPin_(railComShortPin)
                             , railComUartPort_(railComUART)
                             , railComHub_(railComHub)
{
  esp_vfs_t vfs;
  memset(&vfs, 0, sizeof(vfs));
  vfs.flags = ESP_VFS_FLAG_CONTEXT_PTR;
  vfs.ioctl_p = &rmt_track_ioctl;
  vfs.open_p = &rmt_track_open;
  vfs.close_p = &rmt_track_close;
  vfs.write_p = &rmt_track_write;
  ESP_ERROR_CHECK(esp_vfs_register("/dev/track", &vfs, this));

  infoScreen->replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "RMT Init");
  initRMTDevice("OPS", opsRMTChannel_, opsSignalPin_, opsPreambleBits_);
  initRMTDevice("PROG", progRMTChannel_, progSignalPin_, progPreambleBits_);

  esp_intr_alloc(ETS_RMT_INTR_SOURCE            // RMT peripheral ISR source
               , RMT_ISR_FLAGS                  // ISR flags (shared, C code)
               , rmt_tx_complete_isr_callback   // callback function
               , this                           // arg for the callback
               , nullptr);                      // discard ISR handle

  if (railComEnablePin_ != NOT_A_PIN)
  {
    LOG(INFO
    , "Initializing RailCom detector (hb-en:%d,rc-en:%d,br-en:%d,rc:%d,uart:%d)"
    , opsOutputEnablePin_
    , railComEnablePin_
    , railComBrakeEnablePin_
    , railComReceivePin
    , railComUartPort_);
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
    ESP_ERROR_CHECK(uart_driver_install(railComUartPort_, UART_FIFO_LEN + 1, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_disable_rx_intr(railComUartPort_));
    ESP_ERROR_CHECK(uart_disable_tx_intr(railComUartPort_));
    railcomReader_ = std::bind(&RMTTrackDevice::read_railcom_response, this);
    railcomEnabled_ = true;
  }

  // if we do not have a RailCom reader implementation at this point we either
  // don't have it enabled or init failed. Asign the default no-op handler.
  if (!railcomReader_)
  {
    railcomReader_ = std::bind(&RMTTrackDevice::read_railcom_response_noop
                             , this);
  }

  opsHBridge_.reset(
    new MonitoredHBridge(stack
                       , opsSenseChannel
                       , opsOutputEnablePin_
                       , opsThermalPin
                       , OPS_HBRIDGE_LIMIT_MILIAMPS
                       , OPS_HBRIDGE_MAX_MILIAMPS
                       , "OPS"
                       , OPS_HBRIDGE_TYPE_NAME
                       , opsCfg)
  );

  progHBridge_.reset(
    new MonitoredHBridge(stack
                       , progSenseChannel
                       , progOutputEnablePin_
                       , PROG_HBRIDGE_MAX_MILIAMPS
                       , "PROG"
                       , PROG_HBRIDGE_TYPE_NAME
                       , progCfg)
  );
}

ssize_t RMTTrackDevice::write(int fd, const void * data, size_t size)
{
  if (size != sizeof(Packet))
  {
    errno = EINVAL;
    return -1;
  }
  const Packet *sourcePacket{(Packet *)data};

  if (sourcePacket->packet_header.is_marklin)
  {
    // drop marklin packets as unsupported for now.
    errno = ENOTSUP;
    return -1;
  }

  // check if it is a PROG packet or OPS packet
  if (sourcePacket->packet_header.send_long_preamble && progSignalActive_)
  {
    // looks like a programming track packet, send it there.
    WRITE_PACKET_TO_QUEUE(progPacketQueue_, packetQueueLock_, data, size)
    return 1;
  }
  else if (opsSignalActive_)
  {
    // looks like an OPS track packet, send it there.
    WRITE_PACKET_TO_QUEUE(opsPacketQueue_, packetQueueLock_, data, size)
    return 1;
  }
  else
  {
    if (LOGLEVEL >= VERBOSE)
    {
      Packet pkt;
      memcpy(&pkt, data, size);
      LOG(VERBOSE, "[RMT] Discarding packet as track is not ENABLED:\n%s",
          dcc::packet_to_string(pkt).c_str());
    }
    // discard packet
    return 1;
  }

  // packet queue is full!
  errno = ENOSPC;
  return -1;
}

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

int RMTTrackDevice::close(int fd)
{
  if (!devOpened_)
  {
    LOG_ERROR("[RMT] Attempt to close fd %d without it being open", fd);
  }
  devOpened_ = false;
  return 0;
}

int RMTTrackDevice::ioctl(int fd, int cmd, va_list args)
{
  // Attempt to write a Packet to the queue
  if (IOC_TYPE(cmd) == CAN_IOC_MAGIC &&
      IOC_SIZE(cmd) == NOTIFIABLE_TYPE &&
      cmd == CAN_IOC_WRITE_OPS_ACTIVE)
  {
    AtomicHolder l(&packetQueueLock_);
    Notifiable* n = reinterpret_cast<Notifiable*>(va_arg(args, uintptr_t));
    HASSERT(n);
    if (!opsPacketQueue_->space())
    {
      unsigned state = portENTER_CRITICAL_NESTED();
      if (!opsPacketQueue_->space())
      {
        // stash the notifiable so we can call it later when there is space
        std::swap(n, opsWritableNotifiable_);
      }
      portEXIT_CRITICAL_NESTED(state);
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
    AtomicHolder l(&packetQueueLock_);
    Notifiable* n = reinterpret_cast<Notifiable*>(va_arg(args, uintptr_t));
    HASSERT(n);
    if (!progPacketQueue_->space())
    {
      unsigned state = portENTER_CRITICAL_NESTED();
      if (!progPacketQueue_->space())
      {
        // stash the notifiable so we can call it later when there is space
        std::swap(n, progWritableNotifiable_);
      }
      portEXIT_CRITICAL_NESTED(state);
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

void RMTTrackDevice::ops_rmt_transmit_complete()
{
  // Process any pending RailCom data
  railcomReader_();

  // If the OPS signal is active generate the next packet for TX
  if (opsSignalActive_)
  {
    // retrieve the next DCC packet to 
    encode_next_ops_packet();

    // start transmission of the next DCC packet
    ESP_ERROR_CHECK(rmt_write_items(opsRMTChannel_    // RMT channel to use
                                  , opsEncodedPacket_ // encoded data to TX
                                  , opsEncodedLength_ // number of bits to TX
                                  , false)            // wait for TX complete
    );
  }
}

void RMTTrackDevice::prog_rmt_transmit_complete()
{
  // If the PROG signal is active generate the next packet for TX
  if (progSignalActive_)
  {
    // retrieve the next DCC packet to 
    encode_next_prog_packet();

    // start transmission of the next DCC packet
    ESP_ERROR_CHECK(rmt_write_items(progRMTChannel_    // RMT channel to use
                                  , progEncodedPacket_ // encoded data to TX
                                  , progEncodedLength_ // number of bits to TX
                                  , false)             // wait for TX complete
    );
  }
}

void RMTTrackDevice::send(Buffer<Packet> *b, unsigned prio)
{
  WRITE_PACKET_TO_QUEUE(opsPacketQueue_, packetQueueLock_, b->data(), b->size());
  b->unref();
}

void RMTTrackDevice::enable_ops_output()
{
  if (!opsSignalActive_)
  {
    AtomicHolder l(&packetQueueLock_);
    opsSignalActive_ = true;
    LOG(INFO, "[RMT] Starting RMT for OPS");

    opsHBridge_->enable();

    // send one bit to kickstart the signal, remaining data will come from the
    // packet queue. We intentionally do not wait for the RMT TX complete here.
    rmt_write_items(opsRMTChannel_, &DCC_RMT_ONE_BIT, 1, false);
  }
}

void RMTTrackDevice::disable_ops_output()
{
  if (opsSignalActive_)
  {
    AtomicHolder l(&packetQueueLock_);
    opsSignalActive_ = false;
    LOG(INFO, "[RMT] Shutting down RMT for OPS");

    opsHBridge_->disable();
  }
}

void RMTTrackDevice::enable_prog_output()
{
  if (!progSignalActive_)
  {
    AtomicHolder l(&packetQueueLock_);
    progSignalActive_ = true;
    LOG(INFO, "[RMT] Starting RMT for PROG");

    // send one bit to kickstart the signal, remaining data will come from the
    // packet queue. We intentionally do not wait for the RMT TX complete here.
    rmt_write_items(progRMTChannel_, &DCC_RMT_ONE_BIT, 1, false);

    progHBridge_->enable();
  }
}

void RMTTrackDevice::disable_prog_output()
{
  if (progSignalActive_)
  {
    AtomicHolder l(&packetQueueLock_);
    progSignalActive_ = false;
    LOG(INFO, "[RMT] Shutting down RMT for PROG");

    progHBridge_->disable();

    // if we have any pending packets, consume them now so we do not send them
    // to the track.
    if (progPacketQueue_->pending())
    {
      LOG(INFO, "[RMT] Discarding %d pending packets for PROG"
        , progPacketQueue_->pending());
      progPacketQueue_->consume(progPacketQueue_->pending());
    }
  }
}

void RMTTrackDevice::generate_status_json(JsonArray output)
{
  output.add(serialized(opsHBridge_->getStateAsJson()));
  output.add(serialized(progHBridge_->getStateAsJson()));
}

void RMTTrackDevice::broadcast_status()
{
  opsHBridge_->broadcastStatus();
  progHBridge_->broadcastStatus();
}

string RMTTrackDevice::get_info_screen_data()
{
  infoDataFirst_ = !infoDataFirst_;
  if (infoDataFirst_)
  {
    return opsHBridge_->getInfoScreenData();
  }
  return progHBridge_->getInfoScreenData();
}

void RMTTrackDevice::initRMTDevice(const char *name
                                 , rmt_channel_t channel
                                 , gpio_num_t pin
                                 , uint8_t preambleCount)
{
  uint16_t maxBitCount = preambleCount               // preamble bits
                        + (Packet::MAX_PAYLOAD * 8)  // payload bits
                        +  Packet::MAX_PAYLOAD       // end of byte bits
                        + 1                          // end of packet bit
                        + 1;                         // RMT extra bit
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

void RMTTrackDevice::encode_next_ops_packet()
{
  send_last_railcom_response_buffer(opsPacketRepeatCount_ > 0);

  ENCODE_PACKET(opsPacketQueue_, opsPreambleBits_, opsEncodedPacket_
              , opsEncodedLength_, opsPacketRepeatCount_
              , opsWritableNotifiable_)

  get_next_railcom_response_buffer(packet->feedback_key);
}

void RMTTrackDevice::encode_next_prog_packet()
{
  ENCODE_PACKET(progPacketQueue_, progPreambleBits_, progEncodedPacket_
              , progEncodedLength_, progPacketRepeatCount_
              , progWritableNotifiable_)
}

void RMTTrackDevice::send_last_railcom_response_buffer(bool allocate_another)
{
  if (railcomEnabled_ && railComFeedback_)
  {
    // send the feedback to the hub
    railComHub_->send(railComFeedback_);
    railComFeedback_ = nullptr;

    // if we need another feedback packet allocate it now with the same key
    if (allocate_another)
    {
      get_next_railcom_response_buffer(opsPacketFeedbackKey_);
    }
  }
}

void RMTTrackDevice::get_next_railcom_response_buffer(uintptr_t key)
{
  if (railcomEnabled_)
  {
    railComFeedback_ = railComHub_->alloc();
    railComFeedback_->data()->reset(key);
    opsPacketFeedbackKey_ = key;
  }
}

void RMTTrackDevice::read_railcom_response()
{
  // Ensure the signal pin is LOW before starting RailCom detection
  ESP_ERROR_CHECK(gpio_set_level(opsSignalPin_, 0));
  ets_delay_us(RAILCOM_PACKET_END_DELAY_USEC);
  // Enable the BRAKE pin on the h-bridge to force it into coast mode
  ESP_ERROR_CHECK(gpio_set_level(railComBrakeEnablePin_, 1));
  // Disable the h-bridge output after a short delay to allow the h-bridge to
  // sink remaining current.
  ets_delay_us(RAILCOM_BRAKE_ENABLE_DELAY_USEC);
  ESP_ERROR_CHECK(gpio_set_level(opsOutputEnablePin_, 0));
  // flush the uart buffers
  ESP_ERROR_CHECK(uart_flush(railComUartPort_));
  // enable receive interrupts on the uart
  uart_enable_rx_intr(railComUartPort_);
  // enable the RailCom receiver circuitry
  ESP_ERROR_CHECK(gpio_set_level(railComEnablePin_, 1));

  // allow the RailCom detector to stabilize before we try and read data
  ets_delay_us(RAILCOM_ENABLE_TRANSIENT_DELAY_USEC);

  uint8_t buf[8];
  uint8_t buf_idx = 0;
  int buf_used = uart_read_bytes(railComUartPort_, buf, 2, RAILCOM_MAX_READ_DELAY_CH_1);
  while (--buf_used >= 0)
  {
    railComFeedback_->data()->add_ch1_data(buf[buf_idx++]);
  }
  // flush the uart buffers
  ESP_ERROR_CHECK(uart_flush(railComUartPort_));
  buf_used = uart_read_bytes(railComUartPort_, buf, 6, RAILCOM_MAX_READ_DELAY_CH_2);
  buf_idx = 0;
  while (--buf_used >= 0)
  {
    railComFeedback_->data()->add_ch2_data(buf[buf_idx++]);
  }

  ESP_ERROR_CHECK(gpio_set_level(railComEnablePin_, 0));
  if (gpio_get_level(railComShortPin_))
  {
    LOG_ERROR("RailCom short detected!!!");
  }
  ESP_ERROR_CHECK(gpio_set_level(opsOutputEnablePin_, 1));
  ets_delay_us(RAILCOM_BRAKE_DISABLE_DELAY_USEC);
  ESP_ERROR_CHECK(gpio_set_level(railComBrakeEnablePin_, 0));
}

void RMTTrackDevice::read_railcom_response_noop()
{
}