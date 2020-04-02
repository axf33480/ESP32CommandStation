/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2020 Mike Dunston

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
// Helper macro for encoding a dcc::Packet into an RMT payload
//
// When a packet has remaining repeats the packet data is not re-encoded.
// This will allocate a new RailCom feedback when a new packet is encoded based
// on the alloc_railcom function passed in.
///////////////////////////////////////////////////////////////////////////////
#define ENCODE_PACKET(queue, lock, preambleCount, encodedPacket             \
                    , encodedLength, repeatCount, notifiable)               \
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
  railcomDriver_->set_feedback_key(packet.feedback_key);

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
                             , const char *name
                             , const rmt_channel_t channel
                             , const uint8_t preambleBitCount
                             , size_t packet_queue_len
                             , gpio_num_t pin
                             , RailcomDriver *railcomDriver
                             )
                             : name_(name)
                             , channel_(channel)
                             , preambleBitCount_(preambleBitCount)
                             , railcomDriver_(railcomDriver)
                             , packetQueue_(DeviceBuffer<dcc::Packet>::create(packet_queue_len))
{
  uint16_t maxBitCount = preambleBitCount                 // preamble bits
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
          , name_, channel_, pin, memoryBlocks, maxBitCount
          , (memoryBlocks * RMT_MEM_ITEM_NUM), RMT_CLOCK_DIVIDER
          , DCC_ZERO_BIT_PULSE_USEC, DCC_ONE_BIT_PULSE_USEC);
  rmt_config_t config =
  {
    .rmt_mode = RMT_MODE_TX,
    .channel = channel_,
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
  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(channel_, 0, RMT_ISR_FLAGS));
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

  if (active_)
  {
    AtomicHolder l(&packetQueueLock_);
    dcc::Packet* writePacket;
    if (packetQueue_->space() &&
        packetQueue_->data_write_pointer(&writePacket))
    {
      memcpy(writePacket, data, size);
      packetQueue_->advance(1);
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
  if (IOC_TYPE(cmd) == CAN_IOC_MAGIC && IOC_SIZE(cmd) == NOTIFIABLE_TYPE &&
      cmd == CAN_IOC_WRITE_ACTIVE)
  {
    Notifiable* n = reinterpret_cast<Notifiable*>(va_arg(args, uintptr_t));
    HASSERT(n);
    {
      AtomicHolder l(&packetQueueLock_);
      if (!packetQueue_->space())
      {
        // stash the notifiable so we can call it later when there is space
        std::swap(n, notifiable_);
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
void RMTTrackDevice::rmt_transmit_complete()
{
  // If the OPS signal is active generate the next packet for TX
  if (active_)
  {
    uint32_t encodedLength = 0;
    rmt_item32_t encodedPacket[MAX_DCC_PACKET_BITS];
    ENCODE_PACKET(packetQueue_, packetQueueLock_, preambleBitCount_
                , encodedPacket, encodedLength, packetRepeatCount_
                , notifiable_)
    railcomDriver_->start_cutout();
    // send the packet to the RMT, note not using memcpy for the packet as this
    // directly accesses hardware registers.
    RMT_SET_FIFO(RMT_DATA_MODE_MEM);
    for(uint32_t index = 0; index < encodedLength; index++)
    {
      RMT_SET_DATA(channel_, index) = encodedPacket[index].val;
    }
    // RMT marker for "end of data"
    RMT_SET_DATA(channel_, encodedLength) = 0;
    // start transmit
    RMT_CONFIG(channel_).mem_rd_rst = 1;
    RMT_CONFIG(channel_).mem_owner = RMT_MEM_OWNER_TX;
    RMT_CONFIG(channel_).tx_start = 1;
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
    AtomicHolder l(&packetQueueLock_);
    dcc::Packet* writePacket;
    if (packetQueue_->space() &&
        packetQueue_->data_write_pointer(&writePacket))
    {
      memcpy(writePacket, b->data(), b->size());
      packetQueue_->advance(1);
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
void RMTTrackDevice::enable()
{
  AtomicHolder h(this);
  if (!active_)
  {
    active_ = true;
    ets_printf("[RMT] Starting RMT for %s\n", name_);
    // send one bit to kickstart the signal, remaining data will come from the
    // packet queue. We intentionally do not wait for the RMT TX complete here.
    rmt_write_items(channel_, &DCC_RMT_ONE_BIT, 1, false);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Disables the OPS track output if enabled.
///////////////////////////////////////////////////////////////////////////////
void RMTTrackDevice::disable()
{
  AtomicHolder h(this);
  if (active_)
  {
    active_ = false;
    AtomicHolder l(&packetQueueLock_);
    packetQueue_->flush();
    ets_printf("[RMT] Shutting down RMT for %s\n", name_);
  }
}