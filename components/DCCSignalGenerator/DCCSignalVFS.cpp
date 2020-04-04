/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020 Mike Dunston

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
#include "EStopHandler.h"
#include "HBridgeThermalMonitor.h"
#include "TrackPowerBitInterface.h"

#include <dcc/ProgrammingTrackBackend.hxx>
#include <esp_vfs.h>
#include <freertos_drivers/arduino/DummyGPIO.hxx>
#include <freertos_drivers/esp32/Esp32Gpio.hxx>
#include <map>
#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/RefreshLoop.hxx>
#include <StatusDisplay.h>
#include <StatusLED.h>
#include <utils/GpioInitializer.hxx>
#include <utils/logging.h>

namespace esp32cs
{
/// RMT channel to use for the OPS track output.
static constexpr rmt_channel_t OPS_RMT_CHANNEL = RMT_CHANNEL_0;

/// RMT channel to use for the PROG track output.
static constexpr rmt_channel_t PROG_RMT_CHANNEL = RMT_CHANNEL_3;

/// File descriptor to use/return for the OPS track output.
static const int OPS_FD_NUM = 0;

/// File descriptor to use/return for the PROG track output.
static const int PROG_FD_NUM = 1;

/// OPS Track signal pin.
GPIO_PIN(OPS_SIGNAL, GpioOutputSafeLow, CONFIG_OPS_SIGNAL_PIN);

/// OPS Track h-bridge enable pin.
GPIO_PIN(OPS_ENABLE, GpioOutputSafeLow, CONFIG_OPS_ENABLE_PIN);

#ifdef CONFIG_OPS_THERMAL_PIN
/// OPS Track h-bridge thermal alert pin, active LOW.
GPIO_PIN(OPS_THERMAL, GpioInputPU, CONFIG_OPS_THERMAL_PIN);
#else
/// OPS Track h-bridge thermal alert pin, not connected to physical pin.
typedef DummyPinWithReadHigh OPS_THERMAL_Pin;
#endif // CONFIG_OPS_THERMAL_PIN

#if defined(CONFIG_OPS_RAILCOM)
/// OPS Track h-bridge brake pin, active HIGH.
GPIO_PIN(OPS_RAILCOM_BRAKE, GpioInputPU, CONFIG_OPS_RAILCOM_BRAKE_PIN);

/// RailCom detector enable pin, active HIGH.
GPIO_PIN(OPS_RAILCOM_ENABLE, GpioInputPD, CONFIG_OPS_RAILCOM_ENABLE_PIN);

/// RailCom detector short pin, active LOW.
GPIO_PIN(OPS_RAILCOM_SHORT, GpioInputPU, CONFIG_OPS_RAILCOM_SHORT_PIN);

/// RailCom detector UART.
static constexpr uart_port_t RAILCOM_UART_NUM =
  (uart_port_t)CONFIG_OPS_RAILCOM_UART;

/// RailCom driver instance for the OPS track.
NoRailcomDriver opsRailComDriver;
#else
/// OPS Track h-bridge brake pin, active HIGH.
typedef DummyPinWithReadHigh OPS_RAILCOM_BRAKE_Pin;

/// RailCom detector enable pin, active HIGH.
typedef DummyPinWithRead OPS_RAILCOM_ENABLE_Pin;

/// RailCom detector short pin, active LOW.
typedef DummyPinWithReadHigh OPS_RAILCOM_SHORT_Pin;

/// RailCom detector UART, unused.
static constexpr uart_port_t RAILCOM_UART_NUM = UART_NUM_1;

/// RailCom driver instance for the OPS track.
NoRailcomDriver opsRailComDriver;

#endif // CONFIG_OPS_RAILCOM

/// RailCom driver instance for the PROG track, unused.
NoRailcomDriver progRailComDriver;

/// PROG Track signal pin.
GPIO_PIN(PROG_SIGNAL, GpioOutputSafeLow, CONFIG_PROG_SIGNAL_PIN);

/// PROG Track h-bridge enable pin.
GPIO_PIN(PROG_ENABLE, GpioOutputSafeLow, CONFIG_PROG_ENABLE_PIN);

/// Initializer for all GPIO pins.
typedef GpioInitializer<
  OPS_SIGNAL_Pin, OPS_ENABLE_Pin, OPS_THERMAL_Pin
, PROG_SIGNAL_Pin, PROG_ENABLE_Pin
, OPS_RAILCOM_BRAKE_Pin, OPS_RAILCOM_ENABLE_Pin, OPS_RAILCOM_SHORT_Pin
> DCCGpioInitializer;

uninitialized<RMTTrackDevice> ops_track;
uninitialized<HBridgeShortDetector> ops_track_mon;
uninitialized<HBridgeThermalMonitor> ops_thermal_mon;
uninitialized<RMTTrackDevice> prog_track;
uninitialized<HBridgeShortDetector> prog_track_mon;
uninitialized<TrackPowerBit> track_power;
uninitialized<openlcb::BitEventConsumer> power_event;
uninitialized<EStopHandler> estop_handler;
uninitialized<ProgrammingTrackBackend> prog_track_backend;
openlcb::RefreshLoop *dcc_poller;

/// Updates the status display with the current state of the track outputs.
static void update_status_display()
{
  auto status = Singleton<StatusDisplay>::instance();
  status->track_power("%s:%s %s:%s", CONFIG_OPS_TRACK_NAME
                    , OPS_ENABLE_Pin::get() ? "On" : "Off"
                    , CONFIG_PROG_TRACK_NAME
                    , PROG_ENABLE_Pin::get() ? "On" : "Off");
}

/// Triggers an estop event to be sent
void initiate_estop()
{
  // TODO: add event publish
  estop_handler->set_state(true);
}

/// Enables the OPS track output
void enable_ops_track_output()
{
  LOG(INFO, "[Track] Enabling OPS track output");
  ops_track->enable();
  OPS_ENABLE_Pin::set_on();
  Singleton<StatusLED>::instance()->setStatusLED(
        StatusLED::LED::OPS_TRACK, StatusLED::COLOR::GREEN);
  update_status_display();
}

/// Enables the OPS track output
void disable_ops_track_output()
{
  LOG(INFO, "[Track] Disabling OPS track output");
  ops_track->disable();
  OPS_ENABLE_Pin::set_off();
  Singleton<StatusLED>::instance()->setStatusLED(
        StatusLED::LED::OPS_TRACK, StatusLED::COLOR::OFF);
  update_status_display();
}

/// Enables the PROG track output
static void enable_prog_track_output()
{
  LOG(INFO, "[Track] Enabling PROG track output");
  prog_track->enable();
  PROG_ENABLE_Pin::set_on();
  Singleton<StatusLED>::instance()->setStatusLED(
        StatusLED::LED::PROG_TRACK, StatusLED::COLOR::GREEN);
  update_status_display();
}

/// Disables the PROG track outputs
static void disable_prog_track_output()
{
  LOG(INFO, "[Track] Disabling PROG track output");
  PROG_ENABLE_Pin::set_off();
  prog_track->disable();
  Singleton<StatusLED>::instance()->setStatusLED(
        StatusLED::LED::PROG_TRACK, StatusLED::COLOR::OFF);
  update_status_display();
}

/// Disables all track outputs
void disable_track_outputs()
{
  disable_ops_track_output();
  disable_prog_track_output();
}

/// ESP32 VFS ::write() impl for the RMTTrackDevice.
/// @param fd is the file descriptor being written to.
/// @param data is the data to write.
/// @param size is the size of data.
/// @returns number of bytes written.
static ssize_t dcc_vfs_write(int fd, const void *data, size_t size)
{
  if (fd == OPS_FD_NUM)
  {
    return ops_track->write(fd, data, size);
  }
  return prog_track->write(fd, data, size);
}

/// ESP32 VFS ::open() impl for the RMTTrackDevice
/// @param path is the file location to be opened.
/// @param flags is not used.
/// @param mode is not used.
/// @returns file descriptor for the opened file location.
static int dcc_vfs_open(const char *path, int flags, int mode)
{
  int fd = OPS_FD_NUM;
  if (!strcasecmp(path + 1, CONFIG_OPS_TRACK_NAME))
  {
    fd = OPS_FD_NUM;
  }
  else if (!strcasecmp(path + 1, CONFIG_PROG_TRACK_NAME))
  {
    fd = PROG_FD_NUM;
  }
  LOG(INFO, "[Track] Connecting track interface (track:%s, fd:%d)", path + 1, fd);
  return fd;
}

/// ESP32 VFS ::close() impl for the RMTTrackDevice.
/// @param fd is the file descriptor to close.
/// @returns the status of the close() operation, only returns zero.
static int dcc_vfs_close(int fd)
{
  LOG(INFO, "[Track] Disconnecting track interface (track:%s, fd:%d)"
    , (fd == OPS_FD_NUM) ? CONFIG_OPS_TRACK_NAME : CONFIG_PROG_TRACK_NAME, fd);
  return 0;
}

/// ESP32 VFS ::ioctl() impl for the RMTTrackDevice.
/// @param fd is the file descriptor to operate on.
/// @param cmd is the ioctl command to execute.
/// @param args are the arguments to ioctl.
/// @returns the result of the ioctl command, zero on success, non-zero will
/// set errno.
static int dcc_vfs_ioctl(int fd, int cmd, va_list args)
{
  if (fd == OPS_FD_NUM)
  {
    return ops_track->ioctl(fd, cmd, args);
  }
  return prog_track->ioctl(fd, cmd, args);
}

/// RMT transmit complete ISR callback.
///
/// @param channel is the RMT channel that has completed transmission.
/// @param ctx is unused.
///
/// This is called automatically by the RMT peripheral when it reaches the end
/// of TX data.
static void rmt_tx_complete_isr_callback(rmt_channel_t channel, void *ctx)
{
  // Check if the channel 0 TX END has been triggered
  // if so the OPS TX has completed and we need to clear the event
  if (channel == OPS_RMT_CHANNEL)
  {
    ops_track->rmt_transmit_complete();
  }

  // Check if the channel 1 TX END has been triggered
  // if so the PROG TX has completed and we need to clear the event
  if (channel == PROG_RMT_CHANNEL)
  {
    prog_track->rmt_transmit_complete();
  }
}

/// Initializes the ESP32 VFS adapter for the DCC track interface and the short
/// detection devices.
/// @param node is the OpenLCB node to bind to.
/// @param service is the OpenLCB @ref Service to use for recurring tasks.
/// @param ops_cfg is the CDI element for the OPS track output.
/// @param prog_cfg is the CDI element for the PROG track output.
void init_dcc_vfs(openlcb::Node *node, Service *service
                , const esp32cs::TrackOutputConfig &ops_cfg
                , const esp32cs::TrackOutputConfig &prog_cfg)
{
  // register the VFS handler as the LocalTrackIf uses this to route DCC
  // packets to the track.
  esp_vfs_t vfs;
  memset(&vfs, 0, sizeof(vfs));
  vfs.flags = ESP_VFS_FLAG_DEFAULT;
  vfs.ioctl = &esp32cs::dcc_vfs_ioctl;
  vfs.open = &esp32cs::dcc_vfs_open;
  vfs.close = &esp32cs::dcc_vfs_close;
  vfs.write = &esp32cs::dcc_vfs_write;

  LOG(INFO, "[Track] Registering /dev/track VFS interface");
  ESP_ERROR_CHECK(esp_vfs_register("/dev/track", &vfs, nullptr));

  DCCGpioInitializer::hw_init();
  OPS_ENABLE_Pin::set_pulldown_on();
  PROG_ENABLE_Pin::set_pulldown_on();

  ops_thermal_mon.emplace(node, ops_cfg, OPS_THERMAL_Pin::instance());

  ops_track_mon.emplace(node, (adc1_channel_t)CONFIG_OPS_ADC
                      , OPS_ENABLE_Pin::instance()
                      , CONFIG_OPS_HBRIDGE_LIMIT_MILLIAMPS
                      , CONFIG_OPS_HBRIDGE_MAX_MILLIAMPS
                      , CONFIG_OPS_TRACK_NAME
                      , CONFIG_OPS_HBRIDGE_TYPE_NAME
                      , ops_cfg);

  prog_track_mon.emplace(node, (adc1_channel_t)CONFIG_PROG_ADC
                       , PROG_ENABLE_Pin::instance()
                       , CONFIG_PROG_HBRIDGE_MAX_MILLIAMPS
                       , CONFIG_PROG_TRACK_NAME
                       , CONFIG_PROG_HBRIDGE_TYPE_NAME
                       , prog_cfg);

  // This can not use the emplace option as it does not appear to map correctly
  dcc_poller = new openlcb::RefreshLoop(node,
    { ops_thermal_mon->polling()
    , ops_track_mon.operator->()
    , prog_track_mon.operator->()
  });

  ops_track.emplace(node, CONFIG_OPS_TRACK_NAME, OPS_RMT_CHANNEL
                  , CONFIG_OPS_PREAMBLE_BITS, CONFIG_OPS_PACKET_QUEUE_SIZE
                  , OPS_SIGNAL_Pin::pin(), &opsRailComDriver);

  prog_track.emplace(node, CONFIG_PROG_TRACK_NAME, PROG_RMT_CHANNEL
                   , CONFIG_PROG_PREAMBLE_BITS, CONFIG_PROG_PACKET_QUEUE_SIZE
                   , PROG_SIGNAL_Pin::pin(), &progRailComDriver);

  // hook into the RMT ISR to have callbacks when TX completes
  rmt_register_tx_end_callback(rmt_tx_complete_isr_callback, nullptr);

  LOG(INFO
    , "[Track] Registering LCC EventConsumer for Track Power (On:%s, Off:%s)"
    , uint64_to_string_hex(openlcb::Defs::CLEAR_EMERGENCY_OFF_EVENT).c_str()
    , uint64_to_string_hex(openlcb::Defs::EMERGENCY_OFF_EVENT).c_str());
  track_power.emplace(node, OPS_ENABLE_Pin::instance());
  power_event.emplace(track_power.operator->());

  // Initialize the e-stop event handler
  estop_handler.emplace(node);

  // Initialize the Programming Track backend handler
  prog_track_backend.emplace(service, &enable_prog_track_output
                           , &disable_prog_track_output);

  update_status_display();

#if defined(CONFIG_OPS_ENERGIZE_ON_STARTUP)
  power_event->set_state(true);
#endif
}

/// @return string containing a two element json array of the track monitors.
std::string get_track_state_json()
{
  return StringPrintf("[%s,%s]", ops_track_mon->getStateAsJson().c_str()
                    , prog_track_mon->getStateAsJson().c_str());
}

/// @return DCC++ status data from the OPS track only.
std::string get_track_state_for_dccpp()
{
  return ops_track_mon->get_state_for_dccpp();
}

} // namespace esp32cs