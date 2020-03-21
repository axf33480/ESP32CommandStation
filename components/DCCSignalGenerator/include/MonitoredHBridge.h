/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018-2019 Mike Dunston

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

#ifndef MONITORED_H_BRIDGE_
#define MONITORED_H_BRIDGE_

#include "TrackOutputDescriptor.h"

#include <executor/StateFlow.hxx>
#include <openlcb/EventHandlerTemplates.hxx>
#include <openlcb/Node.hxx>
#include <os/OS.hxx>
#include <utils/ConfigUpdateListener.hxx>
#include <utils/format_utils.hxx>
#include <utils/logging.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

class MonitoredHBridge : public StateFlowBase, public DefaultConfigUpdateListener {
public:
  MonitoredHBridge(openlcb::Node *
                 , Service *
                 , const adc1_channel_t
                 , const gpio_num_t
                 , const gpio_num_t
                 , const uint32_t
                 , const uint32_t
                 , const std::string &
                 , const std::string &
                 , const esp32cs::TrackOutputConfig &);

  MonitoredHBridge(openlcb::Node *
                 , Service *
                 , const adc1_channel_t
                 , const gpio_num_t
                 , const uint32_t
                 , const uint32_t
                 , const std::string &
                 , const std::string &
                 , const esp32cs::TrackOutputConfig &);

  MonitoredHBridge(openlcb::Node *
                 , Service *
                 , const adc1_channel_t
                 , const gpio_num_t
                 , const uint32_t
                 , const std::string &
                 , const std::string &
                 , const esp32cs::TrackOutputConfig &);

  enum STATE
  {
    STATE_OVERCURRENT       = 1
  , STATE_SHUTDOWN          = 2
  , STATE_THERMAL_SHUTDOWN  = 4
  , STATE_ON                = 8
  , STATE_OFF               = 16
  };

  std::string getName()
  {
    return name_;
  }

  uint32_t getMaxMilliAmps()
  {
    return maxMilliAmps_;
  }

  uint32_t getLastReading()
  {
    return lastReading_;
  }

  bool isProgrammingTrack()
  {
    return isProgTrack_;
  }

  bool isEnabled()
  {
    return state_ != STATE_OFF;
  }

  float getUsage()
  {
    if (state_ != STATE_OFF)
    {
      return ((lastReading_ * maxMilliAmps_) / 4096.0f);
    }
    return 0.0f;
  }

  std::string getState();

  std::string getStateAsJson();

  std::string getStatusData();

  std::string get_state_for_dccpp();

  void disable();

  void enable();

  UpdateAction apply_configuration(int fd, bool initial_load, BarrierNotifiable *done) override
  {
    AutoNotify n(done);
    UpdateAction res = UPDATED;
    openlcb::EventId short_detected = cfg_.event_short().read(fd);
    openlcb::EventId short_cleared = cfg_.event_short_cleared().read(fd);
    openlcb::EventId shutdown = cfg_.event_shutdown().read(fd);
    openlcb::EventId shutdown_cleared = cfg_.event_shutdown_cleared().read(fd);
    openlcb::EventId thermal_shutdown = cfg_.event_thermal_shutdown().read(fd);
    openlcb::EventId thermal_shutdown_cleared = cfg_.event_thermal_shutdown_cleared().read(fd);

    if (initial_load)
    {
      start_flow(STATE(init));
      res = REINIT_NEEDED;
    }

    // reinitialize the event producer
    auto saved_node = shortBit_.node();
    if (short_detected != shortBit_.event_on() ||
        short_cleared != shortBit_.event_off())
    {
      shortBit_.openlcb::MemoryBit<uint8_t>::~MemoryBit();
      new (&shortBit_)openlcb::MemoryBit<uint8_t>(saved_node, short_detected, short_cleared, &state_, STATE_OVERCURRENT);
      shortProducer_.openlcb::BitEventProducer::~BitEventProducer();
      new (&shortProducer_)openlcb::BitEventProducer(&shortBit_);
      res = REINIT_NEEDED;
    }

    if (shutdown != shortBit_.event_on() ||
        shutdown_cleared != shortBit_.event_off())
    {
      saved_node = shutdownBit_.node();
      shutdownBit_.openlcb::MemoryBit<uint8_t>::~MemoryBit();
      new (&shutdownBit_)openlcb::MemoryBit<uint8_t>(saved_node, shutdown, shutdown_cleared, &state_, STATE_SHUTDOWN);
      shutdownProducer_.openlcb::BitEventProducer::~BitEventProducer();
      new (&shutdownProducer_)openlcb::BitEventProducer(&shutdownBit_);
      res = REINIT_NEEDED;
    }

    if (thermal_shutdown != shortBit_.event_on() ||
        thermal_shutdown_cleared != shortBit_.event_off())
    {
      saved_node = thermalBit_.node();
      thermalBit_.openlcb::MemoryBit<uint8_t>::~MemoryBit();
      new (&thermalBit_)openlcb::MemoryBit<uint8_t>(saved_node, thermal_shutdown, thermal_shutdown_cleared, &state_, STATE_THERMAL_SHUTDOWN);
      thermalProducer_.openlcb::BitEventProducer::~BitEventProducer();
      new (&thermalProducer_)openlcb::BitEventProducer(&thermalBit_);
      res = REINIT_NEEDED;
    }
    return res;
  }

  void factory_reset(int fd) override
  {
    LOG(INFO
      , "[LCC] MonitoredHBridge(%s) factory_reset(%d) invoked, defaulting "
        "configuration", name_.c_str(), fd);
    cfg_.description().write(fd, StringPrintf("%s Track", name_.c_str()));
  }

private:
  const adc1_channel_t channel_;
  const gpio_num_t enablePin_;
  const gpio_num_t thermalWarningPin_;
  const uint32_t maxMilliAmps_;
  const std::string name_;
  const std::string bridgeType_;
  const bool isProgTrack_;
  uint32_t overCurrentLimit_{0};
  uint32_t shutdownLimit_{0};
  uint32_t warnLimit_{0};
  uint32_t progAckLimit_{0};
  const esp32cs::TrackOutputConfig cfg_;
  const uint8_t targetLED_;
  const uint8_t adcSampleCount_{32};
  const uint64_t checkInterval_{MSEC_TO_NSEC(50)};
  const uint8_t overCurrentRetryCount_{3};
  const uint64_t overCurrentRetryInterval_{MSEC_TO_NSEC(25)};
  const uint64_t currentReportInterval_{SEC_TO_USEC(30)};
  const uint8_t thermalWarningRetryCount_{3};
  const uint64_t thermalWarningRetryInterval_{MSEC_TO_NSEC(25)};
  StateFlowTimer timer_{this};
  openlcb::MemoryBit<uint8_t> shortBit_;
  openlcb::MemoryBit<uint8_t> shutdownBit_;
  openlcb::MemoryBit<uint8_t> thermalBit_;
  openlcb::BitEventProducer shortProducer_;
  openlcb::BitEventProducer shutdownProducer_;
  openlcb::BitEventProducer thermalProducer_;
  openlcb::WriteHelper helper_;
  BarrierNotifiable n_;
  uint64_t lastReport_{0};
  uint32_t lastReading_{0};
  uint8_t state_{STATE_OFF};
  OSMutex requestedStateLock_;
  uint8_t requestedState_{STATE_OFF};
  uint8_t lastRequestedState_{STATE_OFF};
  uint8_t overCurrentCheckCount_{0};
  uint8_t thermalWarningCheckCount_{0};

  STATE_FLOW_STATE(init);
  STATE_FLOW_STATE(check);

  Action sleep_and_check_state()
  {
    return sleep_and_call(&timer_, checkInterval_, STATE(check));
  }

  Action sleep_and_check_overcurrent()
  {
    return sleep_and_call(&timer_, overCurrentRetryInterval_, STATE(check));
  }

  Action sleep_and_check_thermal_warning()
  {
    return sleep_and_call(&timer_, thermalWarningRetryInterval_, STATE(check));
  }
};

#endif // MONITORED_H_BRIDGE_