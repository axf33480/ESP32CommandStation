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

#pragma once

#include "ESP32CommandStation.h"
#include <executor/StateFlow.hxx>
#include <openlcb/ConfigRepresentation.hxx>
#include <openlcb/SimpleStack.hxx>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include "stateflows/StatusLED.h"

/// Track output configuration
CDI_GROUP(TrackOutputConfig);
CDI_GROUP_ENTRY(description,
                openlcb::StringConfigEntry<15>,
                Name("Description"),
                Description("Track output description."));
CDI_GROUP_ENTRY(event_short,
                openlcb::EventConfigEntry,
                Name("Short Detected"),
                Description("This event will be produced when a short has been detected on the track output."));
CDI_GROUP_ENTRY(event_short_cleared,
                openlcb::EventConfigEntry,
                Name("Short Cleared"),
                Description("This event will be produced when a short has been cleared on the track output."));
CDI_GROUP_ENTRY(event_shutdown,
                openlcb::EventConfigEntry,
                Name("H-Bridge Shutdown"),
                Description("This event will be produced when the track output power has exceeded the safety threshold of the H-Bridge."));
CDI_GROUP_ENTRY(event_shutdown_cleared,
                openlcb::EventConfigEntry,
                Name("H-Bridge Shutdown Cleared"),
                Description("This event will be produced when the track output power has returned to safe levels."));
CDI_GROUP_ENTRY(event_thermal_shutdown,
                openlcb::EventConfigEntry,
                Name("H-Bridge Thermal Shutdown"),
                Description("This event will be produced when the H-Bridge raises a thermal warning alert."));
CDI_GROUP_ENTRY(event_thermal_shutdown_cleared,
                openlcb::EventConfigEntry,
                Name("H-Bridge Thermal Shutdown Cleared"),
                Description("This event will be produced when the H-Bridge clears the thermal warning alert."));
CDI_GROUP_END();

#ifndef ADC_CURRENT_ATTENUATION
#define ADC_CURRENT_ATTENUATION ADC_ATTEN_DB_11
#endif

class MonitoredHBridge : public StateFlowBase, public DefaultConfigUpdateListener {
public:
  MonitoredHBridge(SimpleCanStack *stack
                 , const adc1_channel_t senseChannel
                 , const gpio_num_t enablePin
                 , const gpio_num_t thermalWarningPin
                 , const uint32_t limitMilliAmps
                 , const uint32_t maxMilliAmps
                 , const string &name
                 , const string &bridgeType
                 , const TrackOutputConfig &cfg
                 , const bool programmingTrack=false) :
    StateFlowBase(stack->service())
    , DefaultConfigUpdateListener()
    , channel_(senseChannel)
    , enablePin_(enablePin)
    , thermalWarningPin_(thermalWarningPin)
    , maxMilliAmps_(maxMilliAmps)
    , name_(name)
    , bridgeType_(bridgeType)
    , isProgTrack_(programmingTrack)
    , overCurrentLimit_(((((limitMilliAmps << 3) + limitMilliAmps) / 10) << 12) / maxMilliAmps_) // ~90% max value
    , shutdownLimit_(4090)
    , cfg_(cfg)
    , targetLED_(isProgTrack_ ? StatusLED::LED::PROG_TRACK : StatusLED::LED::OPS_TRACK)
    , shortBit_(stack->node(), 0, 0, &state_, STATE_OVERCURRENT)
    , shutdownBit_(stack->node(), 0, 0, &state_, STATE_SHUTDOWN)
    , thermalBit_(stack->node(), 0, 0, &state_, STATE_THERMAL_SHUTDOWN)
    , shortProducer_(&shortBit_)
    , shutdownProducer_(&shortBit_)
    , thermalProducer_(&shortBit_)
  {
    if (isProgTrack_)
    {
      // programming track needs to be current limited to ~250mA
      overCurrentLimit_ = (250 << 12) / maxMilliAmps_;
      shutdownLimit_ = overCurrentLimit_ << 1;
    }
    // set warning limit to ~75% of overcurrent limit
    warnLimit_ = ((overCurrentLimit_ << 1) + overCurrentLimit_) >> 2;
  }

  enum STATE
  {
    STATE_OVERCURRENT = 1
  , STATE_SHUTDOWN = 2
  , STATE_THERMAL_SHUTDOWN = 4
  , STATE_ON = 8
  , STATE_OFF = 16
  };

  string getName()
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

  string getState()
  {
    switch (state_)
    {
      case STATE_ON:
        return JSON_VALUE_NORMAL;
      case STATE_OVERCURRENT:
        return JSON_VALUE_FAULT;
      case STATE_OFF:
      default:
        return JSON_VALUE_OFF;
    }
    return JSON_VALUE_ERROR;
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

  string getStateAsJson()
  {
    return StringPrintf("{"
                          "\"name\":\"%s\","
                          "\"state\":\"%s\","
                          "\"usage\":%.2f,"
                          "\"prog\":\"%s\""
                        "}"
                        , name_.c_str()
                        , getState().c_str()
                        , getUsage()
                        , isProgrammingTrack() ? JSON_VALUE_TRUE : JSON_VALUE_FALSE
                       );
  }

  string getInfoScreenData()
  {
    if (state_ == STATE_ON)
    {
      return StringPrintf("%s:On (%2.2f A)", name_.c_str(), getUsage());
    }
    else if (state_ == STATE_OVERCURRENT)
    {
      return StringPrintf("%s:F (%2.2f A)", name_.c_str(), getUsage());
    }
    return StringPrintf("%s:Off", name_.c_str());
  }

  void broadcastStatus()
  {
    if (state_ == STATE_ON)
    {
      wifiInterface.broadcast(StringPrintf("<p1 %s><a %s %d>", name_.c_str(), name_.c_str(), getLastReading()));
    }
    else if (state_ == STATE_OVERCURRENT)
    {
      wifiInterface.broadcast(StringPrintf("<p2 %s>", name_.c_str()));
    }
    else
    {
      wifiInterface.broadcast(StringPrintf("<p0 %s>", name_.c_str()));
    }
  }

  void disable()
  {
    if(state_ != STATE_OFF)
    {
      state_ = STATE_OFF;
      yield_and_call(STATE(check));
      LOG(INFO, "[%s] Disabling h-bridge", name_.c_str());
    }
    statusLED->setStatusLED(targetLED_, StatusLED::COLOR::OFF);
  }

  void enable()
  {
    state_ = STATE_ON;
    yield_and_call(STATE(check));
    LOG(INFO, "[%s] Enabling h-bridge", name_.c_str());
    statusLED->setStatusLED(targetLED_, StatusLED::COLOR::GREEN);
#if LOCONET_ENABLED
    if (!isProgTrack_)
    {
      locoNet.reportPower(true);
    }
#endif
  }

  UpdateAction apply_configuration(int fd, bool initial_load, BarrierNotifiable *done) override
  {
      AutoNotify n(done);
      EventId short_detected = cfg_.event_short().read(fd);
      EventId short_cleared = cfg_.event_short_cleared().read(fd);

      EventId shutdown = cfg_.event_short().read(fd);
      EventId shutdown_cleared = cfg_.event_short_cleared().read(fd);

      EventId thermal_shutdown = cfg_.event_short().read(fd);
      EventId thermal_shutdown_cleared = cfg_.event_short_cleared().read(fd);

      // reinitialize the event producer
      auto saved_node = shortBit_.node();
      shortBit_.MemoryBit<uint8_t>::~MemoryBit();
      new (&shortBit_)MemoryBit<uint8_t>(saved_node, short_detected, short_cleared, &state_, STATE_OVERCURRENT);
      shortProducer_.BitEventProducer::~BitEventProducer();
      new (&shortProducer_)BitEventProducer(&shortBit_);

      shutdownBit_.MemoryBit<uint8_t>::~MemoryBit();
      new (&shutdownBit_)MemoryBit<uint8_t>(saved_node, shutdown, shutdown_cleared, &state_, STATE_SHUTDOWN);
      shutdownProducer_.BitEventProducer::~BitEventProducer();
      new (&shutdownProducer_)BitEventProducer(&shutdownBit_);

      thermalBit_.MemoryBit<uint8_t>::~MemoryBit();
      new (&thermalBit_)MemoryBit<uint8_t>(saved_node, thermal_shutdown, thermal_shutdown_cleared, &state_, STATE_THERMAL_SHUTDOWN);
      thermalProducer_.BitEventProducer::~BitEventProducer();
      new (&thermalProducer_)BitEventProducer(&thermalBit_);

      start_flow(STATE(init));

      return REINIT_NEEDED; // Causes events identify.
  }

  void factory_reset(int fd) override
  {
      LOG(VERBOSE, "Factory Reset Helper invoked");
      cfg_.description().write(fd, StringPrintf("%s Track Output", name_.c_str()).c_str());
  }

private:
  const adc1_channel_t channel_;
  const gpio_num_t enablePin_;
  const gpio_num_t thermalWarningPin_;
  const uint32_t maxMilliAmps_;
  const string name_;
  const string bridgeType_;
  const bool isProgTrack_;
  uint32_t overCurrentLimit_{0};
  uint32_t shutdownLimit_{0};
  uint32_t warnLimit_{0};
  const TrackOutputConfig cfg_;
  const StatusLED::LED targetLED_;
  const uint8_t adcSampleCount_{64};
  const uint64_t checkInterval_{MSEC_TO_NSEC(25)};
  const uint8_t overCurrentRetryCount_{3};
  const uint64_t overCurrentRetryInterval_{MSEC_TO_NSEC(250)};
  const uint64_t currentReportInterval_{SEC_TO_USEC(30)};
  StateFlowTimer timer_{this};
  MemoryBit<uint8_t> shortBit_;
  MemoryBit<uint8_t> shutdownBit_;
  MemoryBit<uint8_t> thermalBit_;
  BitEventProducer shortProducer_;
  BitEventProducer shutdownProducer_;
  BitEventProducer thermalProducer_;
  WriteHelper helper_;
  BarrierNotifiable n_;
  uint64_t lastReport_{0};
  uint32_t lastReading_{0};
  uint8_t state_{STATE_OFF};
  uint8_t overCurrentCheckCount_{0};

  Action init() {
    adc1_config_channel_atten(channel_, ADC_CURRENT_ATTENUATION);
    LOG(INFO,
        "[%s] Monitoring h-bridge (%s %u mA max) using ADC 1:%d\n"
        "Short limit %u/4096 (%.2f mA), events (on: %s, off: %s)\n"
        "Shutdown limit %u/4096 (%.2f mA), events (on: %s, off: %s)\n"
        "Thermal warning pin %d, events (on: %s, off: %s)\n"
        "Output enable pin %d"
        , name_.c_str()
        , bridgeType_.c_str()
        , maxMilliAmps_
        , channel_
        , overCurrentLimit_
        , ((overCurrentLimit_ * maxMilliAmps_) / 4096.0f)
        , uint64_to_string_hex(shortBit_.event_on()).c_str()
        , uint64_to_string_hex(shortBit_.event_off()).c_str()
        , shutdownLimit_
        , ((shutdownLimit_ * maxMilliAmps_) / 4096.0f)
        , uint64_to_string_hex(shutdownBit_.event_on()).c_str()
        , uint64_to_string_hex(shutdownBit_.event_off()).c_str()
        , thermalWarningPin_
        , uint64_to_string_hex(thermalBit_.event_on()).c_str()
        , uint64_to_string_hex(thermalBit_.event_off()).c_str()
        , enablePin_
    );

    gpio_pad_select_gpio(enablePin_);
    ESP_ERROR_CHECK(gpio_set_direction(enablePin_, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_pulldown_en(enablePin_));
    ESP_ERROR_CHECK(gpio_set_level(enablePin_, 0));

    if (thermalWarningPin_ >= 0)
    {
      gpio_pad_select_gpio(thermalWarningPin_);
      ESP_ERROR_CHECK(gpio_set_direction(thermalWarningPin_, GPIO_MODE_INPUT));
      ESP_ERROR_CHECK(gpio_pullup_en(thermalWarningPin_));
    }

#if ENERGIZE_OPS_TRACK_ON_STARTUP
    return call_immediately(STATE(sleep_and_check_state));
#else
    return wait();
#endif
  }

  Action check()
  {
    if (state_ == STATE_OFF)
    {
      ESP_ERROR_CHECK(gpio_set_level(enablePin_, 0));
      statusLED->setStatusLED(targetLED_, StatusLED::COLOR::OFF);
      return wait();
    }

    uint8_t initialState = state_;
    StatusLED::COLOR statusLEDColor = StatusLED::COLOR::GREEN;
    std::vector<int> samples;
    while(samples.size() < adcSampleCount_) {
      samples.push_back(adc1_get_raw(channel_));
      usleep(1);
    }
    lastReading_ = (std::accumulate(samples.begin(), samples.end(), 0) / samples.size());
    if (lastReading_ >= shutdownLimit_)
    {
      state_ = STATE_SHUTDOWN;
      statusLEDColor = StatusLED::COLOR::RED_BLINK;
    }
    else if (lastReading_ >= overCurrentLimit_)
    {
      // disable the h-bridge output
      ESP_ERROR_CHECK(gpio_set_level(enablePin_, 0));
      if(overCurrentCheckCount_++ >= overCurrentRetryCount_)
      {
        state_ = STATE_OVERCURRENT;
        statusLEDColor = StatusLED::COLOR::RED;
      }
      else
      {
        return call_immediately(STATE(sleep_and_check_overcurrent));
      }
    }
    else
    {
      if (thermalWarningPin_ >= 0 && gpio_get_level(thermalWarningPin_))
      {
        state_ = STATE_THERMAL_SHUTDOWN;
        statusLEDColor = StatusLED::COLOR::YELLOW_BLINK;
      }
      else if(initialState != STATE_ON)
      {
        state_ = STATE_ON;
        overCurrentCheckCount_ = 0;
        statusLEDColor = StatusLED::COLOR::GREEN;
        if (lastReading_ >= warnLimit_)
        {
          statusLEDColor = StatusLED::COLOR::YELLOW;
        }
      }
    }
    if(esp_timer_get_time() - lastReport_ > currentReportInterval_)
    {
      lastReport_ = esp_timer_get_time();
      LOG(INFO, "[%s] %6.0f mA / %d mA", name_.c_str(), getUsage() / 1000.0f, maxMilliAmps_);
    }

    if (initialState != state_)
    {
      ESP_ERROR_CHECK(gpio_set_level(enablePin_, state_ == STATE_ON));
      // if we were in OVERCURRENT and we aren't now, or we are now
      // in OVERCURRENT, send the event.
      if ((initialState == STATE_OVERCURRENT && state_ != STATE_OVERCURRENT)
        || state_ == STATE_OVERCURRENT)
      {
        shortProducer_.SendEventReport(&helper_, n_.reset(this));
      }
      // if we were in SHUTDOWN and we aren't now, or we are now
      // in SHUTDOWN, send the event.
      if ((initialState == STATE_SHUTDOWN && state_ != STATE_SHUTDOWN)
        || state_ == STATE_SHUTDOWN)
      {
        shutdownProducer_.SendEventReport(&helper_, n_.reset(this));
      }
      // if we were in THERMAL_SHUTDOWN and we aren't now, or we are now
      // in THERMAL_SHUTDOWN, send the event.
      if ((initialState == STATE_THERMAL_SHUTDOWN && state_ != STATE_THERMAL_SHUTDOWN)
        || state_ == STATE_THERMAL_SHUTDOWN)
      {
        thermalProducer_.SendEventReport(&helper_, n_.reset(this));
      }
      statusLED->setStatusLED(targetLED_, statusLEDColor);
#if LOCONET_ENABLED
      if (!isProgTrack_)
      {
        locoNet.reportPower(state_ == STATE_ON);
      }
#endif
    }

    return call_immediately(STATE(sleep_and_check_state));
  }

  Action sleep_and_check_state() {
    return sleep_and_call(&timer_, checkInterval_, STATE(check));
  }

  Action sleep_and_check_overcurrent() {
    return sleep_and_call(&timer_, overCurrentRetryInterval_, STATE(check));
  }
};
