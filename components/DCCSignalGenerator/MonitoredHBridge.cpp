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

#include "MonitoredHBridge.h"
#include <dcc/ProgrammingTrackBackend.hxx>
#include <json.hpp>
#include <numeric>
#include <StatusLED.h>

MonitoredHBridge::MonitoredHBridge(openlcb::Node *node
                                 , Service *service
                                 , const adc1_channel_t senseChannel
                                 , const gpio_num_t enablePin
                                 , const uint32_t limitMilliAmps
                                 , const uint32_t maxMilliAmps
                                 , const string &name
                                 , const string &bridgeType
                                 , const esp32cs::TrackOutputConfig &cfg)
  : StateFlowBase(service)
  , DefaultConfigUpdateListener()
  , channel_(senseChannel)
  , enablePin_(enablePin)
  , thermalWarningPin_((gpio_num_t)-1)
  , maxMilliAmps_(maxMilliAmps)
  , name_(name)
  , bridgeType_(bridgeType)
  , isProgTrack_(false)
  , overCurrentLimit_(((((limitMilliAmps << 3) + limitMilliAmps) / 10) << 12) / maxMilliAmps_) // ~90% max value
  , shutdownLimit_(4080)
  , cfg_(cfg)
  , targetLED_(StatusLED::LED::OPS_TRACK)
  , shortBit_(node, 0, 0, &state_, STATE_OVERCURRENT)
  , shutdownBit_(node, 0, 0, &state_, STATE_SHUTDOWN)
  , thermalBit_(node, 0, 0, &state_, STATE_THERMAL_SHUTDOWN)
  , shortProducer_(&shortBit_)
  , shutdownProducer_(&shortBit_)
  , thermalProducer_(&shortBit_)
{
  // set warning limit to ~75% of overcurrent limit
  warnLimit_ = ((overCurrentLimit_ << 1) + overCurrentLimit_) >> 2;
}

MonitoredHBridge::MonitoredHBridge(openlcb::Node *node
                                 , Service *service
                                 , const adc1_channel_t senseChannel
                                 , const gpio_num_t enablePin
                                 , const gpio_num_t thermalWarningPin
                                 , const uint32_t limitMilliAmps
                                 , const uint32_t maxMilliAmps
                                 , const string &name
                                 , const string &bridgeType
                                 , const esp32cs::TrackOutputConfig &cfg)
  : StateFlowBase(service)
  , DefaultConfigUpdateListener()
  , channel_(senseChannel)
  , enablePin_(enablePin)
  , thermalWarningPin_(thermalWarningPin)
  , maxMilliAmps_(maxMilliAmps)
  , name_(name)
  , bridgeType_(bridgeType)
  , isProgTrack_(false)
  , overCurrentLimit_(((((limitMilliAmps << 3) + limitMilliAmps) / 10) << 12) / maxMilliAmps_) // ~90% max value
  , shutdownLimit_(4080)
  , cfg_(cfg)
  , targetLED_(StatusLED::LED::OPS_TRACK)
  , shortBit_(node, 0, 0, &state_, STATE_OVERCURRENT)
  , shutdownBit_(node, 0, 0, &state_, STATE_SHUTDOWN)
  , thermalBit_(node, 0, 0, &state_, STATE_THERMAL_SHUTDOWN)
  , shortProducer_(&shortBit_)
  , shutdownProducer_(&shortBit_)
  , thermalProducer_(&shortBit_)
{
  // set warning limit to ~75% of overcurrent limit
  warnLimit_ = ((overCurrentLimit_ << 1) + overCurrentLimit_) >> 2;
}

MonitoredHBridge::MonitoredHBridge(openlcb::Node *node
                                 , Service *service
                                 , const adc1_channel_t senseChannel
                                 , const gpio_num_t enablePin
                                 , const uint32_t maxMilliAmps
                                 , const string &name
                                 , const string &bridgeType
                                 , const esp32cs::TrackOutputConfig &cfg)
  : StateFlowBase(service)
  , DefaultConfigUpdateListener()
  , channel_(senseChannel)
  , enablePin_(enablePin)
  , thermalWarningPin_((gpio_num_t)-1)
  , maxMilliAmps_(maxMilliAmps)
  , name_(name)
  , bridgeType_(bridgeType)
  , isProgTrack_(true)
  , overCurrentLimit_((250 << 12) / maxMilliAmps_) // ~250mA
  , shutdownLimit_((500 << 12) / maxMilliAmps_)
  , progAckLimit_((60 << 12) / maxMilliAmps_)      // ~60mA
  , cfg_(cfg)
  , targetLED_(StatusLED::LED::PROG_TRACK)
  , shortBit_(node, 0, 0, &state_, STATE_OVERCURRENT)
  , shutdownBit_(node, 0, 0, &state_, STATE_SHUTDOWN)
  , thermalBit_(node, 0, 0, &state_, STATE_THERMAL_SHUTDOWN)
  , shortProducer_(&shortBit_)
  , shutdownProducer_(&shortBit_)
  , thermalProducer_(&shortBit_)
{
  // set warning limit to ~75% of overcurrent limit
  warnLimit_ = ((overCurrentLimit_ << 1) + overCurrentLimit_) >> 2;
}

string MonitoredHBridge::getState()
{
  switch (state_)
  {
    case STATE_ON:
      return "Normal";
    case STATE_OVERCURRENT:
      return "Fault";
    case STATE_OFF:
    default:
      return "Off";
  }
  return "Error";
}

string MonitoredHBridge::getStateAsJson()
{
  return StringPrintf("{"
                        "\"name\":\"%s\","
                        "\"state\":\"%s\","
                        "\"usage\":%.2f,"
                        "\"prog\":\"%s\""
                      "}"
                      , name_.c_str()
                      , getState().c_str()
                      , getUsage() / 1000.0f
                      , isProgrammingTrack() ? "true" : "false"
                      );
}

string MonitoredHBridge::getStatusData()
{
  if (state_ == STATE_ON)
  {
    return StringPrintf("%s:On (%2.2f A)", name_.c_str(), getUsage() / 1000.0f);
  }
  else if (state_ == STATE_OVERCURRENT)
  {
    return StringPrintf("%s:F (%2.2f A)", name_.c_str(), getUsage() / 1000.0f);
  }
  return StringPrintf("%s:Off", name_.c_str());
}

string MonitoredHBridge::get_state_for_dccpp()
{
  if (state_ == STATE_ON)
  {
    return StringPrintf("<p1 %s><a %s %d>", name_.c_str(), name_.c_str(), getLastReading());
  }
  else if (state_ == STATE_OVERCURRENT)
  {
    return StringPrintf("<p2 %s>", name_.c_str());
  }
  return StringPrintf("<p0 %s>", name_.c_str());
}

void MonitoredHBridge::disable()
{
  if(state_ != STATE_OFF)
  {
    OSMutexLock l(&requestedStateLock_);
    requestedState_ = STATE_OFF;
    timer_.ensure_triggered();
  }
}

void MonitoredHBridge::enable()
{
  OSMutexLock l(&requestedStateLock_);
  requestedState_ = STATE_ON;
  timer_.ensure_triggered();
}

StateFlowBase::Action MonitoredHBridge::init()
{
  adc1_config_channel_atten(channel_, (adc_atten_t)CONFIG_ADC_ATTENUATION);
  string thermalPinLine = "";
  string progAckLine = "";
  if (thermalWarningPin_ >= 0)
  {
    thermalPinLine =
      StringPrintf("\nThermal warning pin %d, events (on: %s, off: %s)"
                 , thermalWarningPin_
                 , uint64_to_string_hex(thermalBit_.event_on()).c_str()
                 , uint64_to_string_hex(thermalBit_.event_off()).c_str());
  }

  if (isProgTrack_)
  {
    progAckLine = StringPrintf("\nProg ACK: %u/4096 (%.2f mA)", progAckLimit_
                            , ((progAckLimit_ * maxMilliAmps_) / 4096.0f));
  }

  LOG(INFO,
      "[%s] Monitoring h-bridge (%s %u mA max) using ADC 1:%d, en-pin:%d\n"
      "Short limit %u/4096 (%.2f mA), events (on: %s, off: %s)\n"
      "Shutdown limit %u/4096 (%.2f mA), events (on: %s, off: %s)"
      "%s%s"
      , name_.c_str()
      , bridgeType_.c_str()
      , maxMilliAmps_
      , channel_
      , enablePin_
      , overCurrentLimit_
      , ((overCurrentLimit_ * maxMilliAmps_) / 4096.0f)
      , uint64_to_string_hex(shortBit_.event_on()).c_str()
      , uint64_to_string_hex(shortBit_.event_off()).c_str()
      , shutdownLimit_
      , ((shutdownLimit_ * maxMilliAmps_) / 4096.0f)
      , uint64_to_string_hex(shutdownBit_.event_on()).c_str()
      , uint64_to_string_hex(shutdownBit_.event_off()).c_str()
      , thermalPinLine.c_str()
      , progAckLine.c_str()
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

#if CONFIG_OPS_ENERGIZE_ON_STARTUP
  if (!isProgTrack_)
  {
    enable();
  }
#endif

  return call_immediately(STATE(sleep_and_check_state));
}

StateFlowBase::Action MonitoredHBridge::check()
{
  uint8_t initialState = state_;
  StatusLED::COLOR statusLEDColor = StatusLED::COLOR::GREEN;
  vector<int> samples;

  {
    OSMutexLock l(&requestedStateLock_);
    if (lastRequestedState_ != requestedState_)
    {
      lastRequestedState_ = requestedState_;
      if (requestedState_ == STATE_OFF)
      {
        state_ = STATE_OFF;
        LOG(INFO, "[%s] Disabling track output", name_.c_str());
        Singleton<StatusLED>::instance()->setStatusLED(
          (StatusLED::LED)targetLED_, StatusLED::COLOR::OFF);
        ESP_ERROR_CHECK(gpio_set_level(enablePin_, 0));
        return call_immediately(STATE(sleep_and_check_state));
      }
      else if (requestedState_ == STATE_ON)
      {
        state_ = STATE_ON;
        LOG(INFO, "[%s] Enabling track output", name_.c_str());
        Singleton<StatusLED>::instance()->setStatusLED(
          (StatusLED::LED)targetLED_, StatusLED::COLOR::GREEN);
        ESP_ERROR_CHECK(gpio_set_level(enablePin_, 1));
        return call_immediately(STATE(sleep_and_check_state));
      }
    }
    else if (state_ == STATE_OFF)
    {
      // go back to sleep immediately since we are in an OFF state
      return call_immediately(STATE(sleep_and_check_state));
    }
  }

  // collect samples from ADC
  while(samples.size() < adcSampleCount_) {
    samples.push_back(adc1_get_raw(channel_));
    ets_delay_us(1);
  }
  // average the collected samples
  lastReading_ = (std::accumulate(samples.begin(), samples.end(), 0) / samples.size());

  if (lastReading_ >= shutdownLimit_)
  {
    // If the average sample exceeds the shutdown limit (~90% typically)
    // trigger an immediate shutdown.
    LOG_ERROR("[%s] Shutdown threshold breached %6.2f mA (raw: %d / %d)"
            , name_.c_str()
            , getUsage() / 1000.0f
            , lastReading_
            , shutdownLimit_);
    state_ = STATE_SHUTDOWN;
    statusLEDColor = StatusLED::COLOR::RED_BLINK;
  }
  else if (lastReading_ >= overCurrentLimit_)
  {
    // If we have at least a couple averages that are over the soft limit
    // trigger an immediate shutdown as a short is likely to have occurred.
    if(overCurrentCheckCount_++ >= overCurrentRetryCount_)
    {
      // disable the h-bridge output
      ESP_ERROR_CHECK(gpio_set_level(enablePin_, 0));
      LOG_ERROR("[%s] Overcurrent detected %6.2f mA (raw: %d / %d)"
              , name_.c_str()
              , getUsage() / 1000.0f
              , lastReading_
              , overCurrentLimit_);
      state_ = STATE_OVERCURRENT;
      statusLEDColor = StatusLED::COLOR::RED;
    }
    else
    {
      return call_immediately(STATE(sleep_and_check_overcurrent));
    }
  }
  else if (thermalWarningPin_ >= 0 && gpio_get_level(thermalWarningPin_) == 0)
  {
    // If we have at least a couple thermal warnings raised by the h-bridge
    // trigger an immediate shutdown.
    if (thermalWarningCheckCount_++ > thermalWarningRetryCount_ &&
        initialState != STATE_THERMAL_SHUTDOWN)
    {
      LOG_ERROR("[%s] Thermal shutdown detected", name_.c_str());
      state_ = STATE_THERMAL_SHUTDOWN;
      statusLEDColor = StatusLED::COLOR::YELLOW_BLINK;
    }
    else
    {
      return call_immediately(STATE(sleep_and_check_thermal_warning));
    }
  }
  else if(initialState != STATE_ON)
  {
    // If the initial state was not ON (over current, shutdown, thermal, etc)
    // and we have reached this point in the checks we are below the configured
    // limits so enable the track output again.
    LOG(INFO, "[%s] Enabling track output", name_.c_str());
    state_ = STATE_ON;
    overCurrentCheckCount_ = 0;
    statusLEDColor = StatusLED::COLOR::GREEN;
    if (lastReading_ >= warnLimit_)
    {
      statusLEDColor = StatusLED::COLOR::YELLOW;
    }
  }

  // If this is the programming track and the average reading is at least the
  // configured ack level, send a notification to the ProgrammingTrackBackend
  // to wake it up.
  if (isProgTrack_ && state_ == STATE_ON && lastReading_ >= progAckLimit_)
  {
    Singleton<ProgrammingTrackBackend>::instance()->notify_service_mode_ack();
  }

  if (esp_timer_get_time() - lastReport_ > currentReportInterval_)
  {
    lastReport_ = esp_timer_get_time();
    LOG(INFO, "[%s] %6.0f mA / %d mA", name_.c_str(), getUsage() / 1000.0f
      , maxMilliAmps_);
  }

  if (initialState != state_)
  {
    // If this is the programming track notify the ProgrammingTrackBackend of
    // a possible short condition if our state is NOT ON.
    if (isProgTrack_ && state_ != STATE_ON)
    {
      Singleton<ProgrammingTrackBackend>::instance()->notify_service_mode_short();
    }

    // Enable or disable the h-bridge at this point based on the updated state.
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

    // Set our LED to the updated state color value.
    Singleton<StatusLED>::instance()->setStatusLED((StatusLED::LED)targetLED_
                                                 , statusLEDColor);
#if LOCONET_ENABLED
    if (!isProgTrack_)
    {
      locoNet.reportPower(state_ == STATE_ON);
    }
#endif
  }

  // go back to sleep until next interval.
  return call_immediately(STATE(sleep_and_check_state));
}