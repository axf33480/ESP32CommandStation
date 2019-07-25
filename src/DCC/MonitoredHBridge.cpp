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

#ifndef ADC_CURRENT_ATTENUATION
#define ADC_CURRENT_ATTENUATION ADC_ATTEN_DB_11
#endif

MonitoredHBridge::MonitoredHBridge(SimpleCanStack *stack
                 , const adc1_channel_t senseChannel
                 , const gpio_num_t enablePin
                 , const gpio_num_t thermalWarningPin
                 , const uint32_t limitMilliAmps
                 , const uint32_t maxMilliAmps
                 , const string &name
                 , const string &bridgeType
                 , const TrackOutputConfig &cfg
                 , const bool programmingTrack) :
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
                      , getUsage()
                      , isProgrammingTrack() ? JSON_VALUE_TRUE : JSON_VALUE_FALSE
                      );
}

string MonitoredHBridge::getInfoScreenData()
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

void MonitoredHBridge::broadcastStatus()
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

void MonitoredHBridge::disable()
{
  if(state_ != STATE_OFF)
  {
    state_ = STATE_OFF;
    yield_and_call(STATE(check));
    LOG(INFO, "[%s] Disabling h-bridge", name_.c_str());
  }
  statusLED->setStatusLED((StatusLED::LED)targetLED_, StatusLED::COLOR::OFF);
}

void MonitoredHBridge::enable()
{
  state_ = STATE_ON;
  yield_and_call(STATE(check));
  LOG(INFO, "[%s] Enabling h-bridge", name_.c_str());
  statusLED->setStatusLED((StatusLED::LED)targetLED_, StatusLED::COLOR::GREEN);
#if LOCONET_ENABLED
  if (!isProgTrack_)
  {
    locoNet.reportPower(true);
  }
#endif
}

StateFlowBase::Action MonitoredHBridge::init()
{
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

StateFlowBase::Action MonitoredHBridge::check()
{
  if (state_ == STATE_OFF)
  {
    ESP_ERROR_CHECK(gpio_set_level(enablePin_, 0));
    statusLED->setStatusLED((StatusLED::LED)targetLED_, StatusLED::COLOR::OFF);
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
    LOG_ERROR("[%s] Shutdown Threshold breached %6.2f mA (raw: %d)"
            , name_.c_str()
            , getUsage() / 1000.0f
            , lastReading_);
    state_ = STATE_SHUTDOWN;
    statusLEDColor = StatusLED::COLOR::RED_BLINK;
  }
  else if (lastReading_ >= overCurrentLimit_)
  {
    if(overCurrentCheckCount_++ >= overCurrentRetryCount_)
    {
      // disable the h-bridge output
      ESP_ERROR_CHECK(gpio_set_level(enablePin_, 0));
      LOG_ERROR("[%s] Overcurrent detected %6.2f mA (raw: %d)"
              , name_.c_str()
              , getUsage() / 1000.0f
              , lastReading_);
      state_ = STATE_OVERCURRENT;
      statusLEDColor = StatusLED::COLOR::RED;
    }
    else
    {
      return call_immediately(STATE(sleep_and_check_overcurrent));
    }
  }
  else if (thermalWarningPin_ >= 0 && gpio_get_level(thermalWarningPin_))
  {
    LOG_ERROR("[%s] Thermal shutdown detected", name_.c_str());
    state_ = STATE_THERMAL_SHUTDOWN;
    statusLEDColor = StatusLED::COLOR::YELLOW_BLINK;
  }
  else if(initialState != STATE_ON)
  {
    LOG(INFO, "[%s] Enabling to normal operations", name_.c_str());
    state_ = STATE_ON;
    overCurrentCheckCount_ = 0;
    statusLEDColor = StatusLED::COLOR::GREEN;
    if (lastReading_ >= warnLimit_)
    {
      statusLEDColor = StatusLED::COLOR::YELLOW;
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
    statusLED->setStatusLED((StatusLED::LED)targetLED_, statusLEDColor);
#if LOCONET_ENABLED
    if (!isProgTrack_)
    {
      locoNet.reportPower(state_ == STATE_ON);
    }
#endif
  }

  return call_immediately(STATE(sleep_and_check_state));
}