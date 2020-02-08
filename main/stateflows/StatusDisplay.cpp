/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2020 Mike Dunston

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

#if !CONFIG_DISPLAY_TYPE_NONE
#include <Wire.h>

#if CONFIG_DISPLAY_TYPE_OLED
#include "StatusDisplay_OLED_font.h"
#if CONFIG_DISPLAY_OLED_SH1106
#include <SH1106Wire.h>
SH1106Wire oledDisplay(CONFIG_DISPLAY_ADDRESS, INFO_SCREEN_SDA_PIN
                     , INFO_SCREEN_SCL_PIN);
#elif CONFIG_DISPLAY_OLED_SSD1306
#include <SSD1306Wire.h>
SSD1306Wire oledDisplay(CONFIG_DISPLAY_ADDRESS, INFO_SCREEN_SDA_PIN
                      , INFO_SCREEN_SCL_PIN);
#endif
#elif CONFIG_DISPLAY_TYPE_LCD
#include <LiquidCrystal_PCF8574.h>
LiquidCrystal_PCF8574 lcdDisplay(CONFIG_DISPLAY_ADDRESS);
#endif
#endif

static constexpr uint8_t INFOSCREEN_ROTATING_LINE_COUNT = 5;

StatusDisplay::StatusDisplay(openlcb::SimpleCanStack *stack, Service *service)
  : StateFlowBase(service)
#if CONFIG_LOCONET
  , lineCount_(INFOSCREEN_ROTATING_LINE_COUNT + 2)
#else
  , lineCount_(INFOSCREEN_ROTATING_LINE_COUNT)
#endif // CONFIG_LOCONET
{
  clear();
  lccStatCollector_.reset(new LCCStatCollector(stack));
#if !CONFIG_DISPLAY_TYPE_NONE
  start_flow(STATE(init));
#endif
}

void StatusDisplay::clear()
{
  
  LOG(VERBOSE, "[StatusDisplay] clear screen");
  for(int line = 0; line < 5; line++)
  {
    replaceLine(line, "");
  }
}

void StatusDisplay::replaceLine(int row, const std::string &format, ...)
{
  if (row < 0)
  {
    return;
  }
  char buf[256] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
  {
#ifdef INFOSCREEN_LOCKING
    OSMutexLock l(&lock_);
#endif
    screenLines_[row] = buf;
  }
  LOG(VERBOSE, "[StatusDisplay] replaceLine(%d): %s", row, buf);
}

StateFlowBase::Action StatusDisplay::init()
{
#if !CONFIG_DISPLAY_TYPE_NONE
  replaceLine(INFO_SCREEN_STATION_INFO_LINE, "ESP32-CS: v%s", PROJECT_VER);
  replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "Starting Up");

  LOG(INFO, "[StatusDisplay] Detecting display...");
  if(Wire.begin(INFO_SCREEN_SDA_PIN, INFO_SCREEN_SCL_PIN))
  {
    // Verify that there is an I2C device on the expected address
    Wire.beginTransmission(CONFIG_DISPLAY_ADDRESS);
    if(Wire.endTransmission() == I2C_ERROR_OK)
    {
      // Device found, initialize it
#if CONFIG_DISPLAY_TYPE_OLED
      return call_immediately(STATE(initOLED));
#elif CONFIG_DISPLAY_TYPE_LCD
      return call_immediately(STATE(initLCD));
#endif
    }
    return yield_and_call(STATE(i2cScan));
  }
  else
  {
    LOG(FATAL, "Failed to initialize I2C bus with SDA: %d, SCL: %d!"
      , INFO_SCREEN_SDA_PIN, INFO_SCREEN_SCL_PIN);
  }
#endif
  // The only time we should encounter this case is if the I2C init
  // fails. Cleanup and exit the flow.
  LOG(WARNING, "[StatusDisplay] no display detected");
  return exit();
}

StateFlowBase::Action StatusDisplay::i2cScan() {
#if !CONFIG_DISPLAY_TYPE_NONE
  // Scan the I2C bus and dump the output of devices that respond
  std::string scanresults =
    "Scanning for I2C devices...\n"
    "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n"
    "00:         ";
  scanresults.reserve(256);
  for (uint8_t addr=3; addr < 0x78; addr++) {
    if (addr % 16 == 0) {
      scanresults += "\n" + int64_to_string_hex(addr) + ":";
    }
    Wire.beginTransmission(addr);
    if(Wire.endTransmission() == 0) {
      scanresults += int64_to_string_hex(addr);
    } else {
      scanresults += " --";
    }
  }
  LOG(WARNING,
      "I2C display not found at 0x%02x\n%s",
      CONFIG_DISPLAY_ADDRESS,
      scanresults.c_str());
#endif // CONFIG_DISPLAY_TYPE_NONE
  // we are done, shutdown the flow
  return exit();
}

StateFlowBase::Action StatusDisplay::initOLED()
{
#if CONFIG_DISPLAY_TYPE_OLED
#ifdef DISPLAY_OLED_RESET_PIN
  static bool resetCalled = false;
  if(!resetCalled)
  {
    // if we have a reset pin defined, attempt to reset the I2C screen
    LOG(INFO, "[StatusDisplay] Resetting OLED display");
    pinMode(DISPLAY_OLED_RESET_PIN, OUTPUT);
    digitalWrite(DISPLAY_OLED_RESET_PIN, LOW);
    return sleep_and_call(&timer_, MSEC_TO_NSEC(50), STATE(initOLED));
    resetCalled = true;;
  }
  else
  {
    digitalWrite(DISPLAY_OLED_RESET_PIN, HIGH);
  }
#endif // DISPLAY_OLED_RESET_PIN
  LOG(INFO,
      "[StatusDisplay] Detected OLED on address %02x, initializing display...",
      INFO_SCREEN_I2C_TEST_ADDRESS);
  if(!oledDisplay.init())
  {
    LOG_ERROR("[StatusDisplay] Failed to initailize OLED screen, disabling!");
    return exit();
  }
  oledDisplay.setContrast(255);
#if CONFIG_DISPLAY_OLED_VFLIP
  oledDisplay.flipScreenVertically();
#endif // CONFIG_DISPLAY_OLED_VFLIP
  // NOTE: If the InfoScreen_OLED_font.h file is modified with a new font
  // definition, the name of the font needs to be declared on the next line.
  oledDisplay.setFont(Monospaced_plain_10);
  return call_immediately(STATE(update));
#else
  return exit();
#endif // CONFIG_DISPLAY_TYPE_OLED
}

StateFlowBase::Action StatusDisplay::initLCD()
{
#if CONFIG_DISPLAY_TYPE_LCD
  LOG(INFO,
      "[StatusDisplay] Detected LCD on address %02x, initializing %dx%x display..."
    , CONFIG_DISPLAY_ADDRESS, CONFIG_DISPLAY_LCD_COLUMN_COUNT
    , CONFIG_DISPLAY_LCD_LINE_COUNT);
  lcdDisplay.begin(CONFIG_DISPLAY_LCD_COLUMN_COUNT, CONFIG_DISPLAY_LCD_LINE_COUNT);
  lcdDisplay.setBacklight(255);
  lcdDisplay.clear();
  return call_immediately(STATE(update));
#else
  return exit();
#endif // CONFIG_DISPLAY_TYPE_LCD
}

StateFlowBase::Action StatusDisplay::update()
{
#if CONFIG_LOCONET
  static uint8_t _firstLocoNetIndex = INFOSCREEN_ROTATING_LINE_COUNT;
#endif
  // switch to next status line detail set after 10 iterations
  if(++updateCount_ > 10)
  {
    updateCount_ = 0;
    ++rotatingIndex_ %= lineCount_;
  }
  // update the status line details every other iteration
  if(updateCount_ % 2)
  {
    if(rotatingIndex_ == 0)
    {
      replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "Free Heap:%d"
                , heap_caps_get_free_size(MALLOC_CAP_INTERNAL)
      );
    }
    else if (rotatingIndex_ == 1)
    {
      uint64_t seconds = USEC_TO_SEC(esp_timer_get_time());
      replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "Uptime: %02d:%02d:%02d"
                , (uint32_t)(seconds / 3600), (uint32_t)(seconds % 3600) / 60
                , (uint32_t)(seconds % 60)
      );
    }
    else if (rotatingIndex_ == 2)
    {
      replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "Active Locos:%3d"
                , Singleton<AllTrainNodes>::instance()->size()
      );
    }
    else if (rotatingIndex_ == 3)
    {
      replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE
                , trackSignal->get_status_screen_data()
      );
    }
    else if (rotatingIndex_ == 4)
    {
      static uint8_t _lccStatusIndex = 0;
      ++_lccStatusIndex %= 5;
      if(_lccStatusIndex == 0)
      {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC Nodes: %d"
                  , lccStatCollector_->getRemoteNodeCount()
        );
      }
      else if (_lccStatusIndex == 1)
      {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC Lcl: %d"
                  , lccStatCollector_->getLocalNodeCount()
        );
      }
      else if (_lccStatusIndex == 2)
      {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC dg_svc: %d"
                  , lccStatCollector_->getDatagramCount()
        );
      }
      else if (_lccStatusIndex == 3)
      {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC Ex: %d"
                  , lccStatCollector_->getExecutorCount()
        );
      }
      else if (_lccStatusIndex == 4)
      {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC Pool: %d/%d"
                  , lccStatCollector_->getPoolFreeCount()
                  , lccStatCollector_->getPoolSize()
        );
      }
#if CONFIG_LOCONET
    }
    else if (rotatingIndex_ == _firstLocoNetIndex)
    {
      replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LN-RX: %d/%d"
                , locoNet.getRxStats()->rxPackets
                , locoNet.getRxStats()->rxErrors
      );
    }
    else if (rotatingIndex_ == _firstLocoNetIndex + 1)
    {
      replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LN-TX: %d/%d/%d"
                , locoNet.getTxStats()->txPackets
                , locoNet.getTxStats()->txErrors
                , locoNet.getTxStats()->collisions
      );
#endif
    }
  }

  {
#ifdef INFOSCREEN_LOCKING
    OSMutexLock l(&lock_);
#endif
#if CONFIG_DISPLAY_TYPE_OLED
    oledDisplay.clear();
    for(int line = 0; line < CONFIG_DISPLAY_OLED_LINE_COUNT; line++)
    {
      oledDisplay.drawString(0, line * Monospaced_plain_10[1]
                           , screenLines_[line].c_str()
      );
    }
    oledDisplay.display();
#elif CONFIG_DISPLAY_TYPE_LCD
    for(int line = 0; line < CONFIG_DISPLAY_LCD_LINE_COUNT; line++)
    {
      lcdDisplay.setCursor(0, line);
      // space pad to the width of the LCD
      while(screenLines_[line].length() < CONFIG_DISPLAY_LCD_COLUMN_COUNT)
      {
        screenLines_[line] += ' ';
      }
      lcdDisplay.print(screenLines_[line].c_str());
    }
#endif
  }
  return sleep_and_call(&timer_, MSEC_TO_NSEC(450), STATE(update));
}