/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2019 Mike Dunston

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

#ifndef INFO_SCREEN_SDA_PIN
#define INFO_SCREEN_SDA_PIN SDA
#endif
#ifndef INFO_SCREEN_SCL_PIN
#define INFO_SCREEN_SCL_PIN SCL
#endif

#include <Wire.h>
#if INFO_SCREEN_OLED
#include "interfaces/InfoScreen_OLED_font.h"
#define INFO_SCREEN_I2C_TEST_ADDRESS INFO_SCREEN_OLED_I2C_ADDRESS
#if OLED_CHIPSET == SH1106
#include <SH1106Wire.h>
SH1106Wire oledDisplay(INFO_SCREEN_OLED_I2C_ADDRESS, INFO_SCREEN_SDA_PIN, INFO_SCREEN_SCL_PIN);
#elif OLED_CHIPSET == SSD1306
#include <SSD1306Wire.h>
SSD1306Wire oledDisplay(INFO_SCREEN_OLED_I2C_ADDRESS, INFO_SCREEN_SDA_PIN, INFO_SCREEN_SCL_PIN);
#endif
#elif INFO_SCREEN_LCD
#define INFO_SCREEN_I2C_TEST_ADDRESS INFO_SCREEN_LCD_I2C_ADDRESS
#include <LiquidCrystal_PCF8574.h>
LiquidCrystal_PCF8574 lcdDisplay(INFO_SCREEN_LCD_I2C_ADDRESS);
#endif

void InfoScreen::clear() {
  LOG(VERBOSE, "[InfoScreen] clear screen");
  for(int i = 0; i < 5; i++) {
    screenLines_[i] = ""; 
  }
}

void InfoScreen::replaceLine(int row, const std::string &format, ...) {
  if(row < 0) {
    return;
  }
  char buf[256] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
  screenLines_[row] = buf;
  LOG(VERBOSE, "[InfoScreen] replaceLine(%d): %s", row, buf);
}

StateFlowBase::Action InfoScreen::init() {
  replaceLine(INFO_SCREEN_STATION_INFO_LINE, "ESP32-CS: v%s", VERSION);
  replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "Starting Up");

  LOG(INFO, "[InfoScreen] Initializing");
  if(Wire.begin(INFO_SCREEN_SDA_PIN, INFO_SCREEN_SCL_PIN)) {
    // Verify that there is an I2C device on the expected address
    Wire.beginTransmission(INFO_SCREEN_I2C_TEST_ADDRESS);
    if(Wire.endTransmission() == I2C_ERROR_OK) {
      // Device found, initialize it
#if INFO_SCREEN_OLED
      return call_immediately(STATE(initOLED));
#elif INFO_SCREEN_LCD
      return call_immediately(STATE(initLCD));
#endif
    }
    return yield_and_call(STATE(i2cScan));
  } else {
    LOG(FATAL,
        "Failed to initialize I2C bus with SDA: %d, SCL: %d!",
        INFO_SCREEN_SDA_PIN,
        INFO_SCREEN_SCL_PIN);
  }
  // The only time we should encounter this case is if the I2C init
  // fails. Cleanup and exit the flow.
  LOG(WARNING, "[InfoScreen] no output device");
  return exit();
}

StateFlowBase::Action InfoScreen::i2cScan() {
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
      INFO_SCREEN_I2C_TEST_ADDRESS,
      scanresults.c_str());

  // we are done, shutdown the flow
  return exit();
}

StateFlowBase::Action InfoScreen::initOLED() {
#if INFO_SCREEN_OLED
#if defined(INFO_SCREEN_RESET_PIN)
  static bool resetCalled = false;
  if(!resetCalled) {
    // if we have a reset pin defined, attempt to reset the I2C screen
    LOG(INFO, "[InfoScreen] Resetting OLED display");
    pinMode(INFO_SCREEN_RESET_PIN, OUTPUT);
    digitalWrite(INFO_SCREEN_RESET_PIN, LOW);
    return sleep_and_call(&timer_, MSEC_TO_NSEC(50), STATE(initOLED));
    resetCalled = true;;
  } else {
    digitalWrite(INFO_SCREEN_RESET_PIN, HIGH);
  }
#endif
  LOG(INFO,
      "[InfoScreen] Detected OLED on address %02x, initializing display...",
      INFO_SCREEN_I2C_TEST_ADDRESS);
  if(!oledDisplay.init()) {
    LOG_ERROR("[InfoScreen] Failed to initailize OLED screen, disabling!");
    return exit();
  }
  oledDisplay.setContrast(255);
#if INFO_SCREEN_OLED_VERTICAL_FLIP
  oledDisplay.flipScreenVertically();
#endif
  // NOTE: If the InfoScreen_OLED_font.h file is modified with a new font
  // definition, the name of the font needs to be declared on the next line.
  oledDisplay.setFont(Monospaced_plain_10);
  return call_immediately(STATE(update));
#else
  return exit();
#endif
}

StateFlowBase::Action InfoScreen::initLCD() {
#if INFO_SCREEN_LCD
  LOG(INFO,
      "[InfoScreen] Detected LCD on address %02x, initializing %dx%x display...",
      INFO_SCREEN_I2C_TEST_ADDRESS, INFO_SCREEN_LCD_COLUMNS, INFO_SCREEN_LCD_LINES);
  lcdDisplay.begin(INFO_SCREEN_LCD_COLUMNS, INFO_SCREEN_LCD_LINES);
  lcdDisplay.setBacklight(255);
  lcdDisplay.clear();
  return call_immediately(STATE(update));
#else
  return exit();
#endif
}

StateFlowBase::Action InfoScreen::update() {
  static uint8_t _rotatingStatusIndex = 0;
  static uint8_t _rotatingStatusLineCount = 4;
  static uint8_t _motorboardIndex = 0;
  static uint8_t _lccStatusIndex = 0;
  static uint8_t _lastRotation = 0;
#if LOCONET_ENABLED
  static uint8_t _firstLocoNetIndex = 0;
  if(!_firstLocoNetIndex) {
    _firstLocoNetIndex = _rotatingStatusLineCount;
    _rotatingStatusLineCount += 2;
  }
#endif
  // switch to next status line detail set after 10 iterations
  if(++_lastRotation > 10) {
    _lastRotation = 0;
    ++_rotatingStatusIndex %= _rotatingStatusLineCount;
  }
  // update the status line details every other iteration
  if(_lastRotation % 2) {
    if(_rotatingStatusIndex == 0) {
      replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "Free Heap:%d",
        ESP.getFreeHeap());
    } else if (_rotatingStatusIndex == 1) {
      replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "Active Locos:%3d",
        LocomotiveManager::getActiveLocoCount());
    } else if (_rotatingStatusIndex == 2) {
      ++_motorboardIndex %= MotorBoardManager::getMotorBoardCount();
      auto board = MotorBoardManager::getBoardByName(MotorBoardManager::getBoardNames()[_motorboardIndex]);
      if(board && (board->isOn() || board->isOverCurrent())) {
        if(board->isOverCurrent()) {
          replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "%s:F (%2.2f A)",
            board->getName().c_str(), board->getCurrentDraw() / 1000.0f);
        } else if(board->isOn()) {
          replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "%s:On (%2.2f A)",
            board->getName().c_str(), board->getCurrentDraw() / 1000.0f);
        }
      } else if(board) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "%s:Off",
          board->getName().c_str());
      }
    } else if (_rotatingStatusIndex == 3) {
      ++_lccStatusIndex %= 5;
      if(_lccStatusIndex == 0) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC Nodes: %d",
          infoScreenCollector->getRemoteNodeCount());
      } else if (_lccStatusIndex == 1) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC Lcl: %d",
          infoScreenCollector->getLocalNodeCount());
      } else if (_lccStatusIndex == 2) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC dg_svc: %d",
          infoScreenCollector->getDatagramCount());
      } else if (_lccStatusIndex == 3) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC Ex: %d",
          infoScreenCollector->getExecutorCount());
      } else if (_lccStatusIndex == 4) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC Pool: %d/%d",
          infoScreenCollector->getPoolFreeCount(),
          infoScreenCollector->getPoolSize());
      }
#if LOCONET_ENABLED
    } else if (_rotatingStatusIndex == _firstLocoNetIndex) {
      replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LN-RX: %d/%d",
        locoNet.getRxStats()->rxPackets, locoNet.getRxStats()->rxErrors);
    } else if (_rotatingStatusIndex == _firstLocoNetIndex + 1) {
      replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LN-TX: %d/%d/%d",
        locoNet.getTxStats()->txPackets, locoNet.getTxStats()->txErrors, locoNet.getTxStats()->collisions);
#endif
    }
  }
#if INFO_SCREEN_OLED
  oledDisplay.clear();
  for(int line = 0; line < INFO_SCREEN_OLED_LINES; line++) {
    oledDisplay.drawString(0, line * Monospaced_plain_10[1], screenLines_[line].c_str());
  }
  oledDisplay.display();
#elif INFO_SCREEN_LCD
  for(int line = 0; line < INFO_SCREEN_LCD_LINES; line++) {
    lcdDisplay.setCursor(0, line);
    // space pad to the width of the LCD
    while(screenLines_[line].length() < INFO_SCREEN_LCD_COLUMNS) {
      screenLines_[line] += ' ';
    }
    lcdDisplay.print(screenLines_[line].c_str());
  }
#endif
  return sleep_and_call(&timer_, MSEC_TO_NSEC(450), STATE(update));
}
