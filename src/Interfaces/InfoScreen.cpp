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
#include "InfoScreen_OLED_font.h"
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

StateFlowBase::Action InfoScreen::init() {
  LOG(INFO, "[InfoScreen] init start");
  replaceLine(INFO_SCREEN_STATION_INFO_LINE, "ESP32-CS: v%s", VERSION);
  replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "Starting Up");

#if INFO_SCREEN_ENABLED
  if(Wire.begin(INFO_SCREEN_SDA_PIN, INFO_SCREEN_SCL_PIN)) {
  // if we have a reset pin defined, attempt to reset the I2C screen
#if defined(INFO_SCREEN_RESET_PIN)
    pinMode(INFO_SCREEN_RESET_PIN, OUTPUT);
    digitalWrite(INFO_SCREEN_RESET_PIN, LOW);
    delay(50);
    digitalWrite(INFO_SCREEN_RESET_PIN, HIGH);
#endif

    // Check that we can find the screen by its address before attempting to
    // use/configure it.
    Wire.beginTransmission(INFO_SCREEN_I2C_TEST_ADDRESS);
    if(Wire.endTransmission() == 0) {
#if INFO_SCREEN_OLED
      LOG(INFO, "[InfoScreen] init OLED");
      oledDisplay.init();
      oledDisplay.setContrast(255);
      if(INFO_SCREEN_OLED_VERTICAL_FLIP == true) {
        oledDisplay.flipScreenVertically();
      }

      // NOTE: If the InfoScreen_OLED_font.h file is modified with a new font
      // definition, the name of the font needs to be declared on the next line.
      oledDisplay.setFont(Monospaced_plain_10);
#elif INFO_SCREEN_LCD
      LOG(INFO, "[InfoScreen] init LCD");
      lcdDisplay.begin(INFO_SCREEN_LCD_COLUMNS, INFO_SCREEN_LCD_LINES);
      lcdDisplay.setBacklight(255);
      lcdDisplay.clear();
      enabled = true;
#endif
      LOG(INFO, "[InfoScreen] init done, setup callback");
      return sleep_and_call(&timer_, MSEC_TO_NSEC(500), STATE(update));
    }
    LOG(WARNING, "OLED/LCD screen not found at 0x%x\n", INFO_SCREEN_I2C_TEST_ADDRESS);
    printf("Scanning for I2C devices...\n");
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    for (uint8_t addr=3; addr < 0x78; addr++) {
      if (addr % 16 == 0) {
        printf("\n%.2x:", addr);
      }
      Wire.beginTransmission(addr);
      if(Wire.endTransmission() == 0) {
        printf(" %.2x", addr);
      } else {
        printf(" --");
      }
    }
  } else {
    LOG(FATAL,
        "Failed to initialize I2C bus with SDA: %d, SCL: %d!",
        INFO_SCREEN_SDA_PIN,
        INFO_SCREEN_SCL_PIN);
  }
#endif
  return exit();
}

void InfoScreen::clear() {
  LOG(VERBOSE, "[InfoScreen] clear screen");
  for(int i = 0; i < 5; i++) {
    screenLines_[i] = "";
  }
  redraw_ = true;
}

void InfoScreen::print(int col, int row, const std::string &format, ...) {
  char buf[512] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
  screenLines_[row] = screenLines_[row].substr(0, col) + buf + screenLines_[row].substr(col + strlen(buf));
  lineChanged_[row] = true;
  LOG(VERBOSE, "[InfoScreen] print(%d,%d): %s", col, row, buf);
}

void InfoScreen::replaceLine(int row, const std::string &format, ...) {
  char buf[512] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
  screenLines_[row] = buf;
  lineChanged_[row] = true;
  LOG(VERBOSE, "[InfoScreen] replaceLine(%d): %s", row, buf);
}

StateFlowBase::Action InfoScreen::update() {
  static uint8_t _rotatingStatusIndex = 0;
  static uint8_t _rotatingStatusLineCount = 4;
  static uint8_t _motorboardIndex = 0;
  static uint32_t _lastRotation = millis();
  static uint32_t _lastUpdate = millis();
  static uint8_t _lccStatusIndex = 0;
#if LOCONET_ENABLED
  static uint8_t _firstLocoNetIndex = 0;
  if(!_firstLocoNetIndex) {
    _firstLocoNetIndex = _rotatingStatusLineCount;
    _rotatingStatusLineCount += 2;
  }
#endif
  // switch to next status line detail set every five seconds
  if(millis() - _lastRotation >= 5000) {
    _lastRotation = millis();
    ++_rotatingStatusIndex %= _rotatingStatusLineCount;
  }
  // update the status line details every second
  if(millis() - _lastUpdate >= 950) {
    _lastUpdate = millis();
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
          infoScreenCollector.getRemoteNodeCount());
      } else if (_lccStatusIndex == 1) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC Local Nodes: %d",
          infoScreenCollector.getLocalNodeCount());
      } else if (_lccStatusIndex == 2) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC dg_svc: %d",
          infoScreenCollector.getDatagramCount());
      } else if (_lccStatusIndex == 3) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC Ex: %d",
          infoScreenCollector.getExecutorCount());
      } else if (_lccStatusIndex == 4) {
        replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LCC Pool: %d/%d",
          infoScreenCollector.getPoolFreeCount(),
          infoScreenCollector.getPoolSize());
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
    lcdDisplay.print(screenLines_[line].c_str());
  }
#endif
  return sleep_and_call(&timer_, MSEC_TO_NSEC(450), STATE(update));
}
