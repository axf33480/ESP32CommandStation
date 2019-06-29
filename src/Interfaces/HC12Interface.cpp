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

#include "ESP32CommandStation.h"

#ifndef HC12_RADIO_BAUD
#define HC12_RADIO_BAUD 19200
#endif
#ifndef HC12_UART_NUM
#define HC12_UART_NUM 1
#endif
#ifndef HC12_RX_PIN
#define HC12_RX_PIN 9
#endif
#ifndef HC12_TX_PIN
#define HC12_TX_PIN 10
#endif

StateFlowBase::Action HC12Interface::init() {
  LOG(INFO, "[HC12] Initializing UART(%d) at %ul baud on RX %d, TX %d",
      HC12_UART_NUM, HC12_RADIO_BAUD, HC12_RX_PIN, HC12_TX_PIN);
  // initialize the uart device with a 256 byte buffer
  uart_ = uartBegin(HC12_UART_NUM, HC12_RADIO_BAUD, SERIAL_8N1, HC12_RX_PIN, HC12_TX_PIN, 256, false);
  if(uart_) {
    return sleep_and_call(&timer_, updateInterval_, STATE(update));
  }
  LOG_ERROR("[HC12] Initialization failed");
  return exit();
}

StateFlowBase::Action HC12Interface::update() {
  if (uartAvailable(uart_)) {
    uint8_t buf[128] = {0};
    auto len = std::min(uartAvailable(uart_), (uint32_t)128);
    for (int index = 0; index < len; index++) {
      buf[index] = uartRead(uart_);
    }
    consumer_.feed(&buf[0], len);
  }
  return sleep_and_call(&timer_, updateInterval_, STATE(update));
}

void HC12Interface::send(const std::string &buf) {
  if(uart_) {
    uartWriteBuf(uart_, (uint8_t *)(buf.c_str()), buf.length());
  }
}
