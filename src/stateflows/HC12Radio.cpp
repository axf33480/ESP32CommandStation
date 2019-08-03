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

unique_ptr<HC12Radio> hc12;

#ifndef HC12_UART_NUM
#define HC12_UART_NUM 1
#endif

#ifndef HC12_RADIO_BAUD
#define HC12_RADIO_BAUD 19200
#endif
#ifndef HC12_RX_PIN
#define HC12_RX_PIN 9
#endif
#ifndef HC12_TX_PIN
#define HC12_TX_PIN 10
#endif

HC12Radio::HC12Radio(Service *service) : StateFlowBase(service), uart_((uart_port_t)HC12_UART_NUM) {
#if HC12_RADIO_ENABLED
  start_flow(STATE(init));
#endif
}

StateFlowBase::Action HC12Radio::init() {
  LOG(INFO, "[HC12] Initializing UART(%d) at %ul baud on RX %d, TX %d",
      HC12_UART_NUM, HC12_RADIO_BAUD, HC12_RX_PIN, HC12_TX_PIN);

  // Setup UART 115200 8N1, 2k buffer
  uart_config_t uart = {
    .baud_rate           = 115200,
    .data_bits           = UART_DATA_8_BITS,         // 8 bit bytes
    .parity              = UART_PARITY_DISABLE,      // no partity
    .stop_bits           = UART_STOP_BITS_1,         // one stop bit
    .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE, // no flow control
    .rx_flow_ctrl_thresh = 0,                        // unused
    .use_ref_tick        = false                     // unused
  };
  esp_err_t res = uart_param_config(uart_, &uart);
  if (res != ESP_OK) {
    LOG_ERROR("[HC12] Failed to initialize UART%d: %s", HC12_UART_NUM, esp_err_to_name(res));
    return exit();
  }
  res = uart_set_pin(uart_, HC12_TX_PIN, HC12_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (res != ESP_OK) {
    LOG_ERROR("[HC12] Failed to initialize UART%d: %s", HC12_UART_NUM, esp_err_to_name(res));
    return exit();
  }
  res = uart_driver_install(uart_, 2048, 0, 0, NULL, 0);
  if (res != ESP_OK) {
    LOG_ERROR("[HC12] Failed to initialize UART%d: %s", HC12_UART_NUM, esp_err_to_name(res));
    return exit();
  }
  LOG_ERROR("[HC12] Initialized");
  return sleep_and_call(&timer_, updateInterval_, STATE(update));
}

StateFlowBase::Action HC12Radio::update() {
  uint8_t buf[128] = {0};
  int bytesRead = uart_read_bytes(uart_, buf, 128, 1);
  if(bytesRead > 0) {
    consumer_.feed(buf, bytesRead);
  }
  return sleep_and_call(&timer_, updateInterval_, STATE(update));
}

void HC12Radio::send(const std::string &buf) {
#if HC12_RADIO_ENABLED
  uart_write_bytes(uart_, buf.c_str(), buf.length());
#endif
}
