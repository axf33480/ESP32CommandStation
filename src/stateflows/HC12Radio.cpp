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

namespace esp32cs
{

HC12Radio::HC12Radio(Service *service, uart_port_t port)
  : StateFlowBase(service)
  , uart_(port)
{
  start_flow(STATE(initialize));
}

#define LOG_ESP_ERROR_AND_EXIT(cmd)                     \
{                                                       \
  esp_err_t res = cmd;                                  \
  if (res != ESP_OK)                                    \
  {                                                     \
    LOG_ERROR("[HC12] Failed to initialize UART%d: %s"  \
            , HC12_UART_NUM, esp_err_to_name(res));     \
    return exit();                                      \
  }                                                     \
}

StateFlowBase::Action HC12Radio::initialize()
{
  LOG(INFO, "[HC12] Initializing UART(%d) at %ul baud on RX %d, TX %d"
    , config_cs_hc12_uart_num(), config_cs_hc12_uart_speed()
    , config_cs_hc12_rx_pin(), config_cs_hc12_tx_pin());

  // Setup UART 115200 8N1, 2k buffer
  uart_config_t uart =
  {
    .baud_rate           = config_cs_hc12_uart_speed(),
    .data_bits           = UART_DATA_8_BITS,         // 8 bit bytes
    .parity              = UART_PARITY_DISABLE,      // no partity
    .stop_bits           = UART_STOP_BITS_1,         // one stop bit
    .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE, // no flow control
    .rx_flow_ctrl_thresh = 0,                        // unused
    .use_ref_tick        = false                     // unused
  };
  LOG_ESP_ERROR_AND_EXIT(uart_param_config(uart_, &uart))
  LOG_ESP_ERROR_AND_EXIT(uart_set_pin(uart_, config_cs_hc12_tx_pin()      \
                                    , config_cs_hc12_rx_pin()             \
                                    , UART_PIN_NO_CHANGE                  \
                                    , UART_PIN_NO_CHANGE))
  LOG_ESP_ERROR_AND_EXIT(uart_driver_install(uart_                        \
                                           , config_cs_hc12_buffer_size() \
                                           , config_cs_hc12_buffer_size() \
                                           , 0, NULL, 0))

  uartFd_ = open(StringPrintf("/dev/uart/%d"
                            , config_cs_hc12_uart_num()).c_str()
               , O_RDWR | O_NONBLOCK);
  if (uartFd_ >= 0)
  {
    LOG(INFO, "[HC12] Initialized");
    return call_immediately(STATE(wait_for_data));
  }

  // ignore error code here as we are shutting down the interface
  uart_driver_delete(uart_);
  uartFd_ = -1;

  LOG_ERROR("[HC12] Initialization failure, unable to open UART device: %s"
          , strerror(errno));
  return exit();
}

StateFlowBase::Action HC12Radio::data_received()
{
  if (helper_.hasError_)
  {
    LOG_ERROR("[HC12] uart read failed, giving up!");
    return exit();
  }
  tx_buffer_ = std::move(feed(rx_buffer_, RX_BUF_SIZE - helper_.remaining_));
  if (tx_buffer_.length() > 0)
  {
    return write_repeated(&helper_, uartFd_, tx_buffer_.c_str()
                        , tx_buffer_.length(), STATE(wait_for_data));
  }
  return call_immediately(STATE(wait_for_data));
}

StateFlowBase::Action HC12Radio::wait_for_data()
{
  return read_single(&helper_, uartFd_, rx_buffer_, RX_BUF_SIZE
                   , STATE(data_received));
}

} // namespace esp32cs