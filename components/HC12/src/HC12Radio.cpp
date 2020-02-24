/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018-2020 Mike Dunston

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

#include "HC12Radio.h"

namespace esp32cs
{

HC12Radio::HC12Radio(Service *service, uart_port_t port, gpio_num_t rx
                   , gpio_num_t tx)
                   : StateFlowBase(service), uart_(port), rx_(rx), tx_(tx)
{
#if CONFIG_HC12
  start_flow(STATE(initialize));
#endif
}

StateFlowBase::Action HC12Radio::initialize()
{
#if CONFIG_HC12
  CONFIGURE_UART("hc12", uart_, CONFIG_HC12_BAUD_RATE, rx_, tx_
               , CONFIG_HC12_BUFFER_SIZE, CONFIG_HC12_BUFFER_SIZE)

  uartFd_ = open(StringPrintf("/dev/uart/%d", uart_).c_str()
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
#endif
  return exit();
}

StateFlowBase::Action HC12Radio::data_received()
{
  if (helper_.hasError_)
  {
    LOG_ERROR("[HC12] uart read failed, giving up!");
    return exit();
  }
  else if (helper_.remaining_ == RX_BUF_SIZE)
  {
    return yield_and_call(STATE(wait_for_data));
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
  return read_nonblocking(&helper_, uartFd_, rx_buffer_, RX_BUF_SIZE
                        , STATE(data_received));
}

} // namespace esp32cs