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

#ifndef HC12_RADIO_H_
#define HC12_RADIO_H_

#include <executor/StateFlow.hxx>
#include <openlcb/SimpleStack.hxx>

class HC12Radio : public StateFlowBase {
public:
  HC12Radio(openlcb::SimpleCanStack *);
  void send(const std::string &text);
private:
  StateFlowTimer timer_{this};
  uart_port_t uart_;
  DCCPPProtocolConsumer consumer_;
  const uint64_t updateInterval_{MSEC_TO_NSEC(250)};
  STATE_FLOW_STATE(init);
  STATE_FLOW_STATE(update);
};

extern unique_ptr<HC12Radio> hc12;
#endif // HC12_RADIO_H_