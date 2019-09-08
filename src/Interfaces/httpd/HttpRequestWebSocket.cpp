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
#include "interfaces/httpd/Httpd.h"

namespace esp32cs
{
namespace httpd
{

typedef enum
{
  WS_CONNECT
, WS_DISCONNECT
, WS_MESSAGE
} WebSocketEvent;

typedef enum
{
  WS_CONTINUATION = 0x0 // Continuation Frame
, WS_TEXT         = 0x1 // Text Frame
, WS_BINARY       = 0x2 // Binary Frame
, WS_CLOSE        = 0x8 // Connection Close Frame
, WS_PING         = 0x9 // Ping Frame
, WS_PONG         = 0xa // Pong Frame
} WebSocketOpcode;

StateFlowBase::Action WebsocketFlow::read_packet()
{
  return delete_this();
}

StateFlowBase::Action WebsocketFlow::send_packet()
{
  return delete_this();
}

StateFlowBase::Action WebsocketFlow::packet_received()
{
  return delete_this();
}

} // namespace httpd
} // namespace esp32cs
