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

#ifndef DNS_SERVER_H_
#define DNS_SERVER_H_

#include <stdint.h>
#include "os/OS.hxx"

class CaptivePortalDNSD
{
public:
  CaptivePortalDNSD(ip4_addr_t, uint16_t=53, string="dnsd");
  ~CaptivePortalDNSD();
  void dns_process_thread();
private:
  ip4_addr_t local_ip_;
  uint16_t port_;
  std::string name_;
  OSThread dns_thread_;
  bool shutdown_{false};
  bool shutdownComplete_{false};

  struct DNSHeader
  {
    uint16_t id;           // identification number
    union {
      struct {
        uint16_t rd  : 1;  // recursion desired
        uint16_t tc  : 1;  // truncated message
        uint16_t aa  : 1;  // authoritive answer
        uint16_t opc : 4;  // message_type
        uint16_t qr  : 1;  // query/response flag
        uint16_t rc  : 4;  // response code
        uint16_t z   : 3;  // its z! reserved
        uint16_t ra  : 1;  // recursion available
      };
      uint16_t flags;
    };
    uint16_t questions;    // number of question entries
    uint16_t answers;      // number of answer entries
    uint16_t authorties;   // number of authority entries
    uint16_t resources;    // number of resource entries
  } __attribute__((packed));

  struct DNSResponse
  {
    uint16_t id;
    uint16_t answer;
    uint16_t classes;
    uint32_t ttl;
    uint16_t length;
    uint32_t address;
  } __attribute__((packed));

  static constexpr uint32_t DNS_TASK_STACK_SIZE = 3*1024;
  static constexpr uint16_t DNS_REQ_BUFFER_SIZE = 512;
};


#endif // DNS_SERVER_H_
