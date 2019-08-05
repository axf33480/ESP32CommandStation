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

#ifndef TURNOUTS_H_
#define TURNOUTS_H_

#include <dcc/PacketSource.hxx>
#include <openlcb/DccAccyConsumer.hxx>
#include "interfaces/DCCppProtocol.h"

enum TurnoutType
{
  LEFT=0,
  RIGHT,
  WYE,
  MULTI,
  MAX_TURNOUT_TYPES // NOTE: this must be the last entry in the enum.
};

void encodeDCCAccessoryAddress(uint16_t *boardAddress, int8_t *boardIndex, uint16_t address);
uint16_t decodeDCCAccessoryAddress(uint16_t boardAddress, int8_t boardIndex);

class Turnout : public dcc::NonTrainPacketSource
{
public:
  Turnout(uint16_t, uint16_t, int8_t, bool=false, TurnoutType=TurnoutType::LEFT);
  Turnout(JsonObject);
  virtual ~Turnout() {}
  void update(uint16_t, int8_t, TurnoutType);
  void set(bool=false, bool=true);
  void toJson(JsonObject, bool=false);
  uint16_t getID()
  {
    return _turnoutID;
  }
  uint16_t getAddress()
  {
    return _address;
  }
  uint16_t getBoardAddress()
  {
    return _boardAddress;
  }
  uint8_t getIndex()
  {
    return _index;
  }
  bool isThrown()
  {
    return _thrown;
  }
  void toggle()
  {
    set(!_thrown);
  }
  void showStatus();
  TurnoutType getType()
  {
    return _type;
  }
  void setType(const TurnoutType type)
  {
    _type = type;
  }
  void get_next_packet(unsigned code, dcc::Packet* packet) override;
private:
  uint16_t _turnoutID;
  uint16_t _address;
  int8_t _index;
  uint16_t _boardAddress;
  bool _thrown;
  TurnoutType _type;
};

class TurnoutManager : public PacketFlowInterface, private Atomic
{
public:
  TurnoutManager(openlcb::Node *);
  void clear();
  uint16_t store();
  bool setByID(uint16_t, bool=false, bool=true);
  void setByAddress(uint16_t, bool=false, bool=true);
  bool toggleByID(uint16_t);
  void toggleByAddress(uint16_t);
  void getState(JsonArray, bool=true);
  void showStatus();
  Turnout *createOrUpdate(const uint16_t, const uint16_t, const int8_t, const TurnoutType=TurnoutType::LEFT);
  bool removeByID(const uint16_t);
  bool removeByAddress(const uint16_t);
  Turnout *getTurnoutByIndex(const uint16_t);
  Turnout *getTurnoutByID(const uint16_t);
  Turnout *getTurnoutByAddress(const uint16_t);
  uint16_t getTurnoutCount();
  void send(Buffer<dcc::Packet> *, unsigned);
private:
  std::vector<std::unique_ptr<Turnout>> turnouts_;
  std::unique_ptr<openlcb::DccAccyConsumer> turnoutEventConsumer_;
};

extern std::unique_ptr<TurnoutManager> turnoutManager;

/*
  <T ID BOARD INDEX>:          creates a new turnout ID, with specified BOARD
                               and INDEX. If turnout ID already exists, it is
                               updated with specificed BOARD and INDEX
      returns: <O> if successful and <X> if unsuccessful (ie: out of memory)

  <T ID>:                      deletes definition of turnout ID
      returns: <O> if successful and <X> if unsuccessful (ie: ID does not exist)

  <T>:                         lists all defined turnouts
      returns: <H ID ADDRESS SUBADDRESS THROW> for each defined turnout or <X>
               if no turnouts defined
  <T ID THROW>:                sets turnout ID to either the "thrown" or
                               "unthrown" position
      returns: <H ID THROW>, or <X> if turnout ID does not exist
where
  ID:         the numeric ID (0-32767) of the turnout to control
  BOARD:      the primary address of the decoder controlling this turnout
              (0-511)
  INDEX:      the subaddress of the decoder controlling this turnout (0-3)
  THROW:      0 (unthrown) or 1 (thrown)
*/
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(TurnoutCommandAdapter, "T")

/*
  <Tex ID>:              Toggle turnout by ID.
  <Tex ID ADDRESS TYPE>: Creates a Turnout by DCC address
  <Tex ADDRESS TYPE>:    Create a Turnout by DCC address with automatic
                         assignment of ID
  all will return : <O> if successful and <X> if unsuccessful.

where
  ID:         the numeric ID (0-32767) of the turnout to control
  ADDRESS:    the DCC decoder address for the turnout
  TYPE:       turnout type:
              0 : LEFT
              1 : RIGHT
              2 : WYE
              3 : MULTI
*/
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(TurnoutExCommandAdapter, "Tex")

/*
 <a BOARD INDEX THROW>: Throws a turnout (accessory decoder)
      returns: <H ID THROW>

Note: When this is received a Turnout will be created based on the decoded
DCC address for the accessory decoder.
where
  BOARD:      the primary address of the decoder controlling this turnout
              (0-511)
  INDEX:      the subaddress of the decoder controlling this turnout (0-3)
  THROW:      0 (unthrown) or 1 (thrown)
*/
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(AccessoryCommand, "a")

#endif // TURNOUTS_H_