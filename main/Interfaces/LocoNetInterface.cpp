/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2020 Mike Dunston

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

#if LOCONET_ENABLED
LocoNetESP32Uart locoNet(LOCONET_RX_PIN, LOCONET_TX_PIN, LOCONET_UART, LOCONET_INVERTED_LOGIC, LOCONET_ENABLE_RX_PIN_PULLUP);

// TODO: consider bridge code to wake up an executable already executing inside
// the stack rather than create on the fly.
#define GET_LOCO_VIA_EXECUTOR(NAME, address)                                          \
  TrainImpl *NAME = nullptr;                                                          \
  {                                                                                   \
    SyncNotifiable n;                                                                 \
    extern unique_ptr<OpenMRN> openmrn;                                               \
    openmrn->stack()->executor()->add(new CallbackExecutable(                         \
    [&]()                                                                             \
    {                                                                                 \
      NAME = Singleton<AllTrainNodes>::instance()->get_train_impl(commandstation::DccMode::DCC_128_LONG_ADDRESS \
                                      , address);                                     \
      n.notify();                                                                     \
    }));                                                                              \
    n.wait_for_notification();                                                        \
  }

// temporary solution for persistance of the locomotive address to slot
// mapping. Each slot can hold one locomotive at a time, this will require
// additional maintenance for the purging of slots etc.
// TODO: add locking to protect this vector.
vector<uint16_t> slotMap;

void initializeLocoNet()
{
  Singleton<StatusDisplay>::instance()->status("LocoNet Init");
  locoNet.begin();
  locoNet.onPacket(OPC_GPON, [](lnMsg *msg)
  {
    LOG(INFO, "[LocoNet] Requesting Track Power ON");
    trackSignal->enable_ops_output();
  });
  locoNet.onPacket(OPC_GPOFF, [](lnMsg *msg)
  {
    LOG(INFO, "[LocoNet] Requesting Track Power OFF");
    trackSignal->disable_ops_output();
  });
  locoNet.onPacket(OPC_IDLE, [](lnMsg *msg)
  {
    LOG(INFO, "[LocoNet] Requesting eStop!");
    Singleton<esp32cs::EStopHandler>::instance()->set_state(true);
  });
  locoNet.onPacket(OPC_LOCO_ADR, [](lnMsg *msg)
  {
    lnMsg response = {0};
    uint16_t locoAddress = (msg->la.adr_lo + (msg->la.adr_hi << 7));
    // TODO: add locking
    uint8_t slot = slotMap.size() + 1;
    slotMap[slot - 1] = locoAddress;
    GET_LOCO_VIA_EXECUTOR(loco, locoAddress);
    response.sd.command = OPC_SL_RD_DATA;
    response.sd.mesg_size = 0x0E;
    response.sd.slot = slot;
    response.sd.stat = LOCO_IDLE | DEC_MODE_128;
    response.sd.adr = msg->la.adr_lo;
    response.sd.adr2 = msg->la.adr_hi;
    response.sd.dirf = DIRF_F0;
    response.sd.trk = GTRK_MLOK1;
    if(trackSignal->is_enabled())
    {
      response.sd.trk |= GTRK_POWER;
    }
    // TODO: add prog track status
    /*
    if(progTrackBusy) {
      response.sd.trk |= GTRK_PROG_BUSY;
    }
    */
    locoNet.send(&response);
  });
  locoNet.onPacket(OPC_LOCO_SPD, [](lnMsg *msg)
  {
    GET_LOCO_VIA_EXECUTOR(loco, slotMap[msg->lsp.slot - 1]);
    if(loco)
    {
      auto speed = loco->get_speed();
      speed.set_dcc_128(msg->lsp.spd);
      loco->set_speed(speed);
    }
    else
    {
      locoNet.send(OPC_LONG_ACK, OPC_LOCO_SPD, 0);
    }
  });
  locoNet.onPacket(OPC_LOCO_DIRF, [](lnMsg *msg)
  {
    GET_LOCO_VIA_EXECUTOR(loco, slotMap[msg->ldf.slot - 1]);
    if(loco)
    {
      auto speed = loco->get_speed();
      speed.set_direction(msg->ldf.dirf & DIRF_DIR);
      loco->set_speed(speed);
      loco->set_fn(0, msg->ldf.dirf & DIRF_F0);
      loco->set_fn(1, msg->ldf.dirf & DIRF_F1);
      loco->set_fn(2, msg->ldf.dirf & DIRF_F2);
      loco->set_fn(3, msg->ldf.dirf & DIRF_F3);
      loco->set_fn(4, msg->ldf.dirf & DIRF_F4);
    }
    else
    {
      locoNet.send(OPC_LONG_ACK, OPC_LOCO_DIRF, 0);
    }
  });
  locoNet.onPacket(OPC_LOCO_SND, [](lnMsg *msg)
  {
    GET_LOCO_VIA_EXECUTOR(loco, slotMap[msg->ls.slot - 1]);
    if(loco)
    {
      loco->set_fn(5, msg->ls.snd & SND_F5);
      loco->set_fn(6, msg->ls.snd & SND_F6);
      loco->set_fn(7, msg->ls.snd & SND_F7);
      loco->set_fn(8, msg->ls.snd & SND_F8);
    }
    else
    {
      locoNet.send(OPC_LONG_ACK, OPC_LOCO_SND, 0);
    }
  });
  // TOOD: reimplement prog track interface
  /*
  locoNet.onPacket(OPC_WR_SL_DATA, [](lnMsg *msg) {
    if(msg->pt.slot == PRG_SLOT) {
      if(msg->pt.command == 0x00) {
        // Cancel / abort request, currently ignored
      } else if (progTrackBusy) {
        locoNet.send(OPC_LONG_ACK, OPC_MASK, 0);
      } else {
        uint16_t cv = PROG_CV_NUM(msg->pt);
        uint8_t value = PROG_DATA(msg->pt);
        if((msg->pt.command & DIR_BYTE_ON_SRVC_TRK) == 0 &&
          (msg->pt.command & PCMD_RW) == 1) { // CV Write on PROG
          if(enterProgrammingMode()) {
            locoNet.send(OPC_LONG_ACK, OPC_MASK, 1);
            msg->pt.command = OPC_SL_RD_DATA;
            if(!writeProgCVByte(cv, value)) {
              msg->pt.pstat = PSTAT_WRITE_FAIL;
            } else {
              msg->pt.data7 = value;
              if(value & 0x80) {
                msg->pt.cvh |= CVH_D7;
              }
            }
            leaveProgrammingMode();
            locoNet.send(msg);
          } else {
            locoNet.send(OPC_LONG_ACK, OPC_MASK, 0);
          }
        } else if((msg->pt.command & DIR_BYTE_ON_SRVC_TRK) == 0 &&
          (msg->pt.command & PCMD_RW) == 0) { // CV Read on PROG
          if(enterProgrammingMode()) {
            locoNet.send(OPC_LONG_ACK, OPC_MASK, 1);
            msg->pt.command = OPC_SL_RD_DATA;
            int16_t value = readCV(cv);
            if(value == -1) {
              msg->pt.pstat = PSTAT_READ_FAIL;
            } else {
              msg->pt.data7 = value & 0x7F;
              if(value & 0x80) {
                msg->pt.cvh |= CVH_D7;
              }
            }
            leaveProgrammingMode();
            locoNet.send(msg);
          } else {
            locoNet.send(OPC_LONG_ACK, OPC_MASK, 0);
          }
        } else if ((msg->pt.command & OPS_BYTE_NO_FEEDBACK) == 0) {
          // CV Write on OPS, no feedback
          locoNet.send(OPC_LONG_ACK, OPC_MASK, 0x40);
          uint16_t locoAddr = ((msg->pt.hopsa & 0x7F) << 7) + (msg->pt.lopsa & 0x7F);
          writeOpsCVByte(locoAddr, cv, value);
        } else if ((msg->pt.command & OPS_BYTE_FEEDBACK) == 0) {
          // CV Write on OPS
          locoNet.send(OPC_LONG_ACK, OPC_MASK, 1);
          uint16_t locoAddr = ((msg->pt.hopsa & 0x7F) << 7) + (msg->pt.lopsa & 0x7F);
          writeOpsCVByte(locoAddr, cv, value);
          msg->pt.command = OPC_SL_RD_DATA;
          if(value & 0x80) {
            msg->pt.cvh |= CVH_D7;
          }
          msg->pt.data7 = value & 0x7F;
          locoNet.send(msg);
        } else {
          // not implemented
          locoNet.send(OPC_LONG_ACK, OPC_MASK, OPC_MASK);
        }
      }
    }
  });
  */
  locoNet.onPacket(OPC_SW_REQ, [](lnMsg *msg)
  {
    // handle only the activate output packet, the deactivate packet will be
    // discarded as a duplicate of the activate since this is converting the
    // request to a DCC packet
    if (msg->srq.sw2 & OPC_SW_REQ_OUT)
    {
      uint16_t address = (msg->srq.sw1 | ((msg->srq.sw2 & 0x0F) << 7)) + 1;
      bool state = (msg->srq.sw2 & OPC_SW_REQ_DIR) == OPC_SW_REQ_DIR;
      turnoutManager->setByAddress(address, state);
    }
  });
  locoNet.onPacket(OPC_INPUT_REP, [](lnMsg *msg)
  {
    uint16_t address = ((msg->ir.in1 | ((msg->ir.in2 & 0x0F) << 7)) << 1) +
      msg->ir.in2 & OPC_INPUT_REP_SW ? 2 : 1;
    LOG(INFO, "[LocoNet] Sensor (%d): %d", address
      , msg->ir.in2 & OPC_INPUT_REP_HI);
    // TODO: add a LocoNet sensor type to record this state transition
  });
  locoNet.onPacket(OPC_SW_REP, [](lnMsg *msg)
  {
    uint16_t address = (msg->srp.sn1 | ((msg->srp.sn2 & 0x0F) << 7)) + 1;
    LOG(INFO, "[LocoNet] Switch status report: %d : %d", address
      , msg->srp.sn2 & OPC_SW_REP_THROWN);
    if (msg->srp.sn2 & OPC_SW_REP_INPUTS)
    {
      // TODO: Aux input reporting
    }
    else
    {
      bool state = (msg->srq.sw2 & OPC_SW_REP_THROWN) == OPC_SW_REP_THROWN;
      // update the current state of the turnout but do not send a DCC packet
      turnoutManager->setByAddress(address, state, false);
    }
  });
}

#endif