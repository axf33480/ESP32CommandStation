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
#include "Turnouts.h"
#include "S88Sensors.h"
#include "RemoteSensors.h"
#include "NextionInterface.h"

const char * buildTime = __DATE__ " " __TIME__;

#ifndef ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS
std::vector<uint8_t> restrictedPins{
  0, // Bootstrap / Firmware Flash Download
  1, // UART0 TX
  2, // Bootstrap / Firmware Flash Download
  3, // UART0 RX
  5, // Bootstrap
  6, 7, 8, 9, 10, 11, // on-chip flash pins
  12, 15 // Bootstrap / SD pins
};
#else
std::vector<uint8_t> restrictedPins;
#endif

#if LOCONET_ENABLED
LocoNetESP32Uart locoNet(LOCONET_RX_PIN, LOCONET_TX_PIN, LOCONET_UART, LOCONET_INVERTED_LOGIC, LOCONET_ENABLE_RX_PIN_PULLUP);
#endif

bool otaComplete = false;
bool otaInProgress = false;

void setup() {

  // Setup UART0 115200 8N1 TX: 1, RX: 3, 2k buffer
  uart_config_t uart0 = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_0, &uart0);
  uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);

  LOG(INFO, "\n\nESP32 Command Station v%s starting up", VERSION);

  // set up ADC1 here since we use it for all motor boards
  adc1_config_width(ADC_WIDTH_BIT_12);

#if NEXTION_ENABLED
  nextionInterfaceInit();
#endif
  configStore = new ConfigurationManager();

  DCCPPProtocolHandler::init();

  wifiInterface.init();
  lccInterface.init();
  MotorBoardManager::registerBoard(MOTORBOARD_CURRENT_SENSE_OPS,
                                   MOTORBOARD_ENABLE_PIN_OPS,
                                   MOTORBOARD_TYPE_OPS,
                                   MOTORBOARD_NAME_OPS);
  MotorBoardManager::registerBoard(MOTORBOARD_CURRENT_SENSE_PROG,
                                   MOTORBOARD_ENABLE_PIN_PROG,
                                   MOTORBOARD_TYPE_PROG,
                                   MOTORBOARD_NAME_PROG,
                                   true);
  dccSignal[DCC_SIGNAL_OPERATIONS] = new SignalGenerator_RMT("OPS", 512, DCC_SIGNAL_OPERATIONS, DCC_SIGNAL_PIN_OPERATIONS, MOTORBOARD_ENABLE_PIN_OPS);
  dccSignal[DCC_SIGNAL_PROGRAMMING] = new SignalGenerator_RMT("PROG", 10, DCC_SIGNAL_PROGRAMMING, DCC_SIGNAL_PIN_PROGRAMMING, MOTORBOARD_ENABLE_PIN_PROG);

  LocomotiveManager::init();
  TurnoutManager::init();

  OutputManager::init();
  SensorManager::init();
#if S88_ENABLED
  S88BusManager::init();
#endif
  RemoteSensorManager::init();
#if LOCONET_ENABLED
  infoScreen.replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "LocoNet Init");
  locoNet.begin();
  locoNet.onPacket(OPC_GPON, [](lnMsg *msg) {
    MotorBoardManager::powerOnAll();
  });
  locoNet.onPacket(OPC_GPOFF, [](lnMsg *msg) {
    MotorBoardManager::powerOffAll();
  });
  locoNet.onPacket(OPC_IDLE, [](lnMsg *msg) {
    LocomotiveManager::emergencyStop();
  });
  locoNet.onPacket(OPC_LOCO_ADR, [](lnMsg *msg) {
    lnMsg response = {0};
    auto loco = LocomotiveManager::getLocomotive(msg->la.adr_lo + (msg->la.adr_hi << 7));
    response.sd.command = OPC_SL_RD_DATA;
    response.sd.mesg_size = 0x0E;
    response.sd.slot = loco->getRegister();
    response.sd.stat = LOCO_IDLE | DEC_MODE_128;
    response.sd.adr = msg->la.adr_lo;
    response.sd.adr2 = msg->la.adr_hi;
    response.sd.dirf = DIRF_F0;
    response.sd.trk = GTRK_MLOK1;
    if(MotorBoardManager::isTrackPowerOn()) {
      response.sd.trk |= GTRK_POWER;
    }
    if(progTrackBusy) {
      response.sd.trk |= GTRK_PROG_BUSY;
    }
    locoNet.send(&response);
  });
  locoNet.onPacket(OPC_LOCO_SPD, [](lnMsg *msg) {
    auto loco = LocomotiveManager::getLocomotiveByRegister(msg->lsp.slot);
    if(loco) {
      loco->setSpeed(msg->lsp.spd);
    } else {
      locoNet.send(OPC_LONG_ACK, OPC_LOCO_SPD, 0);
    }
  });
  locoNet.onPacket(OPC_LOCO_DIRF, [](lnMsg *msg) {
    auto loco = LocomotiveManager::getLocomotiveByRegister(msg->ldf.slot);
    if(loco) {
      loco->setDirection(msg->ldf.dirf & DIRF_DIR);
      loco->setFunction(0, msg->ldf.dirf & DIRF_F0);
      loco->setFunction(1, msg->ldf.dirf & DIRF_F1);
      loco->setFunction(2, msg->ldf.dirf & DIRF_F2);
      loco->setFunction(3, msg->ldf.dirf & DIRF_F3);
      loco->setFunction(4, msg->ldf.dirf & DIRF_F4);
    } else {
      locoNet.send(OPC_LONG_ACK, OPC_LOCO_DIRF, 0);
    }
  });
  locoNet.onPacket(OPC_LOCO_SND, [](lnMsg *msg) {
    auto loco = LocomotiveManager::getLocomotiveByRegister(msg->ls.slot);
    if(loco) {
      loco->setFunction(5, msg->ls.snd & SND_F5);
      loco->setFunction(6, msg->ls.snd & SND_F6);
      loco->setFunction(7, msg->ls.snd & SND_F7);
      loco->setFunction(8, msg->ls.snd & SND_F8);
    } else {
      locoNet.send(OPC_LONG_ACK, OPC_LOCO_SND, 0);
    }
  });
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
  locoNet.onPacket(OPC_INPUT_REP, [](lnMsg *msg) {
    LOG(INFO, "LocoNet INPUT_REPORT %02x : %02x", msg->ir.in1, msg->ir.in2);
  });
  locoNet.onPacket(OPC_SW_REQ, [](lnMsg *msg) {
    LOG(INFO, "LocoNet SW_REQ %02x : %02x", msg->srq.sw1, msg->srq.sw2);
  });
  locoNet.onPacket(OPC_SW_REP, [](lnMsg *msg) {
    LOG(INFO, "LocoNet SW_REP %02x : %02x", msg->srp.sn1, msg->srp.sn2);
  });
#endif

#if ENERGIZE_OPS_TRACK_ON_STARTUP
  MotorBoardManager::powerOnAll();
#else
  MotorBoardManager::powerOffAll();
#endif

  LOG(INFO, "[WatchDog] Reconfiguring Timer (15sec)");
  // reset WDT to 15sec
  esp_task_wdt_init(15, true);

  // Enable watchdog timers
  enableLoopWDT();

  LOG(INFO, "ESP32 Command Station Started!");
  infoScreen.replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "ESP32-CS Started");
}

void loop() {
  lccInterface.update();
  if(otaInProgress && otaComplete) {
    LOG(INFO, "OTA binary has been received, preparing to reboot!");

    // shutdown and cleanup the configuration manager
    delete configStore;

    // wait for the WDT to restart the ESP32
    esp_task_wdt_init(1, true);
    while(true);
  }
  MotorBoardManager::check();
}
