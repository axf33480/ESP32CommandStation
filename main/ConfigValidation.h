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

#ifndef _CONFIG_VALIDATION_H_
#define _CONFIG_VALIDATION_H_

#include "sdkconfig.h"

///////////////////////////////////////////////////////////////////////////////
// Nextion interface configuration validations
///////////////////////////////////////////////////////////////////////////////
#if CONFIG_NEXTION
  #if CONFIG_NEXTION_RX_PIN == CONFIG_NEXTION_TX_PIN
  #error "Invalid Configuration detected, CONFIG_NEXTION_RX_PIN and " \
         "CONFIG_NEXTION_TX_PIN must be unique."
  #endif
  #if CONFIG_STATUS_LED && CONFIG_STATUS_LED_DATA_PIN == CONFIG_NEXTION_RX_PIN
  #error "Invalid Configuration detected, CONFIG_STATUS_LED_DATA_PIN and " \
         "CONFIG_NEXTION_RX_PIN must be unique."
  #endif
  #if CONFIG_STATUS_LED && CONFIG_STATUS_LED_DATA_PIN == CONFIG_NEXTION_TX_PIN
  #error "Invalid Configuration detected, CONFIG_STATUS_LED_DATA_PIN and " \
         "CONFIG_NEXTION_TX_PIN must be unique."
  #endif
  #if CONFIG_HC12
    #if CONFIG_NEXTION_RX_PIN == CONFIG_HC12_RX_PIN
    #error "Invalid Configuration detected, the Nextion and HC12 can not " \
           "share the same RX Pin."
    #endif
    #if CONFIG_NEXTION_TX_PIN == CONFIG_HC12_TX_PIN
    #error "Invalid Configuration detected, the Nextion and HC12 can not " \
           "share the same TX Pin."
    #endif
  #endif
  #if CONFIG_LOCONET
    #if CONFIG_NEXTION_RX_PIN == CONFIG_LOCONET_RX_PIN
    #error "Invalid Configuration detected, the Nextion and LocoNet can not " \
           "share the same RX Pin."
    #endif
    #if CONFIG_NEXTION_TX_PIN == CONFIG_LOCONET_TX_PIN
    #error "Invalid Configuration detected, the Nextion and LocoNet can not " \
           "share the same TX Pin."
    #endif
  #endif
  #if CONFIG_S88
    #if CONFIG_S88_CLOCK_PIN == CONFIG_NEXTION_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and " \
           "CONFIG_S88_CLOCK_PIN must be unique."
    #endif
    #if CONFIG_S88_CLOCK_PIN == CONFIG_NEXTION_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and " \
           "CONFIG_S88_CLOCK_PIN must be unique."
    #endif
    #if CONFIG_S88_RESET_PIN == CONFIG_NEXTION_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and " \
           "CONFIG_S88_RESET_PIN must be unique."
    #endif
    #if CONFIG_S88_RESET_PIN == CONFIG_NEXTION_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and " \
           "CONFIG_S88_RESET_PIN must be unique."
    #endif
    #if CONFIG_S88_LOAD_PIN == CONFIG_NEXTION_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and " \
           "CONFIG_S88_LOAD_PIN must be unique."
    #endif
    #if CONFIG_S88_LOAD_PIN == CONFIG_NEXTION_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and " \
           "CONFIG_S88_LOAD_PIN must be unique."
    #endif
  #endif
#endif

///////////////////////////////////////////////////////////////////////////////
// HC12 Radio interface configuration validations
///////////////////////////////////////////////////////////////////////////////
#if CONFIG_HC12
  #if CONFIG_HC12_RX_PIN == CONFIG_HC12_TX_PIN
  #error "Invalid Configuration detected, CONFIG_HC12_RX_PIN and " \
         "CONFIG_HC12_TX_PIN must be unique."
  #endif
  #if CONFIG_STATUS_LED && CONFIG_STATUS_LED_DATA_PIN == CONFIG_HC12_RX_PIN
  #error "Invalid Configuration detected, CONFIG_STATUS_LED_DATA_PIN and " \
         "NEXTION_TX_PIN must be unique."
  #endif
  #if CONFIG_STATUS_LED && CONFIG_STATUS_LED_DATA_PIN == CONFIG_HC12_TX_PIN
  #error "Invalid Configuration detected, CONFIG_STATUS_LED_DATA_PIN and " \
         "CONFIG_HC12_RX_PIN must be unique."
  #endif
  #if CONFIG_S88
    #if CONFIG_S88_CLOCK_PIN == CONFIG_HC12_RX_PIN
    #error "Invalid Configuration detected, the CONFIG_HC12_RX_PIN and " \
           "CONFIG_S88_CLOCK_PIN must be unique."
    #endif
    #if CONFIG_S88_CLOCK_PIN == CONFIG_HC12_TX_PIN
    #error "Invalid Configuration detected, the CONFIG_HC12_TX_PIN and " \
           "CONFIG_S88_CLOCK_PIN must be unique."
    #endif
    #if CONFIG_S88_RESET_PIN == CONFIG_HC12_RX_PIN
    #error "Invalid Configuration detected, the CONFIG_HC12_RX_PIN and " \
           "CONFIG_S88_RESET_PIN must be unique."
    #endif
    #if CONFIG_S88_RESET_PIN == CONFIG_HC12_TX_PIN
    #error "Invalid Configuration detected, the CONFIG_HC12_TX_PIN and " \
           "CONFIG_S88_RESET_PIN must be unique."
    #endif
    #if CONFIG_S88_LOAD_PIN == CONFIG_HC12_RX_PIN
    #error "Invalid Configuration detected, the CONFIG_HC12_RX_PIN and " \
           "S88_LOAD_PIN must be unique."
    #endif
    #if CONFIG_S88_LOAD_PIN == CONFIG_HC12_TX_PIN
    #error "Invalid Configuration detected, the CONFIG_HC12_TX_PIN and " \
           "CONFIG_S88_LOAD_PIN must be unique."
    #endif
  #endif
  #if CONFIG_LOCONET
    #if CONFIG_LOCONET_RX_PIN == CONFIG_HC12_RX_PIN
    #error "Invalid Configuration detected, the CONFIG_LOCONET_RX_PIN and " \
           "CONFIG_HC12_RX_PIN must be unique."
    #endif
    #if CONFIG_LOCONET_TX_PIN == CONFIG_HC12_TX_PIN
    #error "Invalid Configuration detected, the CONFIG_LOCONET_TX_PIN and " \
           "CONFIG_HC12_TX_PIN must be unique."
    #endif
  #endif
#endif

///////////////////////////////////////////////////////////////////////////////
// LocoNet interface configuration validations
///////////////////////////////////////////////////////////////////////////////
#if CONFIG_LOCONET
  #if CONFIG_STATUS_LED && CONFIG_STATUS_LED_DATA_PIN == CONFIG_LOCONET_RX_PIN
  #error "Invalid Configuration detected, CONFIG_STATUS_LED_DATA_PIN and " \
         "CONFIG_LOCONET_RX_PIN must be unique."
  #endif
  #if CONFIG_STATUS_LED && CONFIG_STATUS_LED_DATA_PIN == CONFIG_LOCONET_TX_PIN
  #error "Invalid Configuration detected, CONFIG_STATUS_LED_DATA_PIN and " \
         "CONFIG_LOCONET_TX_PIN must be unique."
  #endif
  #if CONFIG_S88
    #if CONFIG_S88_CLOCK_PIN == CONFIG_LOCONET_RX_PIN
    #error "Invalid Configuration detected, the CONFIG_LOCONET_RX_PIN and " \
           "CONFIG_S88_CLOCK_PIN must be unique."
    #endif
    #if CONFIG_S88_CLOCK_PIN == CONFIG_LOCONET_TX_PIN
    #error "Invalid Configuration detected, the CONFIG_LOCONET_TX_PIN and " \
           "CONFIG_S88_CLOCK_PIN must be unique."
    #endif
    #if CONFIG_S88_RESET_PIN == CONFIG_LOCONET_RX_PIN
    #error "Invalid Configuration detected, the CONFIG_LOCONET_RX_PIN and " \
           "CONFIG_S88_RESET_PIN must be unique."
    #endif
    #if CONFIG_S88_RESET_PIN == CONFIG_LOCONET_TX_PIN
    #error "Invalid Configuration detected, the CONFIG_LOCONET_TX_PIN and " \
           "CONFIG_S88_RESET_PIN must be unique."
    #endif
    #if CONFIG_S88_LOAD_PIN == CONFIG_LOCONET_RX_PIN
    #error "Invalid Configuration detected, the CONFIG_LOCONET_RX_PIN and " \
           "CONFIG_S88_LOAD_PIN must be unique."
    #endif
    #if CONFIG_S88_LOAD_PIN == CONFIG_LOCONET_TX_PIN
    #error "Invalid Configuration detected, the CONFIG_LOCONET_TX_PIN and " \
           "CONFIG_S88_LOAD_PIN must be unique."
    #endif
  #endif
#endif

///////////////////////////////////////////////////////////////////////////////
// LCC interface configuration validations
///////////////////////////////////////////////////////////////////////////////
#if !defined(CONFIG_LCC_NODE_ID)
#error Invalid Configuration detected, LCC Node ID must be specified.
#endif

#if CONFIG_LCC_CAN_ENABLED
  #if CONFIG_STATUS_LED && CONFIG_STATUS_LED_DATA_PIN == CONFIG_LCC_CAN_RX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
         "LCC_CAN_RX_PIN must be unique."
  #endif
  #if CONFIG_S88
    #if CONFIG_S88_CLOCK_PIN == CONFIG_LCC_CAN_RX_PIN
    #error "Invalid Configuration detected, CONFIG_LCC_CAN_RX_PIN and " \
           "CONFIG_S88_CLOCK_PIN must be unique."
    #endif
    #if CONFIG_S88_LOAD_PIN == CONFIG_LCC_CAN_RX_PIN
    #error "Invalid Configuration detected, CONFIG_LCC_CAN_RX_PIN and " \
           "CONFIG_S88_LOAD_PIN must be unique."
    #endif
    #if CONFIG_S88_RESET_PIN == CONFIG_LCC_CAN_RX_PIN
    #error "Invalid Configuration detected, CONFIG_LCC_CAN_RX_PIN and " \
           "CONFIG_S88_RESET_PIN must be unique."
    #endif
  #endif
  #if CONFIG_LOCONET
    #if CONFIG_LCC_CAN_RX_PIN == CONFIG_LOCONET_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
           "CONFIG_LOCONET_RX_PIN must be unique."
    #endif
    #if CONFIG_LCC_CAN_RX_PIN == CONFIG_LOCONET_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
           "CONFIG_LOCONET_TX_PIN must be unique."
    #endif
  #endif
  #if CONFIG_NEXTION
    #if CONFIG_LCC_CAN_RX_PIN == NEXTION_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
           "NEXTION_RX_PIN must be unique."
    #endif
    #if CONFIG_LCC_CAN_RX_PIN == NEXTION_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
           "NEXTION_TX_PIN must be unique."
    #endif
  #endif
  #if CONFIG_STATUS_LED && CONFIG_STATUS_LED_DATA_PIN == CONFIG_LCC_CAN_TX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
         "LCC_CAN_TX_PIN must be unique."
  #endif
  #if CONFIG_S88
    #if CONFIG_S88_CLOCK_PIN == CONFIG_LCC_CAN_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "CONFIG_S88_CLOCK_PIN must be unique."
    #endif
    #if CONFIG_S88_LOAD_PIN == CONFIG_LCC_CAN_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "CONFIG_S88_LOAD_PIN must be unique."
    #endif
    #if CONFIG_S88_RESET_PIN == CONFIG_LCC_CAN_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "CONFIG_S88_RESET_PIN must be unique."
    #endif
  #endif
  #if CONFIG_LOCONET
    #if CONFIG_LCC_CAN_TX_PIN == CONFIG_LOCONET_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "CONFIG_LOCONET_RX_PIN must be unique."
    #endif
    #if CONFIG_LCC_CAN_TX_PIN == CONFIG_LOCONET_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "CONFIG_LOCONET_TX_PIN must be unique."
    #endif
  #endif
  #if CONFIG_NEXTION
    #if CONFIG_LCC_CAN_TX_PIN == NEXTION_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "NEXTION_RX_PIN must be unique."
    #endif
    #if CONFIG_LCC_CAN_TX_PIN == NEXTION_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "NEXTION_TX_PIN must be unique."
    #endif
  #endif
  #if CONFIG_LCC_CAN_RX_PIN == CONFIG_LCC_CAN_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
           "LCC_CAN_TX_PIN must be unique."
  #endif
#endif

#endif // _CONFIG_VALIDATION_H_