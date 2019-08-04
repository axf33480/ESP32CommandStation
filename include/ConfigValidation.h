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

#ifndef _CONFIG_VALIDATION_H_
#define _CONFIG_VALIDATION_H_

///////////////////////////////////////////////////////////////////////////////
// Ensure SSID and PASSWORD are provided.
///////////////////////////////////////////////////////////////////////////////
#if !defined(SSID_NAME) || !defined(SSID_PASSWORD)
#error "Invalid Configuration detected, SSID_NAME or SSID_PASSWORD in " \
       "Config_WiFi.h has not been defined properly!"
#endif

///////////////////////////////////////////////////////////////////////////////
// Ensure the required h-bridge parameters are specified and not overlapping.
///////////////////////////////////////////////////////////////////////////////

#if !defined(OPS_ENABLE_PIN) || \
    !defined(OPS_THERMAL_PIN) || \
    !defined(OPS_CURRENT_SENSE_ADC) || \
    !defined(OPS_HBRIDGE_TYPE) || \
    !defined(PROG_ENABLE_PIN) || \
    !defined(PROG_CURRENT_SENSE_ADC) || \
    !defined(PROG_HBRIDGE_TYPE) || \
    !defined(OPS_SIGNAL_PIN) || \
    !defined(PROG_SIGNAL_PIN)
#error "Invalid Configuration detected, one (or more) entries in " \
       "Config_HBridge.h has not been defined properly."
#endif

#if OPS_PREAMBLE_BITS < 11
#error "Invalid Configuration detected, OPS_PREAMBLE_BITS is too low, a " \
       "minimum of 11 bits must be transmitted for the DCC for the DCC " \
       "decoder to accept the packets. Adjust this in Config_HBridge.h."
#endif

#if OPS_PREAMBLE_BITS > 20
#error "Invalid Configuration detected, OPS_PREAMBLE_BITS is too high!." \
       "The OPS track output supports only up to 20 preamble bits. " \
       "Adjust this in Config_HBridge.h."
#endif

#if PROG_PREAMBLE_BITS < 22
#error "Invalid Configuration detected, PROG_PREAMBLE_BITS is too low, a " \
       "minimum of 22 bits must be transmitted for reliability on the PROG " \
       "track. Adjust this in Config_HBridge.h."
#endif

#if PROG_PREAMBLE_BITS > 75
#error "Invalid Configuration detected, PROG_PREAMBLE_BITS is too high!." \
       "The PROG track output supports only up to 75 preamble bits. " \
       "Adjust this in Config_HBridge.h."
#endif

#if OPS_ENABLE_PIN == PROG_ENABLE_PIN
#error "Invalid Configuration detected, OPS_ENABLE_PIN and PROG_ENABLE_PIN " \
       "must be unique."
#endif

#if OPS_SIGNAL_PIN == PROG_SIGNAL_PIN
#error "Invalid Configuration detected, OPS_SIGNAL_PIN and PROG_SIGNAL_PIN " \
       "must be unique."
#endif

#if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == OPS_ENABLE_PIN
#error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
       "OPS_ENABLE_PIN must be unique."
#endif

#if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == PROG_ENABLE_PIN
#error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
       "PROG_ENABLE_PIN must be unique."
#endif

#if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == OPS_SIGNAL_PIN
#error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
       "OPS_SIGNAL_PIN must be unique."
#endif

#if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == PROG_SIGNAL_PIN
#error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
       "PROG_SIGNAL_PIN must be unique."
#endif

///////////////////////////////////////////////////////////////////////////////
// Ensure either OLED or LCD display is active and not both.
///////////////////////////////////////////////////////////////////////////////
#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED && \
    defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD
#error "Invalid Configuration detected, it is not supported to include both " \
       "OLED and LCD support."
#endif

///////////////////////////////////////////////////////////////////////////////
// Nextion interface configuration validations
///////////////////////////////////////////////////////////////////////////////
#if NEXTION_ENABLED
  #if NEXTION_UART_RX_PIN == NEXTION_UART_TX_PIN
  #error "Invalid Configuration detected, NEXTION_UART_RX_PIN and " \
         "NEXTION_UART_TX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == NEXTION_UART_RX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
         "NEXTION_UART_RX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == NEXTION_UART_TX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
         "NEXTION_UART_TX_PIN must be unique."
  #endif
  #if HC12_RADIO_ENABLED
    #if NEXTION_UART_NUM == HC12_UART_NUM
    #error "Invalid Configuration detected, the Nextion and HC12 can not " \
           "share the UART interface."
    #endif
    #if NEXTION_UART_RX_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the Nextion and HC12 can not " \
           "share the same RX Pin."
    #endif
    #if NEXTION_UART_TX_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the Nextion and HC12 can not " \
           "share the same TX Pin."
    #endif
  #endif
  #if LOCONET_ENABLED
    #if NEXTION_UART_NUM == LOCONET_UART
    #error "Invalid Configuration detected, the Nextion and LocoNet can not " \
           "share the UART interface."
    #endif
    #if NEXTION_UART_RX_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the Nextion and LocoNet can not " \
           "share the same RX Pin."
    #endif
    #if NEXTION_UART_TX_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the Nextion and LocoNet can not " \
           "share the same TX Pin."
    #endif
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == NEXTION_UART_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and " \
           "S88_CLOCK_PIN must be unique."
    #endif
    #if S88_CLOCK_PIN == NEXTION_UART_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and " \
           "S88_CLOCK_PIN must be unique."
    #endif
    #if S88_RESET_PIN == NEXTION_UART_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and " \
           "S88_RESET_PIN must be unique."
    #endif
    #if S88_RESET_PIN == NEXTION_UART_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and " \
           "S88_RESET_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == NEXTION_UART_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and " \
           "S88_LOAD_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == NEXTION_UART_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and " \
           "S88_LOAD_PIN must be unique."
    #endif
  #endif
#endif

///////////////////////////////////////////////////////////////////////////////
// HC12 Radio interface configuration validations
///////////////////////////////////////////////////////////////////////////////
#if HC12_RADIO_ENABLED
  #if HC12_RX_PIN == HC12_TX_PIN
  #error "Invalid Configuration detected, HC12_RX_PIN and HC12_TX_PIN must " \
         "be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == HC12_RX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
         "NEXTION_TX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == HC12_TX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
         "HC12_TX_PIN must be unique."
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the HC12_RX_PIN and " \
           "S88_CLOCK_PIN must be unique."
    #endif
    #if S88_CLOCK_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the HC12_TX_PIN and " \
           "S88_CLOCK_PIN must be unique."
    #endif
    #if S88_RESET_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the HC12_RX_PIN and " \
           "S88_RESET_PIN must be unique."
    #endif
    #if S88_RESET_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the HC12_TX_PIN and " \
           "S88_RESET_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the HC12_RX_PIN and " \
           "S88_LOAD_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the HC12_TX_PIN and " \
           "S88_LOAD_PIN must be unique."
    #endif
  #endif
  #if LOCONET_ENABLED
    #if LOCONET_UART == HC12_UART_NUM
    #error "Invalid Configuration detected, the LocoNet and HC12 can not " \
           "share the UART interface."
    #endif
    #if LOCONET_RX_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the LOCONET_RX_PIN and " \
           "HC12_RX_PIN must be unique."
    #endif
    #if LOCONET_TX_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the LOCONET_TX_PIN and " \
           "HC12_TX_PIN must be unique."
    #endif
  #endif
#endif

///////////////////////////////////////////////////////////////////////////////
// LocoNet interface configuration validations
///////////////////////////////////////////////////////////////////////////////
#if LOCONET_ENABLED
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == LOCONET_RX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
         "LOCONET_RX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == LOCONET_TX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
         "LOCONET_TX_PIN must be unique."
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the LOCONET_RX_PIN and " \
           "S88_CLOCK_PIN must be unique."
    #endif
    #if S88_CLOCK_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the LOCONET_TX_PIN and " \
           "S88_CLOCK_PIN must be unique."
    #endif
    #if S88_RESET_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the LOCONET_RX_PIN and " \
           "S88_RESET_PIN must be unique."
    #endif
    #if S88_RESET_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the LOCONET_TX_PIN and " \
           "S88_RESET_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the LOCONET_RX_PIN and " \
           "S88_LOAD_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the LOCONET_TX_PIN and " \
           "S88_LOAD_PIN must be unique."
    #endif
  #endif
#endif

///////////////////////////////////////////////////////////////////////////////
// LCC interface configuration validations
///////////////////////////////////////////////////////////////////////////////
#if LCC_CAN_RX_PIN != NOT_A_PIN
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == LCC_CAN_RX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
         "LCC_CAN_RX_PIN must be unique."
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == LCC_CAN_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
           "S88_CLOCK_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == LCC_CAN_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
           "S88_LOAD_PIN must be unique."
    #endif
    #if S88_RESET_PIN == LCC_CAN_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
           "S88_RESET_PIN must be unique."
    #endif
  #endif
  #if LOCONET_ENABLED
    #if LCC_CAN_RX_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
           "LOCONET_RX_PIN must be unique."
    #endif
    #if LCC_CAN_RX_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
           "LOCONET_TX_PIN must be unique."
    #endif
  #endif
  #if NEXTION_ENABLED
    #if LCC_CAN_RX_PIN == NEXTION_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
           "NEXTION_RX_PIN must be unique."
    #endif
    #if LCC_CAN_RX_PIN == NEXTION_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
           "NEXTION_TX_PIN must be unique."
    #endif
  #endif
#endif
#if LCC_CAN_TX_PIN != NOT_A_PIN
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == LCC_CAN_TX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and " \
         "LCC_CAN_TX_PIN must be unique."
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == LCC_CAN_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "S88_CLOCK_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == LCC_CAN_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "S88_LOAD_PIN must be unique."
    #endif
    #if S88_RESET_PIN == LCC_CAN_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "S88_RESET_PIN must be unique."
    #endif
  #endif
  #if LOCONET_ENABLED
    #if LCC_CAN_TX_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "LOCONET_RX_PIN must be unique."
    #endif
    #if LCC_CAN_TX_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "LOCONET_TX_PIN must be unique."
    #endif
  #endif
  #if NEXTION_ENABLED
    #if LCC_CAN_TX_PIN == NEXTION_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "NEXTION_RX_PIN must be unique."
    #endif
    #if LCC_CAN_TX_PIN == NEXTION_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and " \
           "NEXTION_TX_PIN must be unique."
    #endif
  #endif
#endif
#if LCC_CAN_RX_PIN == LCC_CAN_TX_PIN && \
    LCC_CAN_RX_PIN != NOT_A_PIN && \
    LCC_CAN_TX_PIN != NOT_A_PIN
  #error "Invalid Configuration detected, LCC_CAN_RX_PIN and " \
         "LCC_CAN_TX_PIN must be unique."
#endif

#endif // _CONFIG_VALIDATION_H_