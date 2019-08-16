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

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WHICH PINS ARE USED BY MAIN/PROG MOTOR SHIELDS
//
// CURRENT SENSE PIN MAPPINGS:
// ADC1_CHANNEL_0 : 36
// ADC1_CHANNEL_1 : 37 -- NOT USABLE
// ADC1_CHANNEL_2 : 38 -- NOT USABLE
// ADC1_CHANNEL_3 : 39
// ADC1_CHANNEL_4 : 32
// ADC1_CHANNEL_5 : 33
// ADC1_CHANNEL_6 : 34
// ADC1_CHANNEL_7 : 35
//
// NOTE: GPIO 37 and GPIO 38 are not usable as they are connected to GPIO 36 and
// GPIO 39 internally with a capacitor. Therefore ADC1_CHANNEL_1 and ADC1_CHANNEL_2
// are not suitable for usage, regardless of if the ESP32 board exposes these pins.
//
// SUPPORTED MOTORBOARD TYPES:
// L298           : Arduino Motor shield Rev3 based on the L298 chip. Max Output 2A per channel https://store.arduino.cc/usa/arduino-motor-shield-rev3
// LMD18200       : Texas Instruments LMD18200 55V 3A h-bridge. http://www.ti.com/lit/ds/symlink/lmd18200.pdf
// POLOLU         : Pololu MC33926 Motor Driver (shield or carrier). Max Output 2.5A per channel https://www.pololu.com/product/1213 / https://www.pololu.com/product/2503
// BTS7960B_5A    : Infineon Technologies BTS 7960 Motor Driver Module. Max Output 5A (43A actual max) https://www.infineon.com/dgdl/bts7960b-pb-final.pdf
// BTS7960B_10A   : Infineon Technologies BTS 7960 Motor Driver Module. Max Output 10A (43A actual max) https://www.infineon.com/dgdl/bts7960b-pb-final.pdf

// MAIN TRACK NOTORBOARD ENABLED PIN
#define OPS_ENABLE_PIN 25
// MAIN TRACK H-Bridge Thermal Warning Pin
#define OPS_THERMAL_PIN -1
// MAIN TRACK MOTORBOARD CURRENT SENSE ADC PIN
#define OPS_CURRENT_SENSE_ADC ADC1_CHANNEL_0
// MAIN TRACK MOTORBOARD MOTOR_BOARD_TYPE
#define OPS_HBRIDGE_TYPE L298

// PROG TRACK NOTORBOARD ENABLED PIN
#define PROG_ENABLE_PIN 23
// PROG TRACK MOTORBOARD CURRENT SENSE ADC PIN
#define PROG_CURRENT_SENSE_ADC ADC1_CHANNEL_3
// PROG TRACK MOTORBOARD MOTOR_BOARD_TYPE
#define PROG_HBRIDGE_TYPE L298

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WHICH PINS ARE USED FOR DCC SIGNAL GENERATION
//
// OPERATIONS TRACK DCC SIGNAL PIN
#define OPS_SIGNAL_PIN 19

// PROGRAMMING TRACK DCC SIGNAL PIN
#define PROG_SIGNAL_PIN 18

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE THE CURRENT SENSE ATTENUATION. THIS IS USED BY THE ADC SYSTEM TO SCALE
// THE CURRENT SENSE VALUES BASED ON THE MOTOR SHIELD.
//
// SUPPORTED VALUES: ADC_ATTEN_DB_11, ADC_ATTEN_DB_6, ADC_ATTEN_DB_2_5, ADC_ATTEN_DB_0
//
// IF LEFT UNDEFINED ADC_ATTEN_DB_11 WILL BE USED.

//#define ADC_CURRENT_ATTENUATION ADC_ATTEN_DB_11

/////////////////////////////////////////////////////////////////////////////////////
//
// The following parameters define how many preamble bits will be transmitted as part
// of the DCC packet to the track. For some older sound decodes it may be necessary
// to increase from 22 bits on the PROG track to 30 or even 40.
//
// The maximum number of preamble bits is 75. For OPS the minimum to send is 11 but
// 16 is required for RailCom support.

#define OPS_PREAMBLE_BITS 16

#define PROG_PREAMBLE_BITS 22

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WHICH PINS ARE USED FOR OPS RAILCOM DETECTION
//

//#define RAILCOM_BRAKE_ENABLE_PIN NOT_A_PIN
//#define RAILCOM_ENABLE_PIN NOT_A_PIN
//#define RAILCOM_SHORT_PIN NOT_A_PIN
//#define RAILCOM_UART 2
//#define RAILCOM_UART_RX_PIN NOT_A_PIN

/////////////////////////////////////////////////////////////////////////////////////
