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
// An LCD display can be used to display runtime status information about the
// command station.
//

// LCD Display I2C address, most use 0x27
#define INFO_SCREEN_LCD_I2C_ADDRESS 0x27

// Number of visible lines on the LCD, usually 2 or 4 lines.
#define INFO_SCREEN_LCD_LINES 4

// Number of visible characters in one line on the LCD, usually 16 or 20.
#define INFO_SCREEN_LCD_COLUMNS 20

/////////////////////////////////////////////////////////////////////////////////////

#define INFO_SCREEN_LCD true
