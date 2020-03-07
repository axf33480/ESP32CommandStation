/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2020 Mike Dunston

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

#include "sdkconfig.h"
#include "StatusDisplay.h"

#include <AllTrainNodes.hxx>
#include <ConfigurationManager.h>
#include <driver/i2c.h>
#include <esp_ota_ops.h>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <RMTTrackDevice.h>

static constexpr uint8_t STATUS_DISPLAY_LINE_COUNT = 5;

static constexpr TickType_t DISPLAY_I2C_TIMEOUT =
  pdMS_TO_TICKS(CONFIG_DISPLAY_I2C_TIMEOUT_MSEC);

#if CONFIG_DISPLAY_TYPE_OLED

/// This is the width of the font used on the OLED display.
static constexpr uint8_t OLED_FONT_WIDTH = 8;

/// This is the height of the font used on the OLED display.
static constexpr uint8_t OLED_FONT_HEIGHT = 8;

/// This is an 8x8 1bpp font with a 90deg rotation to make it usable on the
/// OLED display. Adding a new character can be done by simply adding a new
/// entry to the map using the hex character code and the 8 bytes for the
/// glyph data. If a character is not present in the map it will be rendered
/// using 0xff (checkerboard box) to indicate a missing character.
static constexpr uint8_t oled_font[0x80][OLED_FONT_WIDTH] =
{
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0000 (nul)
  { 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA },   // U+0001 (50% on/off pattern)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0002
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0003
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0004
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0005
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0006
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0007
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0008
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0009
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+000A
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+000B
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+000C
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+000D
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+000E
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+000F
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0010
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0011
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0012
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0013
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0014
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0015
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0016
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0017
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0018
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0019
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+001A
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+001B
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+001C
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+001D
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+001E
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+001F
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0020 (space)
  { 0x00, 0x00, 0x06, 0x5F, 0x5F, 0x06, 0x00, 0x00 },   // U+0021 (!)
  { 0x00, 0x03, 0x03, 0x00, 0x03, 0x03, 0x00, 0x00 },   // U+0022 (")
  { 0x14, 0x7F, 0x7F, 0x14, 0x7F, 0x7F, 0x14, 0x00 },   // U+0023 (#)
  { 0x24, 0x2E, 0x6B, 0x6B, 0x3A, 0x12, 0x00, 0x00 },   // U+0024 ($)
  { 0x46, 0x66, 0x30, 0x18, 0x0C, 0x66, 0x62, 0x00 },   // U+0025 (%)
  { 0x30, 0x7A, 0x4F, 0x5D, 0x37, 0x7A, 0x48, 0x00 },   // U+0026 (&)
  { 0x04, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 },   // U+0027 (')
  { 0x00, 0x1C, 0x3E, 0x63, 0x41, 0x00, 0x00, 0x00 },   // U+0028 (()
  { 0x00, 0x41, 0x63, 0x3E, 0x1C, 0x00, 0x00, 0x00 },   // U+0029 ())
  { 0x08, 0x2A, 0x3E, 0x1C, 0x1C, 0x3E, 0x2A, 0x08 },   // U+002A (*)
  { 0x08, 0x08, 0x3E, 0x3E, 0x08, 0x08, 0x00, 0x00 },   // U+002B (+)
  { 0x00, 0x80, 0xE0, 0x60, 0x00, 0x00, 0x00, 0x00 },   // U+002C (,)
  { 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00 },   // U+002D (-)
  { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00 },   // U+002E (.)
  { 0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00 },   // U+002F (/)
  { 0x3E, 0x7F, 0x71, 0x59, 0x4D, 0x7F, 0x3E, 0x00 },   // U+0030 (0)
  { 0x40, 0x42, 0x7F, 0x7F, 0x40, 0x40, 0x00, 0x00 },   // U+0031 (1)
  { 0x62, 0x73, 0x59, 0x49, 0x6F, 0x66, 0x00, 0x00 },   // U+0032 (2)
  { 0x22, 0x63, 0x49, 0x49, 0x7F, 0x36, 0x00, 0x00 },   // U+0033 (3)
  { 0x18, 0x1C, 0x16, 0x53, 0x7F, 0x7F, 0x50, 0x00 },   // U+0034 (4)
  { 0x27, 0x67, 0x45, 0x45, 0x7D, 0x39, 0x00, 0x00 },   // U+0035 (5)
  { 0x3C, 0x7E, 0x4B, 0x49, 0x79, 0x30, 0x00, 0x00 },   // U+0036 (6)
  { 0x03, 0x03, 0x71, 0x79, 0x0F, 0x07, 0x00, 0x00 },   // U+0037 (7)
  { 0x36, 0x7F, 0x49, 0x49, 0x7F, 0x36, 0x00, 0x00 },   // U+0038 (8)
  { 0x06, 0x4F, 0x49, 0x69, 0x3F, 0x1E, 0x00, 0x00 },   // U+0039 (9)
  { 0x00, 0x00, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00 },   // U+003A (:)
  { 0x00, 0x80, 0xE6, 0x66, 0x00, 0x00, 0x00, 0x00 },   // U+003B (;)
  { 0x08, 0x1C, 0x36, 0x63, 0x41, 0x00, 0x00, 0x00 },   // U+003C (<)
  { 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x00, 0x00 },   // U+003D (=)
  { 0x00, 0x41, 0x63, 0x36, 0x1C, 0x08, 0x00, 0x00 },   // U+003E (>)
  { 0x02, 0x03, 0x51, 0x59, 0x0F, 0x06, 0x00, 0x00 },   // U+003F (?)
  { 0x3E, 0x7F, 0x41, 0x5D, 0x5D, 0x1F, 0x1E, 0x00 },   // U+0040 (@)
  { 0x7C, 0x7E, 0x13, 0x13, 0x7E, 0x7C, 0x00, 0x00 },   // U+0041 (A)
  { 0x41, 0x7F, 0x7F, 0x49, 0x49, 0x7F, 0x36, 0x00 },   // U+0042 (B)
  { 0x1C, 0x3E, 0x63, 0x41, 0x41, 0x63, 0x22, 0x00 },   // U+0043 (C)
  { 0x41, 0x7F, 0x7F, 0x41, 0x63, 0x3E, 0x1C, 0x00 },   // U+0044 (D)
  { 0x41, 0x7F, 0x7F, 0x49, 0x5D, 0x41, 0x63, 0x00 },   // U+0045 (E)
  { 0x41, 0x7F, 0x7F, 0x49, 0x1D, 0x01, 0x03, 0x00 },   // U+0046 (F)
  { 0x1C, 0x3E, 0x63, 0x41, 0x51, 0x73, 0x72, 0x00 },   // U+0047 (G)
  { 0x7F, 0x7F, 0x08, 0x08, 0x7F, 0x7F, 0x00, 0x00 },   // U+0048 (H)
  { 0x00, 0x41, 0x7F, 0x7F, 0x41, 0x00, 0x00, 0x00 },   // U+0049 (I)
  { 0x30, 0x70, 0x40, 0x41, 0x7F, 0x3F, 0x01, 0x00 },   // U+004A (J)
  { 0x41, 0x7F, 0x7F, 0x08, 0x1C, 0x77, 0x63, 0x00 },   // U+004B (K)
  { 0x41, 0x7F, 0x7F, 0x41, 0x40, 0x60, 0x70, 0x00 },   // U+004C (L)
  { 0x7F, 0x7F, 0x0E, 0x1C, 0x0E, 0x7F, 0x7F, 0x00 },   // U+004D (M)
  { 0x7F, 0x7F, 0x06, 0x0C, 0x18, 0x7F, 0x7F, 0x00 },   // U+004E (N)
  { 0x1C, 0x3E, 0x63, 0x41, 0x63, 0x3E, 0x1C, 0x00 },   // U+004F (O)
  { 0x41, 0x7F, 0x7F, 0x49, 0x09, 0x0F, 0x06, 0x00 },   // U+0050 (P)
  { 0x1E, 0x3F, 0x21, 0x71, 0x7F, 0x5E, 0x00, 0x00 },   // U+0051 (Q)
  { 0x41, 0x7F, 0x7F, 0x09, 0x19, 0x7F, 0x66, 0x00 },   // U+0052 (R)
  { 0x26, 0x6F, 0x4D, 0x59, 0x73, 0x32, 0x00, 0x00 },   // U+0053 (S)
  { 0x03, 0x41, 0x7F, 0x7F, 0x41, 0x03, 0x00, 0x00 },   // U+0054 (T)
  { 0x7F, 0x7F, 0x40, 0x40, 0x7F, 0x7F, 0x00, 0x00 },   // U+0055 (U)
  { 0x1F, 0x3F, 0x60, 0x60, 0x3F, 0x1F, 0x00, 0x00 },   // U+0056 (V)
  { 0x7F, 0x7F, 0x30, 0x18, 0x30, 0x7F, 0x7F, 0x00 },   // U+0057 (W)
  { 0x43, 0x67, 0x3C, 0x18, 0x3C, 0x67, 0x43, 0x00 },   // U+0058 (X)
  { 0x07, 0x4F, 0x78, 0x78, 0x4F, 0x07, 0x00, 0x00 },   // U+0059 (Y)
  { 0x47, 0x63, 0x71, 0x59, 0x4D, 0x67, 0x73, 0x00 },   // U+005A (Z)
  { 0x00, 0x7F, 0x7F, 0x41, 0x41, 0x00, 0x00, 0x00 },   // U+005B ([)
  { 0x01, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x00 },   // U+005C (\)
  { 0x00, 0x41, 0x41, 0x7F, 0x7F, 0x00, 0x00, 0x00 },   // U+005D (])
  { 0x08, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x08, 0x00 },   // U+005E (^)
  { 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80 },   // U+005F (_)
  { 0x00, 0x00, 0x03, 0x07, 0x04, 0x00, 0x00, 0x00 },   // U+0060 (`)
  { 0x20, 0x74, 0x54, 0x54, 0x3C, 0x78, 0x40, 0x00 },   // U+0061 (a)
  { 0x41, 0x7F, 0x3F, 0x48, 0x48, 0x78, 0x30, 0x00 },   // U+0062 (b)
  { 0x38, 0x7C, 0x44, 0x44, 0x6C, 0x28, 0x00, 0x00 },   // U+0063 (c)
  { 0x30, 0x78, 0x48, 0x49, 0x3F, 0x7F, 0x40, 0x00 },   // U+0064 (d)
  { 0x38, 0x7C, 0x54, 0x54, 0x5C, 0x18, 0x00, 0x00 },   // U+0065 (e)
  { 0x48, 0x7E, 0x7F, 0x49, 0x03, 0x02, 0x00, 0x00 },   // U+0066 (f)
  { 0x98, 0xBC, 0xA4, 0xA4, 0xF8, 0x7C, 0x04, 0x00 },   // U+0067 (g)
  { 0x41, 0x7F, 0x7F, 0x08, 0x04, 0x7C, 0x78, 0x00 },   // U+0068 (h)
  { 0x00, 0x44, 0x7D, 0x7D, 0x40, 0x00, 0x00, 0x00 },   // U+0069 (i)
  { 0x60, 0xE0, 0x80, 0x80, 0xFD, 0x7D, 0x00, 0x00 },   // U+006A (j)
  { 0x41, 0x7F, 0x7F, 0x10, 0x38, 0x6C, 0x44, 0x00 },   // U+006B (k)
  { 0x00, 0x41, 0x7F, 0x7F, 0x40, 0x00, 0x00, 0x00 },   // U+006C (l)
  { 0x7C, 0x7C, 0x18, 0x38, 0x1C, 0x7C, 0x78, 0x00 },   // U+006D (m)
  { 0x7C, 0x7C, 0x04, 0x04, 0x7C, 0x78, 0x00, 0x00 },   // U+006E (n)
  { 0x38, 0x7C, 0x44, 0x44, 0x7C, 0x38, 0x00, 0x00 },   // U+006F (o)
  { 0x84, 0xFC, 0xF8, 0xA4, 0x24, 0x3C, 0x18, 0x00 },   // U+0070 (p)
  { 0x18, 0x3C, 0x24, 0xA4, 0xF8, 0xFC, 0x84, 0x00 },   // U+0071 (q)
  { 0x44, 0x7C, 0x78, 0x4C, 0x04, 0x1C, 0x18, 0x00 },   // U+0072 (r)
  { 0x48, 0x5C, 0x54, 0x54, 0x74, 0x24, 0x00, 0x00 },   // U+0073 (s)
  { 0x00, 0x04, 0x3E, 0x7F, 0x44, 0x24, 0x00, 0x00 },   // U+0074 (t)
  { 0x3C, 0x7C, 0x40, 0x40, 0x3C, 0x7C, 0x40, 0x00 },   // U+0075 (u)
  { 0x1C, 0x3C, 0x60, 0x60, 0x3C, 0x1C, 0x00, 0x00 },   // U+0076 (v)
  { 0x3C, 0x7C, 0x70, 0x38, 0x70, 0x7C, 0x3C, 0x00 },   // U+0077 (w)
  { 0x44, 0x6C, 0x38, 0x10, 0x38, 0x6C, 0x44, 0x00 },   // U+0078 (x)
  { 0x9C, 0xBC, 0xA0, 0xA0, 0xFC, 0x7C, 0x00, 0x00 },   // U+0079 (y)
  { 0x4C, 0x64, 0x74, 0x5C, 0x4C, 0x64, 0x00, 0x00 },   // U+007A (z)
  { 0x08, 0x08, 0x3E, 0x77, 0x41, 0x41, 0x00, 0x00 },   // U+007B ({)
  { 0x00, 0x00, 0x00, 0x77, 0x77, 0x00, 0x00, 0x00 },   // U+007C (|)
  { 0x41, 0x41, 0x77, 0x3E, 0x08, 0x08, 0x00, 0x00 },   // U+007D (})
  { 0x02, 0x03, 0x01, 0x03, 0x02, 0x03, 0x01, 0x00 },   // U+007E (~)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }    // U+007F
};

// Control command bytes for the OLED display
static constexpr uint8_t OLED_COMMAND_STREAM           = 0x00;
static constexpr uint8_t OLED_COMMAND_SINGLE           = 0x80;
static constexpr uint8_t OLED_DATA_STREAM              = 0x40;

// Basic display behavior commands
static constexpr uint8_t OLED_DISPLAY_OFF              = 0xAE;
static constexpr uint8_t OLED_DISPLAY_RAM              = 0xA4;
static constexpr uint8_t OLED_DISPLAY_NORMAL           = 0xA6;
static constexpr uint8_t OLED_DISPLAY_ON               = 0xAF;
static constexpr uint8_t OLED_SET_CONTRAST             = 0x81;

// OLED hardware configuration commands 
static constexpr uint8_t OLED_DISPLAY_START_LINE       = 0x40;
static constexpr uint8_t OLED_SET_SEGMENT_MAP_NORMAL   = 0xA0;
static constexpr uint8_t OLED_SET_SEGMENT_MAP_INVERTED = 0xA1;
static constexpr uint8_t OLED_DISPLAY_MUX              = 0xA8;
static constexpr uint8_t OLED_SET_SCAN_MODE_NORMAL     = 0xC0;
static constexpr uint8_t OLED_SET_SCAN_MODE_INVERTED   = 0xC8;
static constexpr uint8_t OLED_DISPLAY_OFFSET           = 0xD3;
static constexpr uint8_t OLED_COM_PIN_MAP              = 0xDA;

// OLED memory addressing control
static constexpr uint8_t OLED_MEMORY_MODE              = 0x20;
static constexpr uint8_t OLED_MEMORY_MODE_HORIZONTAL   = 0x00;
static constexpr uint8_t OLED_MEMORY_MODE_VERTICAL     = 0x01;
static constexpr uint8_t OLED_MEMORY_MODE_PAGE         = 0x02;
static constexpr uint8_t OLED_MEMORY_COLUMN_RANGE      = 0x21;
static constexpr uint8_t OLED_MEMORY_PAGE_RANGE        = 0x22;
static constexpr uint8_t OLED_SET_PAGE                 = 0xB0;

// OLED Timing/Driving control
static constexpr uint8_t OLED_CLOCK_DIVIDER            = 0xD5;
static constexpr uint8_t OLED_SET_CHARGEPUMP           = 0x8D;
static constexpr uint8_t OLED_CHARGEPUMP_ON            = 0x14;
static constexpr uint8_t OLED_SET_PRECHARGE            = 0xD9;
static constexpr uint8_t OLED_SET_VCOMH                = 0xDB;

// OLED scroll control
static constexpr uint8_t OLED_DISABLE_SCROLL           = 0x2E;

#elif CONFIG_DISPLAY_TYPE_LCD

// LCD control commands
static constexpr uint8_t LCD_DISPLAY_SHIFT             = 0x10;
static constexpr uint8_t LCD_FUNCTION_SET              = 0x20;
static constexpr uint8_t LCD_ADDRESS_SET               = 0x80;

// LCD shift flags
static constexpr uint8_t LCD_SHIFT_LEFT                = 0x08;
static constexpr uint8_t LCD_SHIFT_RIGHT               = 0x0C;

// LCD commands
static constexpr uint8_t LCD_CMD_CLEAR_SCREEN          = 0x01;
static constexpr uint8_t LCD_CMD_RETURN_HOME           = 0x02;
static constexpr uint8_t LCD_CMD_ENTRY_MODE            = 0x04;
static constexpr uint8_t LCD_CMD_DISPLAY_CONTROL       = 0x08;

static constexpr uint8_t LCD_ENTRY_AUTO_SCROLL         = 0x01;
static constexpr uint8_t LCD_ENTRY_LEFT_TO_RIGHT       = 0x02;

// display control flags
static constexpr uint8_t LCD_CURSOR_BLINK_ON           = 0x01;
static constexpr uint8_t LCD_CURSOR_ON                 = 0x02;
static constexpr uint8_t LCD_DISPLAY_ON                = 0x04;

static constexpr uint8_t LCD_8BIT_MODE                 = 0x10;
static constexpr uint8_t LCD_4BIT_MODE                 = 0x00;
static constexpr uint8_t LCD_TWO_LINE_MODE             = 0x08;
static constexpr uint8_t LCD_ONE_LINE_MODE             = 0x00;

static constexpr int LCD_LINE_OFFSETS[] = { 0x00, 0x40, 0x14, 0x54 };

static inline bool send_to_lcd(uint8_t addr, uint8_t value)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, value, true);
  i2c_master_stop(cmd);
  esp_err_t ret =
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(I2C_NUM_0, cmd
                                                     , DISPLAY_I2C_TIMEOUT));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    return false;
  }
  return true;
}

static inline bool send_lcd_nibble(uint8_t addr, uint8_t value, bool data)
{
  uint8_t nibble = (value << 4);
  
  // always send the data with the backlight flag enabled
  nibble |= CONFIG_DISPLAY_LCD_BACKLIGHT_BITMASK;

  // if this is a data byte set the register select flag
  if (data)
  {
    nibble |= CONFIG_DISPLAY_LCD_REGISTER_SELECT_BITMASK;
  }

  // send with ENABLE flag set
  if(!send_to_lcd(addr, nibble | CONFIG_DISPLAY_LCD_ENABLE_BITMASK))
  {
    return false;
  }

  ets_delay_us(1);
  // send without the ENABLE flag set
  if(!send_to_lcd(addr, nibble & ~CONFIG_DISPLAY_LCD_ENABLE_BITMASK))
  {
    return false;
  }
  ets_delay_us(37);
  return true;
}

static inline bool send_lcd_byte(uint8_t addr, uint8_t value, bool data)
{
  return send_lcd_nibble(addr, (value >> 4) & 0x0F, data) &&
         send_lcd_nibble(addr, value & 0x0F, data);
}

#endif

#define I2C_READ_REG(address, reg, data, data_size, status)               \
  {                                                                       \
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                         \
    i2c_master_start(cmd);                                                \
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);  \
    i2c_master_write_byte(cmd, reg, true);                                \
    i2c_master_start(cmd);                                                \
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);   \
    if (data_size > 1)                                                    \
    {                                                                     \
      i2c_master_read(cmd, &data, data_size - 1, I2C_MASTER_ACK);         \
    }                                                                     \
    i2c_master_read_byte(cmd, &data + data_size - 1, I2C_MASTER_NACK);    \
    i2c_master_stop(cmd);                                                 \
    status = i2c_master_cmd_begin(I2C_NUM_0, cmd, DISPLAY_I2C_TIMEOUT);   \
    i2c_cmd_link_delete(cmd);                                             \
  }

StatusDisplay::StatusDisplay(openlcb::SimpleCanStack *stack, Service *service)
  : StateFlowBase(service), lccStatCollector_(stack)
{
#if !CONFIG_DISPLAY_TYPE_NONE
  clear();
  info("ESP32-CS: v%s", esp_ota_get_app_description()->version);
  wifi("IP:Pending");
  Singleton<Esp32WiFiManager>::instance()->add_event_callback(
    std::bind(&StatusDisplay::wifi_event, this, std::placeholders::_1));
  start_flow(STATE(init));
#endif
}

void StatusDisplay::clear()
{
  LOG(VERBOSE, "[StatusDisplay] clear screen");
  for(int line = 0; line < CONFIG_DISPLAY_LINE_COUNT; line++)
  {
    lines_[line] = "";
    lineChanged_[line] = true;
  }
}

void StatusDisplay::info(const std::string &format, ...)
{
#if !CONFIG_DISPLAY_TYPE_NONE
  char buf[256] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
  lines_[0] = buf;
  lineChanged_[0] = true;
#endif
}

void StatusDisplay::status(const std::string &format, ...)
{
#if !CONFIG_DISPLAY_TYPE_NONE
  char buf[256] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
  lines_[CONFIG_DISPLAY_LINE_COUNT - 1] = buf;
  lineChanged_[CONFIG_DISPLAY_LINE_COUNT - 1] = true;
#endif
}

void StatusDisplay::wifi(const std::string &format, ...)
{
#if !CONFIG_DISPLAY_TYPE_NONE
  char buf[256] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
#if CONFIG_DISPLAY_LINE_COUNT > 2
  lines_[1] = buf;
  lineChanged_[1] = true;
#else
  lines_[0] = buf;
  lineChanged_[0] = true;
#endif
#endif
}

void StatusDisplay::track_power(const std::string &format, ...)
{
#if CONFIG_DISPLAY_LINE_COUNT > 2
  char buf[256] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
  lines_[2] = buf;
  lineChanged_[2] = true;
#endif
}

void StatusDisplay::wifi_event(system_event_t *event)
{
  if(event->event_id == SYSTEM_EVENT_STA_GOT_IP ||
       event->event_id == SYSTEM_EVENT_AP_START)
  {
    if (event->event_id == SYSTEM_EVENT_STA_GOT_IP)
    {
      tcpip_adapter_ip_info_t ip_info;
      tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);
#if CONFIG_DISPLAY_COLUMN_COUNT > 16 || CONFIG_DISPLAY_TYPE_OLED
      wifi("IP: " IPSTR, IP2STR(&ip_info.ip));
#else
      wifi(IPSTR, IP2STR(&ip_info.ip));
#endif
    }
    else
    {
      wifi("SSID: %s"
         , Singleton<ConfigurationManager>::instance()->getSSID().c_str());
    }
  } else if (event->event_id == SYSTEM_EVENT_STA_LOST_IP ||
              event->event_id == SYSTEM_EVENT_AP_STOP)
  {
    wifi("Disconnected");
  }
}

StateFlowBase::Action StatusDisplay::init()
{
#if !CONFIG_DISPLAY_TYPE_NONE
  LOG(INFO, "[StatusDisplay] Initializing I2C driver...");
  i2c_config_t i2c_config =
  {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = (gpio_num_t)CONFIG_DISPLAY_SDA,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = (gpio_num_t)CONFIG_DISPLAY_SCL,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master =
    {
      .clk_speed = CONFIG_DISPLAY_I2C_BUS_SPEED
    }
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

  // scan a handful of known I2C address ranges for LCD or OLED devices
  esp_err_t ret;
#if CONFIG_DISPLAY_TYPE_OLED
  for(uint8_t addr = 0x3C; addr <= 0x3C; addr++)
  {
    LOG(VERBOSE, "[StatusDisplay] Searching for OLED on address 0x%02x...", addr);
    I2C_READ_REG(addr, 0, regZero_, 1, ret);
    if (ret == ESP_OK)
    {
      i2cAddr_ = addr;
      return call_immediately(STATE(initOLED));
    }
  }
#elif CONFIG_DISPLAY_TYPE_LCD
  // PCF8574 or MCP23008
  for(uint8_t addr = 0x20; addr <= 0x27; addr++)
  {
    LOG(VERBOSE, "[StatusDisplay] Searching for LCD on address 0x%02x...", addr);
    I2C_READ_REG(addr, 0, regZero_, 1, ret);
    if (ret == ESP_OK)
    {
      i2cAddr_ = addr;
      return call_immediately(STATE(initLCD));
    }
  }
  // PCF8574A
  for(uint8_t addr = 0x38; addr <= 0x3F; addr++)
  {
    LOG(VERBOSE, "[StatusDisplay] Searching for LCD on address 0x%02x...", addr);
    I2C_READ_REG(addr, 0, regZero_, 1, ret);
    if (ret == ESP_OK)
    {
      i2cAddr_ = addr;
      return call_immediately(STATE(initLCD));
    }
  }
#endif
  LOG(WARNING, "[StatusDisplay] no display detected");

  // Scan the I2C bus and dump the output of devices that respond
  std::string scanresults =
    "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n"
    "00:         ";
  scanresults.reserve(256);
  for (uint8_t addr=3; addr < 0x78; addr++)
  {
    if (addr % 16 == 0)
    {
      scanresults += "\n" + int64_to_string_hex(addr) + ":";
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, DISPLAY_I2C_TIMEOUT);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
    {
      scanresults += int64_to_string_hex(addr);
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
      scanresults += " ??";
    }
    else
    {
      scanresults += " --";
    }
  }
  LOG(INFO, "[StatusDisplay] %s", scanresults.c_str());
  LOG(WARNING, "[StatusDisplay] A supported display was not detected, below "
      "are the detected I2C devices\n%s", scanresults.c_str());
#endif // !CONFIG_DISPLAY_TYPE_NONE
  return exit();
}

StateFlowBase::Action StatusDisplay::initOLED()
{
#if CONFIG_DISPLAY_TYPE_OLED
  LOG(INFO, "[StatusDisplay] OLED display detected on address 0x%02x"
    , i2cAddr_);
#if CONFIG_DISPLAY_OLED_RESET_PIN != -1
  static bool resetCalled = false;
  if(!resetCalled)
  {
    LOG(INFO, "[StatusDisplay] Resetting OLED display");
    gpio_pad_select_gpio((gpio_num_t)CONFIG_DISPLAY_OLED_RESET_PIN);
    gpio_set_direction((gpio_num_t)CONFIG_DISPLAY_OLED_RESET_PIN
                      , GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)CONFIG_DISPLAY_OLED_RESET_PIN, 0);
    resetCalled = true;
    return sleep_and_call(&timer_, MSEC_TO_NSEC(50), STATE(initOLED));
  }
  else
  {
    gpio_set_level((gpio_num_t)CONFIG_DISPLAY_OLED_RESET_PIN, 1);
  }
#endif // CONFIG_DISPLAY_OLED_RESET_PIN
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2cAddr_ << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_COMMAND_STREAM, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_OFF, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_MUX, true);
  i2c_master_write_byte(cmd, CONFIG_DISPLAY_OLED_WIDTH -1, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_OFFSET, true);
  i2c_master_write_byte(cmd, 0x00, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_START_LINE, true);
  i2c_master_write_byte(cmd, OLED_SET_CHARGEPUMP, true);
  i2c_master_write_byte(cmd, OLED_CHARGEPUMP_ON, true);

  // Test the register zero data with power/state masked to identify the
  // connected chipset since SSD1306 and SH1106 require slightly different
  // initialization parameters.
  if (((regZero_ & 0x0F) == 0x03) || ((regZero_ & 0x0F) == 0x06))
  {
    LOG(INFO, "[StatusDisplay] OLED driver IC: SSD1306");
    i2c_master_write_byte(cmd, OLED_CLOCK_DIVIDER, true);
    i2c_master_write_byte(cmd, 0x80, true);
    i2c_master_write_byte(cmd, OLED_MEMORY_MODE, true);
    i2c_master_write_byte(cmd, OLED_MEMORY_MODE_PAGE, true);
    i2c_master_write_byte(cmd, OLED_SET_PRECHARGE, true);
    i2c_master_write_byte(cmd, 0xF1, true);
    i2c_master_write_byte(cmd, OLED_SET_VCOMH, true);
    i2c_master_write_byte(cmd, 0x40, true);
  }
  else if (((regZero_ & 0x0F) == 0x08))
  {
    LOG(INFO, "[StatusDisplay] OLED driver IC: SH1106");
    sh1106_ = true;
    i2c_master_write_byte(cmd, OLED_CLOCK_DIVIDER, true);
    i2c_master_write_byte(cmd, 0xF0, true);
    i2c_master_write_byte(cmd, OLED_MEMORY_MODE, true);
    i2c_master_write_byte(cmd, OLED_MEMORY_MODE_HORIZONTAL, true);
    i2c_master_write_byte(cmd, OLED_SET_PRECHARGE, true);
    i2c_master_write_byte(cmd, 0x22, true);
    i2c_master_write_byte(cmd, OLED_SET_VCOMH, true);
    i2c_master_write_byte(cmd, 0x30, true);
  }
  else
  {
    LOG(WARNING, "[StatusDisplay] Unrecognized OLED Register zero value: %x"
      , regZero_ & 0x0F);
    // cleanup and abort init process
    i2c_cmd_link_delete(cmd);
    return exit();
  }
  LOG(INFO, "[StatusDisplay] OLED display size: %dx%d (%d lines)"
    , CONFIG_DISPLAY_OLED_WIDTH, CONFIG_DISPLAY_OLED_HEIGHT
    , CONFIG_DISPLAY_LINE_COUNT);

#if CONFIG_DISPLAY_OLED_VFLIP
  i2c_master_write_byte(cmd, OLED_SET_SEGMENT_MAP_INVERTED, true);
  i2c_master_write_byte(cmd, OLED_SET_SCAN_MODE_INVERTED, true);
#else
  i2c_master_write_byte(cmd, OLED_SET_SEGMENT_MAP_NORMAL, true);
  i2c_master_write_byte(cmd, OLED_SET_SCAN_MODE_NORMAL, true);
#endif

  i2c_master_write_byte(cmd, OLED_COM_PIN_MAP, true);
#if CONFIG_DISPLAY_OLED_128x64
  i2c_master_write_byte(cmd, 0x12, true);
#elif CONFIG_DISPLAY_OLED_128x32 || CONFIG_DISPLAY_OLED_96x16
  i2c_master_write_byte(cmd, 0x02, true);
#endif

  i2c_master_write_byte(cmd, OLED_SET_CONTRAST, true);
  i2c_master_write_byte(cmd, CONFIG_DISPLAY_OLED_CONTRAST, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_RAM, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_NORMAL, true);
  i2c_master_write_byte(cmd, OLED_DISABLE_SCROLL, true);
  i2c_master_write_byte(cmd, OLED_DISPLAY_ON, true);
	i2c_master_stop(cmd);

  LOG(INFO, "[StatusDisplay] Initializing OLED display");
  esp_err_t ret =
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(I2C_NUM_0, cmd
                                                     , DISPLAY_I2C_TIMEOUT));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    LOG_ERROR("[StatusDisplay] Failed to initialize the OLED display");
    return exit();
  }
  LOG(INFO, "[StatusDisplay] OLED successfully initialized");

  return call_immediately(STATE(update));
#else
  return exit();
#endif // CONFIG_DISPLAY_TYPE_OLED
}

StateFlowBase::Action StatusDisplay::initLCD()
{
#if CONFIG_DISPLAY_TYPE_LCD
  LOG(INFO,
      "[StatusDisplay] Detected LCD on address 0x%02x, initializing %dx%x "
      "display..."
    , i2cAddr_, CONFIG_DISPLAY_COLUMN_COUNT, CONFIG_DISPLAY_LINE_COUNT);

  // wake up the LCD and init to known state
  send_to_lcd(i2cAddr_, 0x00);
  vTaskDelay(pdMS_TO_TICKS(10));

  // send init data to reset to 8 bit mode
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | LCD_8BIT_MODE | CONFIG_DISPLAY_LCD_ENABLE_BITMASK));
  ets_delay_us(1);
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | LCD_8BIT_MODE));
  ets_delay_us(4537);
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | LCD_8BIT_MODE | CONFIG_DISPLAY_LCD_ENABLE_BITMASK));
  ets_delay_us(1);
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | LCD_8BIT_MODE));
  ets_delay_us(237);
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | LCD_8BIT_MODE | CONFIG_DISPLAY_LCD_ENABLE_BITMASK));
  ets_delay_us(1);
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | LCD_8BIT_MODE));
  ets_delay_us(237);

  // switch to four bit mode
  send_to_lcd(i2cAddr_, (LCD_FUNCTION_SET | CONFIG_DISPLAY_LCD_ENABLE_BITMASK));
  ets_delay_us(1);
  send_to_lcd(i2cAddr_, LCD_FUNCTION_SET);
  ets_delay_us(37);

  // send rest of init commands as 4 bit mode
  send_lcd_byte(i2cAddr_, LCD_FUNCTION_SET | LCD_TWO_LINE_MODE, false);
  send_lcd_byte(i2cAddr_, LCD_CMD_DISPLAY_CONTROL | LCD_DISPLAY_ON, false);
  send_lcd_byte(i2cAddr_, LCD_CMD_CLEAR_SCREEN, false);
  ets_delay_us(1600); // clear takes 1.5ms
  send_lcd_byte(i2cAddr_, LCD_CMD_ENTRY_MODE | LCD_ENTRY_LEFT_TO_RIGHT, false);
  send_lcd_byte(i2cAddr_, LCD_CMD_RETURN_HOME, false);
  ets_delay_us(1600); // home takes 1.5ms
  return call_immediately(STATE(update));
#else
  return exit();
#endif // CONFIG_DISPLAY_TYPE_LCD
}

StateFlowBase::Action StatusDisplay::update()
{
#if CONFIG_LOCONET
  static uint8_t rotatingLineCount = 7;
#else
  static uint8_t rotatingLineCount = 5;
#endif
  // switch to next status line detail set after 10 iterations
  if(++updateCount_ > 10)
  {
    updateCount_ = 0;
    ++rotatingIndex_ %= rotatingLineCount;
  }
  // update the status line details every other iteration
  if(updateCount_ % 2)
  {
    if(rotatingIndex_ == 0)
    {
      status("Free Heap:%d"
                , heap_caps_get_free_size(MALLOC_CAP_INTERNAL)
      );
    }
    else if (rotatingIndex_ == 1)
    {
      uint64_t seconds = USEC_TO_SEC(esp_timer_get_time());
      status("Uptime: %02d:%02d:%02d"
                , (uint32_t)(seconds / 3600), (uint32_t)(seconds % 3600) / 60
                , (uint32_t)(seconds % 60)
      );
    }
    else if (rotatingIndex_ == 2)
    {
      status("Active Locos:%3d"
                , Singleton<commandstation::AllTrainNodes>::instance()->size()
      );
    }
    else if (rotatingIndex_ == 3)
    {
      status(Singleton<RMTTrackDevice>::instance()->get_status_screen_data());
    }
    else if (rotatingIndex_ == 4)
    {
      static uint8_t _lccStatusIndex = 0;
      ++_lccStatusIndex %= 5;
      if(_lccStatusIndex == 0)
      {
        status("LCC Nodes: %d"
                  , lccStatCollector_.getRemoteNodeCount()
        );
      }
      else if (_lccStatusIndex == 1)
      {
        status("LCC Lcl: %d", lccStatCollector_.getLocalNodeCount()
        );
      }
      else if (_lccStatusIndex == 2)
      {
        status("LCC dg_svc: %d", lccStatCollector_.getDatagramCount()
        );
      }
      else if (_lccStatusIndex == 3)
      {
        status("LCC Ex: %d", lccStatCollector_.getExecutorCount()
        );
      }
      else if (_lccStatusIndex == 4)
      {
        status("LCC Pool: %d/%d", lccStatCollector_.getPoolFreeCount()
                  , lccStatCollector_.getPoolSize()
        );
      }
#if CONFIG_LOCONET
    }
    else if (rotatingIndex_ == 5)
    {
      status("LN-RX: %d/%d", locoNet.getRxStats()->rxPackets
                , locoNet.getRxStats()->rxErrors
      );
    }
    else if (rotatingIndex_ == 6)
    {
      status("LN-TX: %d/%d/%d", locoNet.getTxStats()->txPackets
                , locoNet.getTxStats()->txErrors
                , locoNet.getTxStats()->collisions
      );
#endif
    }
  }

  for (uint8_t line = 0; line < CONFIG_DISPLAY_LINE_COUNT; line++)
  {
    // if the line has not changed skip it
    if (!lineChanged_[line])
    {
      continue;
    }

#if CONFIG_DISPLAY_TYPE_OLED
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_COMMAND_STREAM, true);
    i2c_master_write_byte(cmd, OLED_SET_PAGE | line, true);
    if (sh1106_)
    {
      i2c_master_write_byte(cmd, 0x02, true);
      i2c_master_write_byte(cmd, 0x10, true);
    }
    else
    {
      i2c_master_write_byte(cmd, OLED_MEMORY_COLUMN_RANGE, true);
      i2c_master_write_byte(cmd, 0x00, true);
      i2c_master_write_byte(cmd, CONFIG_DISPLAY_OLED_WIDTH - 1, true);
    }
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK_WITHOUT_ABORT(
      i2c_master_cmd_begin(I2C_NUM_0, cmd, DISPLAY_I2C_TIMEOUT));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddr_ << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_DATA_STREAM, true);
    uint8_t col = 0;
    for (auto ch : lines_[line])
    {
      // Check that the character is a renderable character.
      if (ch <= 0x7f)
      {
        i2c_master_write(cmd, (uint8_t *)oled_font[(uint8_t)ch]
                        , OLED_FONT_WIDTH, true);
      }
      else
      {
        // since it is not a renderable character, send the 50% shaded block to
        // the display instead so the rendering stays consistent
        i2c_master_write(cmd, (uint8_t *)oled_font[1], OLED_FONT_WIDTH, true);
      }
      col++;
      // make sure we haven't rendered past the end of the display
      if (col >= CONFIG_DISPLAY_COLUMN_COUNT)
      {
        break;
      }
    }
    while(col++ < CONFIG_DISPLAY_COLUMN_COUNT)
    {
      i2c_master_write(cmd, (uint8_t *)oled_font[0], OLED_FONT_WIDTH, true);
    }
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK_WITHOUT_ABORT(
      i2c_master_cmd_begin(I2C_NUM_0, cmd, DISPLAY_I2C_TIMEOUT));
    i2c_cmd_link_delete(cmd);
#elif CONFIG_DISPLAY_TYPE_LCD
    send_lcd_byte(i2cAddr_, LCD_ADDRESS_SET | LCD_LINE_OFFSETS[line], false);
    uint8_t col = 0;
    for (auto ch : lines_[line])
    {
      send_lcd_byte(i2cAddr_, ch, true);
      col++;
      if (col >= CONFIG_DISPLAY_COLUMN_COUNT)
      {
        break;
      }
    }
    // space pad to the width of the LCD
    while(col++ < CONFIG_DISPLAY_COLUMN_COUNT)
    {
      send_lcd_byte(i2cAddr_, ' ', true);
    }
#endif
    lineChanged_[line] = false;
  }
  return sleep_and_call(&timer_, MSEC_TO_NSEC(450), STATE(update));
}