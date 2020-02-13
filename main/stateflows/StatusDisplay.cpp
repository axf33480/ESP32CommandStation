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

#include "ESP32CommandStation.h"

#include <driver/i2c.h>

static constexpr uint8_t STATUS_DISPLAY_LINE_COUNT = 5;

#if CONFIG_DISPLAY_TYPE_OLED
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
static constexpr uint8_t OLED_SET_PAGE                 = 0x80;

// OLED Timing/Driving control
static constexpr uint8_t OLED_CLOCK_DIVIDER            = 0xD5;
static constexpr uint8_t OLED_SET_CHARGEPUMP           = 0x8D;
static constexpr uint8_t OLED_CHARGEPUMP_ON            = 0x14;
static constexpr uint8_t OLED_SET_PRECHARGE            = 0xD9;
static constexpr uint8_t OLED_SET_VCOMH                = 0xDB;

// OLED scroll control
static constexpr uint8_t OLED_DISABLE_SCROLL           = 0x2E;

/// Initialization sequence for the OLED displays, primarily tested with the
/// SH1106. SSD1306 works similarly but may require additional settings which
/// can be added if needed.
static uint8_t OLED_INIT_SEQ[] =
{
  OLED_COMMAND_STREAM,
  OLED_DISPLAY_OFF,
  OLED_CLOCK_DIVIDER, 0x80,
  OLED_DISPLAY_MUX, CONFIG_DISPLAY_OLED_HEIGHT - 1,
  OLED_DISPLAY_OFFSET, 0x00,
  OLED_DISPLAY_START_LINE,
  OLED_SET_CHARGEPUMP,
  OLED_CHARGEPUMP_ON,
  OLED_MEMORY_MODE, OLED_MEMORY_MODE_HORIZONTAL,
#if CONFIG_DISPLAY_OLED_VFLIP
  OLED_SET_SEGMENT_MAP_INVERTED,
  OLED_SET_SCAN_MODE_INVERTED,
#else
  OLED_SET_SEGMENT_MAP_NORMAL,
  OLED_SET_SCAN_MODE_NORMAL,
#endif
#if CONFIG_DISPLAY_OLED_128x64
  OLED_COM_PIN_MAP, 0x12,
#elif CONFIG_DISPLAY_OLED_128x32 || CONFIG_DISPLAY_OLED_128x16
// 96x16/96x32
  OLED_COM_PIN_MAP, 0x02,
#endif
  OLED_SET_CONTRAST, 0xFF,
  OLED_SET_PRECHARGE, 0x22,
  OLED_SET_VCOMH, 0x20,
  OLED_DISPLAY_RAM,
  OLED_DISPLAY_NORMAL,
  OLED_DISABLE_SCROLL,
  OLED_DISPLAY_ON
};

#endif

static constexpr TickType_t DISPLAY_I2C_TIMEOUT =
  pdMS_TO_TICKS(CONFIG_DISPLAY_I2C_TIMEOUT_MSEC);

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
  : StateFlowBase(service)
#if CONFIG_LOCONET
  , rotatingLineCount_(7)
#else
  , rotatingLineCount_(5)
#endif
{
  clear();
#if !CONFIG_DISPLAY_TYPE_NONE
  lccStatCollector_.emplace(stack);
  start_flow(STATE(init));
#endif
}

void StatusDisplay::clear()
{
  LOG(VERBOSE, "[StatusDisplay] clear screen");
  for(int line = 0; line < TEXT_ROW_COUNT; line++)
  {
    screenLines_[line] = "";
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
  screenLines_[0] = buf;
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
  // last line is always rotating status line
  screenLines_[TEXT_ROW_COUNT - 1] = buf;
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
#if CONFIG_DISPLAY_OLED_LINE_COUNT >= 2 || CONFIG_DISPLAY_LCD_LINE_COUNT > 2
  screenLines_[1] = buf;
#else
  screenLines_[0] = buf;
#endif
#endif
}

void StatusDisplay::tcp_clients(const std::string &format, ...)
{
#if CONFIG_DISPLAY_OLED_LINE_COUNT > 2
  char buf[256] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
  screenLines_[2] = buf;
#endif
}

void StatusDisplay::track_power(const std::string &format, ...)
{
#if CONFIG_DISPLAY_OLED_LINE_COUNT > 2 || CONFIG_DISPLAY_LCD_LINE_COUNT > 2
  char buf[256] = {0};
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format.c_str(), args);
  va_end(args);
  screenLines_[2] = buf;
#endif
}

StateFlowBase::Action StatusDisplay::init()
{
#if !CONFIG_DISPLAY_TYPE_NONE
  bool wantScan{false};
  info("ESP32-CS: v%s", esp_ota_get_app_description()->version);
  status("Starting Up");
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

  LOG(INFO, "[StatusDisplay] Searching for I2C device on address 0x%02x..."
    , CONFIG_DISPLAY_ADDRESS);
  esp_err_t ret;
  I2C_READ_REG(CONFIG_DISPLAY_ADDRESS, 0, regZero_, 1, ret);
  if (ret == ESP_OK)
  {
#if CONFIG_DISPLAY_TYPE_OLED
    return call_immediately(STATE(initOLED));
#elif CONFIG_DISPLAY_TYPE_LCD
    return call_immediately(STATE(initLCD));
#endif
  }
  else if (ret == ESP_ERR_TIMEOUT)
  {
    LOG_ERROR("[StatusDisplay] I2C device at address 0x%02x did not respond "
              "within %dms, giving up", CONFIG_DISPLAY_ADDRESS
            , CONFIG_DISPLAY_I2C_TIMEOUT_MSEC);
  }
  else
  {
    wantScan = true;
  }

  if (wantScan)
  {
    // Scan the I2C bus and dump the output of devices that respond
    std::string scanresults =
      "Scanning for I2C devices...\n"
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
    LOG(WARNING,
        "I2C display not found at 0x%02x\n%s",
        CONFIG_DISPLAY_ADDRESS,
        scanresults.c_str());
  }
  // The only time we should encounter this case is if the I2C init
  // fails. Cleanup and exit the flow.
  LOG(WARNING, "[StatusDisplay] no display detected");
#endif // !CONFIG_DISPLAY_TYPE_NONE
  return exit();
}

StateFlowBase::Action StatusDisplay::initOLED()
{
#if CONFIG_DISPLAY_TYPE_OLED
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
	i2c_master_write_byte(cmd, (CONFIG_DISPLAY_ADDRESS << 1) | I2C_MASTER_WRITE
                      , true);
  regZero_ &= 0xBF;
  if (regZero_ == 0x08)
  {
    LOG(INFO, "[StatusDisplay] SH1106 detected");
    sh1106_ = true;
  }
  if (regZero_ == 0x03 || regZero_ == 0x06)
  {
    LOG(INFO, "[StatusDisplay] SSD1306 detected");
  }
  i2c_master_write(cmd, OLED_INIT_SEQ, sizeof(OLED_INIT_SEQ), true);
	i2c_master_stop(cmd);

  LOG(INFO, "[StatusDisplay] Sending init parameters to OLED display");
  esp_err_t ret =
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(I2C_NUM_0, cmd
                                                     , DISPLAY_I2C_TIMEOUT));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    LOG_ERROR("[StatusDisplay] Failed to initialize the OLED display");
    return exit();
  }
  LOG(INFO, "[StatusDisplay] Initialized OLED");

  return call_immediately(STATE(update));
#else
  return exit();
#endif // CONFIG_DISPLAY_TYPE_OLED
}

StateFlowBase::Action StatusDisplay::initLCD()
{
#if CONFIG_DISPLAY_TYPE_LCD
  LOG(INFO,
      "[StatusDisplay] Detected LCD on address %02x, initializing %dx%x display..."
    , CONFIG_DISPLAY_ADDRESS, CONFIG_DISPLAY_LCD_COLUMN_COUNT
    , CONFIG_DISPLAY_LCD_LINE_COUNT);
  lcdDisplay.begin(CONFIG_DISPLAY_LCD_COLUMN_COUNT, CONFIG_DISPLAY_LCD_LINE_COUNT);
  lcdDisplay.setBacklight(255);
  lcdDisplay.clear();
  return call_immediately(STATE(update));
#else
  return exit();
#endif // CONFIG_DISPLAY_TYPE_LCD
}

StateFlowBase::Action StatusDisplay::update()
{
  // switch to next status line detail set after 10 iterations
  if(++updateCount_ > 10)
  {
    updateCount_ = 0;
    ++rotatingIndex_ %= rotatingLineCount_;
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
                , Singleton<AllTrainNodes>::instance()->size()
      );
    }
    else if (rotatingIndex_ == 3)
    {
      status(trackSignal->get_status_screen_data());
    }
    else if (rotatingIndex_ == 4)
    {
      static uint8_t _lccStatusIndex = 0;
      ++_lccStatusIndex %= 5;
      if(_lccStatusIndex == 0)
      {
        status("LCC Nodes: %d"
                  , lccStatCollector_->getRemoteNodeCount()
        );
      }
      else if (_lccStatusIndex == 1)
      {
        status("LCC Lcl: %d", lccStatCollector_->getLocalNodeCount()
        );
      }
      else if (_lccStatusIndex == 2)
      {
        status("LCC dg_svc: %d", lccStatCollector_->getDatagramCount()
        );
      }
      else if (_lccStatusIndex == 3)
      {
        status("LCC Ex: %d", lccStatCollector_->getExecutorCount()
        );
      }
      else if (_lccStatusIndex == 4)
      {
        status("LCC Pool: %d/%d", lccStatCollector_->getPoolFreeCount()
                  , lccStatCollector_->getPoolSize()
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

#if CONFIG_DISPLAY_TYPE_OLED
  for (uint8_t row = 0; row < TEXT_ROW_COUNT; row++)
  {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd
                        , (CONFIG_DISPLAY_ADDRESS << 1) | I2C_MASTER_WRITE
                        , true);
    i2c_master_write_byte(cmd, OLED_COMMAND_STREAM, true);
    i2c_master_write_byte(cmd, OLED_SET_PAGE | row, true);
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
      i2c_master_cmd_begin(I2C_NUM_0, cmd, DISPLAY_I2C_TIMEOUT << 1));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd
                        , (CONFIG_DISPLAY_ADDRESS << 1) | I2C_MASTER_WRITE
                        , true);
    i2c_master_write_byte(cmd, OLED_DATA_STREAM, true);
    uint8_t col = 0;
    for (auto ch : screenLines_[row])
    {
      i2c_master_write(cmd, (uint8_t *)oled_font[(uint8_t)ch]
                      , OLED_FONT_WIDTH, true);
      col++;
    }
    for(; col < TEXT_COLUMN_COUNT; col++)
    {
      i2c_master_write(cmd, (uint8_t *)oled_font[0], OLED_FONT_WIDTH, true);
    }
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK_WITHOUT_ABORT(
      i2c_master_cmd_begin(I2C_NUM_0, cmd, DISPLAY_I2C_TIMEOUT << 1));
    i2c_cmd_link_delete(cmd);
  }
#elif CONFIG_DISPLAY_TYPE_LCD
  for (int line = 0; line < CONFIG_DISPLAY_LCD_LINE_COUNT; line++)
  {
    lcdDisplay.setCursor(0, line);
    // space pad to the width of the LCD
    while(screenLines_[line].length() < CONFIG_DISPLAY_LCD_COLUMN_COUNT)
    {
      screenLines_[line] += ' ';
    }
    lcdDisplay.print(screenLines_[line].c_str());
  }
#endif
  return sleep_and_call(&timer_, MSEC_TO_NSEC(450), STATE(update));
}