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

#include <stdint.h>

/// This is the width of the font used on the OLED display.
static constexpr uint8_t OLED_FONT_WIDTH = 8;

/// This is the height of the font used on the OLED display.
static constexpr uint8_t OLED_FONT_HEIGHT = 8;

/// This is an 8x8 1bpp font with a 90deg rotation to make it usable on the
/// OLED display. Adding a new character can be done by simply adding a new
/// entry to the map using the hex character code and the 8 bytes for the
/// glyph data.
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

  { 0x00, 0x00, 0x00, 0x5F, 0x01, 0x00, 0x00, 0x00},    // U+0021 (!)
  { 0x00, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x00},    // U+0022 (")
  { 0x14, 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x14, 0x00},    // U+0023 (#)
  { 0x24, 0x2E, 0x6B, 0x2A, 0x6B, 0x3A, 0x12, 0x00},    // U+0024 ($)
  { 0x46, 0x26, 0x10, 0x08, 0x64, 0x62, 0x00, 0x00},    // U+0025 (%)
  { 0x30, 0x7A, 0x4F, 0x5D, 0x37, 0x7A, 0x48, 0x00},    // U+0026 (&)
  { 0x00, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00},    // U+0027 (')
  { 0x00, 0x1C, 0x22, 0x41, 0x41, 0x00, 0x00, 0x00},    // U+0028 (()
  { 0x00, 0x41, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00},    // U+0029 ())
  { 0x08, 0x2A, 0x38, 0x08, 0x38, 0x2A, 0x08, 0x00},    // U+002A (*)
  { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00},    // U+002B (+)
  { 0x00, 0x00, 0x80, 0x60, 0x00, 0x00, 0x00, 0x00},    // U+002C (,)
  { 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00},    // U+002D (-)
  { 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00},    // U+002E (.)
  { 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01, 0x00},    // U+002F (/)
  { 0x3E, 0x61, 0x51, 0x49, 0x45, 0x43, 0x3E, 0x00},    // U+0030 (0)
  { 0x00, 0x44, 0x42, 0x7F, 0x40, 0x40, 0x00, 0x00},    // U+0031 (1)
  { 0x62, 0x51, 0x49, 0x49, 0x49, 0x46, 0x00, 0x00},    // U+0032 (2)
  { 0x22, 0x41, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00},    // U+0033 (3)
  { 0x30, 0x28, 0x24, 0x22, 0x21, 0x7F, 0x20, 0x00},    // U+0034 (4)
  { 0x27, 0x45, 0x45, 0x45, 0x45, 0x39, 0x00, 0x00},    // U+0035 (5)
  { 0x3C, 0x4A, 0x49, 0x49, 0x49, 0x30, 0x00, 0x00},    // U+0036 (6)
  { 0x00, 0x01, 0x01, 0x71, 0x09, 0x07, 0x00, 0x00},    // U+0037 (7)
  { 0x36, 0x49, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00},    // U+0038 (8)
  { 0x06, 0x49, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00},    // U+0039 (9)
  { 0x00, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x00},    // U+003A (:)
  { 0x00, 0x00, 0x80, 0x66, 0x00, 0x00, 0x00, 0x00},    // U+003B (;)
  { 0x08, 0x14, 0x22, 0x41, 0x41, 0x00, 0x00, 0x00},    // U+003C (<)
  { 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x00, 0x00},    // U+003D (=)
  { 0x00, 0x41, 0x41, 0x22, 0x14, 0x08, 0x00, 0x00},    // U+003E (>)
  { 0x02, 0x01, 0x51, 0x09, 0x09, 0x06, 0x00, 0x00},    // U+003F (?)
  { 0x3E, 0x41, 0x41, 0x5D, 0x55, 0x15, 0x1E, 0x00},    // U+0040 (@)
  { 0x7C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x7C, 0x00},    // U+0041 (A)
  { 0x7F, 0x49, 0x49, 0x49, 0x49, 0x7F, 0x36, 0x00},    // U+0042 (B)
  { 0x3E, 0x41, 0x41, 0x41, 0x41, 0x41, 0x22, 0x00},    // U+0043 (C)
  { 0x7F, 0x41, 0x41, 0x41, 0x41, 0x41, 0x3E, 0x00},    // U+0044 (D)
  { 0x7F, 0x49, 0x49, 0x49, 0x49, 0x41, 0x41, 0x00},    // U+0045 (E)
  { 0x7F, 0x09, 0x09, 0x09, 0x09, 0x01, 0x01, 0x00},    // U+0046 (F)
  { 0x3E, 0x41, 0x41, 0x41, 0x41, 0x51, 0x72, 0x00},    // U+0047 (G)
  { 0x7F, 0x08, 0x08, 0x08, 0x08, 0x08, 0x7F, 0x00},    // U+0048 (H)
  { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, 0x00},    // U+0049 (I)
  { 0x30, 0x40, 0x40, 0x40, 0x41, 0x3F, 0x01, 0x00},    // U+004A (J)
  { 0x7F, 0x08, 0x08, 0x08, 0x14, 0x22, 0x41, 0x00},    // U+004B (K)
  { 0x7F, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00},    // U+004C (L)
  { 0x7F, 0x02, 0x04, 0x08, 0x04, 0x02, 0x7F, 0x00},    // U+004D (M)
  { 0x7F, 0x02, 0x04, 0x08, 0x10, 0x20, 0x7F, 0x00},    // U+004E (N)
  { 0x3E, 0x41, 0x41, 0x41, 0x41, 0x41, 0x3E, 0x00},    // U+004F (O)
  { 0x7F, 0x09, 0x09, 0x09, 0x09, 0x09, 0x06, 0x00},    // U+0050 (P)
  { 0x3E, 0x41, 0x41, 0x41, 0x51, 0x21, 0x5E, 0x00},    // U+0051 (Q)
  { 0x7F, 0x09, 0x09, 0x09, 0x19, 0x29, 0x46, 0x00},    // U+0052 (R)
  { 0x26, 0x49, 0x49, 0x49, 0x49, 0x49, 0x32, 0x00},    // U+0053 (S)
  { 0x01, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x01, 0x00},    // U+0054 (T)
  { 0x3F, 0x40, 0x40, 0x40, 0x40, 0x40, 0x3F, 0x00},    // U+0055 (U)
  { 0x0F, 0x10, 0x20, 0x40, 0x20, 0x10, 0x0F, 0x00},    // U+0056 (V)
  { 0x7F, 0x20, 0x10, 0x08, 0x10, 0x10, 0x7F, 0x00},    // U+0057 (W)
  { 0x41, 0x22, 0x14, 0x08, 0x14, 0x22, 0x41, 0x00},    // U+0058 (X)
  { 0x01, 0x02, 0x04, 0x78, 0x04, 0x02, 0x01, 0x00},    // U+0059 (Y)
  { 0x41, 0x61, 0x51, 0x49, 0x45, 0x43, 0x41, 0x00},    // U+005A (Z)
  { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00, 0x00, 0x00},    // U+005B ([)
  { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x00},    // U+005C (\)
  { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00, 0x00, 0x00},    // U+005D (])
  { 0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08, 0x00},    // U+005E (^)
  { 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},    // U+005F (_)
  { 0x00, 0x00, 0x00, 0x03, 0x04, 0x00, 0x00, 0x00},    // U+0060 (`)
  { 0x20, 0x54, 0x54, 0x54, 0x54, 0x78, 0x00, 0x00},    // U+0061 (a)
  { 0x7F, 0x44, 0x44, 0x44, 0x44, 0x38, 0x00, 0x00},    // U+0062 (b)
  { 0x38, 0x44, 0x44, 0x44, 0x44, 0x28, 0x00, 0x00},    // U+0063 (c)
  { 0x38, 0x44, 0x44, 0x44, 0x44, 0x7F, 0x00, 0x00},    // U+0064 (d)
  { 0x38, 0x54, 0x54, 0x54, 0x54, 0x18, 0x00, 0x00},    // U+0065 (e)
  { 0x00, 0x08, 0x7E, 0x09, 0x09, 0x02, 0x00, 0x00},    // U+0066 (f)
  { 0x98, 0xA4, 0xA4, 0xA4, 0xA4, 0x7C, 0x00, 0x00},    // U+0067 (g)
  { 0x7F, 0x08, 0x04, 0x04, 0x04, 0x78, 0x00, 0x00},    // U+0068 (h)
  { 0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00},    // U+0069 (i)
  { 0x60, 0x80, 0x80, 0x80, 0x80, 0x7D, 0x00, 0x00},    // U+006A (j)
  { 0x7F, 0x10, 0x10, 0x10, 0x28, 0x44, 0x00, 0x00},    // U+006B (k)
  { 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00},    // U+006C (l)
  { 0x7C, 0x08, 0x04, 0x78, 0x04, 0x04, 0x78, 0x00},    // U+006D (m)
  { 0x7C, 0x08, 0x04, 0x04, 0x04, 0x78, 0x00, 0x00},    // U+006E (n)
  { 0x38, 0x44, 0x44, 0x44, 0x44, 0x38, 0x00, 0x00},    // U+006F (o)
  { 0xFC, 0x44, 0x44, 0x44, 0x44, 0x38, 0x00, 0x00},    // U+0070 (p)
  { 0x38, 0x44, 0x44, 0x44, 0x44, 0xFC, 0x80, 0x00},    // U+0071 (q)
  { 0x7C, 0x08, 0x04, 0x04, 0x04, 0x18, 0x00, 0x00},    // U+0072 (r)
  { 0x48, 0x54, 0x54, 0x54, 0x54, 0x24, 0x00, 0x00},    // U+0073 (s)
  { 0x04, 0x04, 0x3F, 0x44, 0x44, 0x20, 0x00, 0x00},    // U+0074 (t)
  { 0x3C, 0x40, 0x40, 0x40, 0x20, 0x7C, 0x00, 0x00},    // U+0075 (u)
  { 0x1C, 0x20, 0x40, 0x40, 0x20, 0x1C, 0x00, 0x00},    // U+0076 (v)
  { 0x3C, 0x40, 0x40, 0x38, 0x40, 0x40, 0x3C, 0x00},    // U+0077 (w)
  { 0x44, 0x44, 0x28, 0x10, 0x28, 0x44, 0x44, 0x00},    // U+0078 (x)
  { 0x9C, 0xA0, 0xA0, 0xA0, 0xA0, 0x7C, 0x00, 0x00},    // U+0079 (y)
  { 0x44, 0x64, 0x54, 0x4C, 0x44, 0x44, 0x00, 0x00},    // U+007A (z)
  { 0x00, 0x08, 0x66, 0x41, 0x41, 0x00, 0x00, 0x00},    // U+007B ({)
  { 0x00, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x00},    // U+007C (|)
  { 0x00, 0x00, 0x41, 0x41, 0x66, 0x08, 0x00, 0x00},    // U+007D (})
  { 0x02, 0x01, 0x01, 0x02, 0x02, 0x01, 0x00, 0x00},    // U+007E (~)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},    // U+007F
};