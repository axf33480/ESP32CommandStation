/*! \file */

#ifndef __NEONEXTION_NEXTION
#define __NEONEXTION_NEXTION

#include <stdarg.h>

#if CONFIG_ENABLE_ARDUINO_DEPENDS
#include <Arduino.h>
#include <WString.h>
#define STRING_TYPE String
#elif defined(ESP_IDF_VERSION_MAJOR)
#include <string>
#include <stdio.h>
#include <driver/uart.h>
#define STRING_TYPE std::string
#else
#error unsupported platform
#endif

#include "NextionTypes.h"

class INextionTouchable;

/*!
 * \struct ITouchableListItem
 * \brief Linked list node for INextionTouchable objects.
 */
struct ITouchableListItem
{
  INextionTouchable *item;  //!< Pointer to stored INextionTouchable
  ITouchableListItem *next; //!< Pointer to next list node
};

/*!
 * \class Nextion
 * \brief Driver for a physical Nextion device.
 */
class Nextion
{
public:
#if CONFIG_ENABLE_ARDUINO_DEPENDS
  Nextion(Stream &stream, bool flushSerialBeforeTx = true);
#elif defined(ESP_IDF_VERSION_MAJOR)
  Nextion(uint8_t uartNum, long baud, uint8_t rx_pin, uint8_t tx_pin, bool flushSerialBeforeTx = true);
#endif

  bool init();
  void poll();

  bool refresh();
  bool refresh(const STRING_TYPE &objectName);

  bool sleep();
  bool wake();

  uint16_t getBrightness();
  bool setBrightness(uint16_t val, bool persist = false);

  uint8_t getCurrentPage();

  bool clear(uint32_t colour = NEX_COL_WHITE);
  bool drawPicture(uint16_t x, uint16_t y, uint8_t id);
  bool drawPicture(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t id);
  bool drawStr(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t fontID,
               const STRING_TYPE &str, uint32_t bgColour = NEX_COL_BLACK,
               uint32_t fgColour = NEX_COL_WHITE,
               uint8_t bgType = NEX_BG_SOLIDCOLOUR,
               NextionFontAlignment xCentre = NEX_FA_CENTRE,
               NextionFontAlignment yCentre = NEX_FA_CENTRE);
  bool drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                uint32_t colour);
  bool drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool filled,
                uint32_t colour);
  bool drawCircle(uint16_t x, uint16_t y, uint16_t r, uint32_t colour);

  void registerTouchable(INextionTouchable *touchable);
  void sendCommand(const STRING_TYPE &command);
  void sendCommand(const char *format, ...);
  void sendCommand(const char *format, va_list args);
  bool checkCommandComplete();
  bool receiveNumber(uint32_t *number);
  size_t receiveString(STRING_TYPE &buffer, bool stringHeader=true);

private:
#if CONFIG_ENABLE_ARDUINO_DEPENDS
  Stream &m_serialPort;       //!< Serial port device is attached to
#elif defined(ESP_IDF_VERSION_MAJOR)
  uart_port_t m_serialPort;   //!< Serial port device is attached to
#endif
  uint32_t m_timeout;         //!< Serial communication timeout in ms
  bool m_flushSerialBeforeTx; //!< Flush serial port before transmission
  ITouchableListItem *m_touchableList; //!< LInked list of INextionTouchable
};

#endif
