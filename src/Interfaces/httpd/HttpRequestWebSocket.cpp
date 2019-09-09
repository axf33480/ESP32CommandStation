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

#include "ESP32CommandStation.h"
#include "interfaces/httpd/Httpd.h"

namespace esp32cs
{
namespace httpd
{

typedef enum
{
  CONTINUATION = 0x0 // Continuation Frame
, TEXT         = 0x1 // Text Frame
, BINARY       = 0x2 // Binary Frame
, CLOSE        = 0x8 // Connection Close Frame
, PING         = 0x9 // Ping Frame
, PONG         = 0xA // Pong Frame
} WebSocketOpcode;

WebsocketFlow::WebsocketFlow(Httpd *server, int fd, const string &ws_key
                           , const string &ws_version
                           , WebSocketHandler handler)
                           : StateFlowBase(server)
                           , fd_(fd)
                           , timeout_(MSEC_TO_NSEC(config_httpd_websocket_timeout_ms()))
                           , max_frame_size_(config_httpd_websocket_max_frame_size())
                           , handshake_(STATUS_SWITCH_PROTOCOL)
                           , handler_(handler)
{
  string key_data = ws_key + WEBSOCKET_UUID;
  unsigned char key_sha1[20];

  LOG(INFO, "[WebSocket fd:%d] Connected, starting handshake", fd_);
  // SHA1 encode the ws_key plus the websocket UUID, if this fails close the
  // socket immediately.
  if (!mbedtls_sha1_ret((unsigned char *)key_data.c_str(), key_data.length()
                      , key_sha1))
  {
    handshake_.add_header(HTTP_HEADER_CONNECTION, HTTP_CONNECTION_UPGRADE);
    handshake_.add_header(HTTP_HEADER_UPGRADE, HTTP_UPGRADE_HEADER_WEBSOCKET);
    handshake_.add_header(WEBSOCKET_HEADER_VERSION, ws_version);
    handshake_.add_header(WEBSOCKET_HEADER_ACCEPT
                        , base64_encode(string((char *)key_sha1, 20)));

    start_flow(STATE(start_handshake));
  }
  else
  {
    LOG_ERROR("[WebSocket fd:%d] SHA-1 handshake failed, closing", fd_);
    start_flow(STATE(delete_this));
  }
}

WebsocketFlow::~WebsocketFlow()
{
  LOG(INFO, "[WebSocket fd:%d] Disconnected", fd_);
  if (!sent_disconnect_)
  {
    handler_(fd_, DISCONNECT, false, nullptr, 0);
  }
  ::close(fd_);
}

StateFlowBase::Action WebsocketFlow::read_fully_with_timeout(void *buf
                                                           , size_t size
                                                           , StateFlowBase::Callback c)
{
  buf_ = (uint8_t *)buf;
  buf_offs_ = 0;
  buf_remain_ = size;
  buf_next_ = c;
  return read_repeated_with_timeout(&helper_, timeout_, fd_, buf_, size
                                  , STATE(data_chunk_received));
}

StateFlowBase::Action WebsocketFlow::data_chunk_received()
{
  HASSERT(buf_next_);
  if (helper_.hasError_)
  {
    LOG(INFO, "[WebSocket fd:%d] read-error, disconnecting", fd_);
    return delete_this();
  }
  size_t received = (buf_remain_ - helper_.remaining_);
  buf_remain_ -= received;
  buf_offs_ += received;
  LOG(VERBOSE, "[WebSocket fd:%d] Received %zu bytes, %zu bytes remain", fd_
    , received, buf_remain_);
  if (buf_remain_)
  {
    return read_repeated_with_timeout(&helper_, timeout_, fd_, buf_ + buf_offs_
                                    , buf_remain_, STATE(data_chunk_received));
  }
  return call_immediately(buf_next_);
}

StateFlowBase::Action WebsocketFlow::start_handshake()
{
  size_t len;
  uint8_t *payload = handshake_.get_headers(&len, false, false);
  LOG(INFO, "[WebSocket fd:%d] Sending handshake:\n%s", fd_, payload);
  return write_repeated(&helper_, fd_, payload, len, STATE(read_frame_header));
}

StateFlowBase::Action WebsocketFlow::read_frame_header()
{
  if (!sent_connect_)
  {
    handler_(fd_, CONNECT, false, nullptr, 0);
    sent_connect_ = true;
  }
  // reset frame state to defaults
  header_ = 0;
  opcode_ = 0;
  frameLenType_ = 0;
  frameLength_ = 0;
  maskingKey_ = 0;
  frameBuf_.clear();
  frameBufOffs_ = 0;
  LOG(INFO, "[WebSocket fd:%d] Reading WS packet", fd_);
  return read_fully_with_timeout(&header_, sizeof(uint16_t)
                               , STATE(frame_header_received));
}

StateFlowBase::Action WebsocketFlow::frame_header_received()
{
  opcode_ = static_cast<WebSocketOpcode>(header_ & 0x0F);
  masked_ = ((header_ >> 8) & 0x80);
  uint8_t len = ((header_ >> 8) & 0x7F);
  LOG(INFO, "[WebSocket fd:%d] opc: %d, masked: %d, len: %d", fd_, opcode_, masked_, len);
  if (len < 126)
  {
    frameLenType_ = 0;
    frameLength_ = len;
    return call_immediately(STATE(frame_data_len_received));
  }
  else if (len == 126)
  {
    frameLenType_ = 1;
    frameLength_ = 0;
    // retrieve the payload length as a 16 bit number
    return read_fully_with_timeout(&frameLength16_, sizeof(uint16_t)
                                 , STATE(frame_data_len_received));
  }
  else if (len == 127)
  {
    frameLenType_ = 2;
    // retrieve the payload length as a 64 bit number
    return read_fully_with_timeout(&frameLength_, sizeof(uint64_t)
                                 , STATE(frame_data_len_received));
  }

  return delete_this();
}

StateFlowBase::Action WebsocketFlow::frame_data_len_received()
{
  if (frameLenType_ == 1)
  {
    // byte swap frameLength16_ into frameLength_
    frameLength_ = (frameLength16_ << 8) | (frameLength16_ >> 8);
  }
  else if (frameLenType_ == 2)
  {
    // byte swap frameLength_ (64 bit)
    uint8_t *p = (uint8_t *)frameLength_;
    uint64_t temp =            p[7]        | (uint16_t)(p[6]) << 8
                  | (uint32_t)(p[5]) << 16 | (uint32_t)(p[4]) << 24
                  | (uint64_t)(p[3]) << 32 | (uint64_t)(p[2]) << 40
                  | (uint64_t)(p[1]) << 48 | (uint64_t)(p[0]) << 56;
    frameLength_ = temp;
  }

  if (masked_)
  {
    // frame uses data masking, read the mask and then start reading the
    // frame payload
    return read_fully_with_timeout(&maskingKey_, sizeof(uint32_t)
                                 , STATE(read_frame_data));
  }
  // no masking, start reading the frame data
  return call_immediately(STATE(read_frame_data));
}

StateFlowBase::Action WebsocketFlow::read_frame_data()
{
  LOG(INFO, "[WebSocket fd:%d] Reading WS packet (%d len)", fd_, (int)frameLength_);
  // restrict the size of the fame buffer so we don't use all of the ram for
  // one frame.
  frameBuf_.resize(std::min(frameLength_, max_frame_size_));
  return read_fully_with_timeout(frameBuf_.data(), frameBuf_.size()
                               , STATE(frame_data_received));
}

StateFlowBase::Action WebsocketFlow::frame_data_received()
{
  size_t received = frameBuf_.size() - helper_.remaining_;
  if (received)
  {
    LOG(VERBOSE, "[WebSocket fd:%d] Received %zu bytes", fd_, received);
    if (masked_)
    {
      uint8_t *mask = (uint8_t *)&maskingKey_;
      char buf[10];
      LOG(VERBOSE, "[WebSocket fd:%d] Demasking %zu bytes (mask: %s)", fd_
        , received, unsigned_integer_to_buffer_hex(maskingKey_, buf));
      for (size_t idx = 0; idx < received; idx++)
      {
        frameBuf_[idx] ^= mask[idx % 4];
      }
      frameBuf_[received] = '\0';
      LOG(VERBOSE, "[WebSocket fd:%d] data: %s", fd_, frameBuf_.data());
    }
    if (opcode_ == PING)
    {
      // send PONG
    }
    else if (opcode_ == TEXT || opcode_ == BINARY)
    {
      handler_(fd_, MESSAGE, opcode_ == TEXT, frameBuf_.data(), received);
    }
    else if (opcode_ == CLOSE)
    {
      sent_disconnect_ = true;
      handler_(fd_, DISCONNECT, false, nullptr, 0);
      return delete_this();
    }
  }
  frameLength_ -= received;
  if (frameLength_)
  {
    frameBuf_.resize(std::min(frameLength_, max_frame_size_));
    return read_fully_with_timeout(frameBuf_.data(), frameBuf_.size()
                                , STATE(frame_data_received));
  }
  return call_immediately(STATE(read_frame_header));
}

} // namespace httpd
} // namespace esp32cs
