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

WebSocketFlow::WebSocketFlow(Httpd *server, int fd, uint32_t remote_ip
                           , const string &ws_key, const string &ws_version
                           , WebSocketHandler handler)
                           : StateFlowBase(server)
                           , server_(server)
                           , fd_(fd)
                           , remote_ip_(remote_ip)
                           , timeout_(MSEC_TO_NSEC(config_httpd_websocket_timeout_ms()))
                           , max_frame_size_(config_httpd_websocket_max_frame_size())
                           , handler_(handler)
{
  server_->add_websocket(fd, this);
  string key_data = ws_key + WEBSOCKET_UUID;
  unsigned char key_sha1[20];

  LOG(VERBOSE, "[WebSocket fd:%d] Connected, starting handshake", fd_);
  // SHA1 encode the ws_key plus the websocket UUID, if this fails close the
  // socket immediately.
  if (!mbedtls_sha1_ret((unsigned char *)key_data.c_str(), key_data.length()
                      , key_sha1))
  {
    AbstractHttpResponse resp(STATUS_SWITCH_PROTOCOL);
    resp.add_header(HTTP_HEADER_CONNECTION, HTTP_CONNECTION_UPGRADE);
    resp.add_header(HTTP_HEADER_UPGRADE, HTTP_UPGRADE_HEADER_WEBSOCKET);
    resp.add_header(WEBSOCKET_HEADER_VERSION, ws_version);
    resp.add_header(WEBSOCKET_HEADER_ACCEPT
                  , base64_encode(string((char *)key_sha1, 20)));
    handshake_.assign(std::move(resp.to_string()));

    // Allocate buffer for frame data.
    data_ = (uint8_t *)malloc(max_frame_size_);
    if (data_)
    {
      start_flow(STATE(send_handshake));
      return;
    }
  }
  LOG_ERROR("[WebSocket fd:%d] Error estabilishing connection, aborting", fd_);
  start_flow(STATE(delete_this));
}

WebSocketFlow::~WebSocketFlow()
{
  // remove ourselves from the server so we don't get called again
  server_->remove_websocket(fd_);

  LOG(INFO, "[WebSocket fd:%d] Disconnected", fd_);
  ::close(fd_);

  if (data_)
  {
    free(data_);
  }
  textToSend_.clear();
}

void WebSocketFlow::send_text(string &text)
{
  OSMutexLock l(&textLock_);
  textToSend_.append(text);
}

StateFlowBase::Action WebSocketFlow::read_fully_with_timeout(void *buf
                                                           , size_t size
                                                           , size_t attempts
                                                           , StateFlowBase::Callback success
                                                           , StateFlowBase::Callback timeout)
{
  buf_ = (uint8_t *)buf;
  buf_offs_ = 0;
  buf_remain_ = size;
  buf_next_ = success;
  buf_next_timeout_ = timeout;
  buf_attempts_ = attempts;
  return read_repeated_with_timeout(&helper_, timeout_, fd_, buf_, size
                                  , STATE(data_received));
}

StateFlowBase::Action WebSocketFlow::data_received()
{
  HASSERT(buf_next_);
  if (helper_.hasError_)
  {
    LOG(INFO, "[WebSocket fd:%d] read-error, disconnecting", fd_);
    return yield_and_call(STATE(shutdown_connection));
  }
  size_t received = (buf_remain_ - helper_.remaining_);
  buf_remain_ -= received;
  buf_offs_ += received;
  LOG(VERBOSE, "[WebSocket fd:%d] Received %zu bytes, %zu bytes remain", fd_
    , received, buf_remain_);
  if (buf_remain_ && buf_attempts_ > 0)
  {
    buf_attempts_--;
    return read_repeated_with_timeout(&helper_, timeout_, fd_, buf_ + buf_offs_
                                    , buf_remain_, STATE(data_received));
  }
  else if (buf_remain_)
  {
    return yield_and_call(buf_next_timeout_);
  }
  return yield_and_call(buf_next_);
}

StateFlowBase::Action WebSocketFlow::send_handshake()
{
  LOG(VERBOSE, "[WebSocket fd:%d] Sending handshake:\n%s", fd_
    , handshake_.c_str());
  return write_repeated(&helper_, fd_, handshake_.c_str(), handshake_.length()
                      , STATE(handshake_sent));
}

StateFlowBase::Action WebSocketFlow::handshake_sent()
{
  handshake_.clear();
  if (helper_.hasError_)
  {
    LOG(INFO, "[WebSocket fd:%d] read-error, disconnecting", fd_);
    return yield_and_call(STATE(shutdown_connection));
  }
  handler_(fd_, remote_ip_, CONNECT, false, nullptr, 0);
  return yield_and_call(STATE(read_frame_header));
}

StateFlowBase::Action WebSocketFlow::read_frame_header()
{
  // reset frame state to defaults
  header_ = 0;
  opcode_ = 0;
  frameLenType_ = 0;
  frameLength_ = 0;
  maskingKey_ = 0;
  bzero(data_, max_frame_size_);
  LOG(VERBOSE, "[WebSocket fd:%d] Reading WS packet", fd_);
  return read_fully_with_timeout(&header_, sizeof(uint16_t)
                               , config_httpd_websocket_max_read_attempts()
                               , STATE(frame_header_received)
                               , STATE(send_frame_header));
}

StateFlowBase::Action WebSocketFlow::frame_header_received()
{
  opcode_ = static_cast<WebSocketOpcode>(header_ & 0x0F);
  masked_ = ((header_ >> 8) & 0x80);
  uint8_t len = ((header_ >> 8) & 0x7F);
  LOG(VERBOSE, "[WebSocket fd:%d] opc: %d, masked: %d, len: %d", fd_, opcode_
    , masked_, len);
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
                                 , config_httpd_websocket_max_read_attempts()
                                 , STATE(frame_data_len_received)
                                 , STATE(shutdown_connection));
  }
  else if (len == 127)
  {
    frameLenType_ = 2;
    // retrieve the payload length as a 64 bit number
    return read_fully_with_timeout(&frameLength_, sizeof(uint64_t)
                                 , config_httpd_websocket_max_read_attempts()
                                 , STATE(frame_data_len_received)
                                 , STATE(shutdown_connection));
  }

  // if we get here the frame is malformed, shutdown the connection
  return yield_and_call(STATE(shutdown_connection));
}

StateFlowBase::Action WebSocketFlow::frame_data_len_received()
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
                                 , config_httpd_websocket_max_read_attempts()
                                 , STATE(start_recv_frame_data)
                                 , STATE(shutdown_connection));
  }
  // no masking, start reading the frame data
  return yield_and_call(STATE(start_recv_frame_data));
}

StateFlowBase::Action WebSocketFlow::start_recv_frame_data()
{
  LOG(VERBOSE, "[WebSocket fd:%d] Reading WS packet (%d len)", fd_
    , (int)frameLength_);
  // restrict the size of the fame buffer so we don't use all of the ram for
  // one frame.
  data_size_ = std::min(frameLength_, max_frame_size_);
  return read_fully_with_timeout(data_, data_size_
                               , config_httpd_websocket_max_read_attempts()
                               , STATE(recv_frame_data)
                               , STATE(send_frame_header));
}

StateFlowBase::Action WebSocketFlow::recv_frame_data()
{
  size_t received_len = data_size_ - helper_.remaining_;
  if (received_len)
  {
    LOG(VERBOSE, "[WebSocket fd:%d] Received %zu bytes", fd_, received_len);
    if (masked_)
    {
      uint8_t *mask = reinterpret_cast<uint8_t *>(&maskingKey_);
      char buf[10];
      LOG(VERBOSE, "[WebSocket fd:%d] Demasking %zu bytes (mask: %s)", fd_
        , received_len, unsigned_integer_to_buffer_hex(maskingKey_, buf));
      for (size_t idx = 0; idx < received_len; idx++)
      {
        data_[idx] ^= mask[idx % 4];
      }
    }
    if (opcode_ == PING)
    {
      // send PONG
    }
    else if (opcode_ == TEXT || opcode_ == BINARY)
    {
      handler_(fd_, remote_ip_, MESSAGE, opcode_ == TEXT, data_
             , received_len);
    }
    else if (opcode_ == CLOSE)
    {
      return yield_and_call(STATE(shutdown_connection));
    }
  }
  frameLength_ -= received_len;
  if (frameLength_)
  {
    data_size_ = std::min(frameLength_, max_frame_size_);
    return read_fully_with_timeout(data_, data_size_
                                 , config_httpd_websocket_max_read_attempts()
                                 , STATE(recv_frame_data)
                                 , STATE(shutdown_connection));
  }
  return yield_and_call(STATE(read_frame_header));
}

StateFlowBase::Action WebSocketFlow::shutdown_connection()
{
  handler_(fd_, remote_ip_, DISCONNECT, false, nullptr, 0);
  return delete_this();
}

StateFlowBase::Action WebSocketFlow::send_frame_header()
{
  OSMutexLock l(&textLock_);
  if (textToSend_.empty())
  {
    return yield_and_call(STATE(read_frame_header));
  }
  bzero(data_, max_frame_size_);
  size_t send_size = 0;
  if (textToSend_.length() < 126)
  {
    data_[0] = 0x80 | TEXT;
    data_[1] = textToSend_.length();
    memcpy(data_ + 2, textToSend_.data(), textToSend_.length());
    data_size_ = textToSend_.length();
    send_size = data_size_ + 4;
  }
  else if (textToSend_.length() < data_size_ - 4)
  {
    data_[0] = 0x80 | TEXT;
    data_[1] = 0;// use extended length
    data_[2] = textToSend_.length() & 0xFF;
    data_[3] = (textToSend_.length() >> 8) & 0xFF;
    memcpy(data_+ 4, textToSend_.data(), textToSend_.length());
    data_size_ = textToSend_.length();
    send_size = data_size_ + 4;
  }
  else
  {
    LOG_ERROR("[WebSocket fd:%d] text to send is too long, discarding", fd_);
    textToSend_.clear();
    return yield_and_call(STATE(read_frame_header));
  }
  return write_repeated(&helper_, fd_, data_, send_size, STATE(frame_sent));
}

StateFlowBase::Action WebSocketFlow::frame_sent()
{
  if (helper_.hasError_)
  {
    return yield_and_call(STATE(shutdown_connection));
  }
  OSMutexLock l(&textLock_);
  textToSend_.erase(0, data_size_);
  if (textToSend_.empty())
  {
    return yield_and_call(STATE(read_frame_header));
  }
  return yield_and_call(STATE(send_frame_header));
}

} // namespace httpd
} // namespace esp32cs
