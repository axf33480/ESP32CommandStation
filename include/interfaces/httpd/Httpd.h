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

#ifndef HTTPD_H_
#define HTTPD_H_

#include <map>
#include <stdint.h>

#include <hwcrypto/sha.h>
#include <mbedtls/base64.h>

#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>
#include <utils/Singleton.hxx>
#include <utils/socket_listener.hxx>

namespace esp32cs
{
namespace httpd
{

enum HTTP_STATUS_CODE
{
  STATUS_CONTINUE=100,
  STATUS_SWITCH_PROTOCOL=101,

  STATUS_OK=200,
  STATUS_CREATED=201,
  STATUS_ACCEPTED=202,
  STATUS_NON_AUTH_INFO=203,
  STATUS_NO_CONTENT=204,
  STATUS_RESET_CONTENT=205,
  STATUS_PARTIAL_CONTENT=206,

  STATUS_MULTIPLE_CHOICES=300,
  STATUS_MOVED_PERMANENTLY=301,
  STATUS_FOUND=302,
  STATUS_SEE_OTHER=303,
  STATUS_NOT_MODIFIED=304,
  STATUS_USE_PROXY=305,
  STATUS_TEMP_REDIRECT=307,

  STATUS_BAD_REQUEST=400,
  STATUS_NOT_AUTHORIZED=401,
  STATUS_PAYMENT_REQUIRED=402,
  STATUS_FORBIDDEN=403,
  STATUS_NOT_FOUND=404,
  STATUS_NOT_ALLOWED=405,
  STATUS_NOT_ACCEPTABLE=406,
  STATUS_PROXY_AUTH_REQ=407,
  STATUS_TIMEOUT=408,
  STATUS_CONFLICT=409,
  STATUS_GONE=410,
  STATUS_LENGTH_REQ=411,
  STATUS_PRECOND_FAIL=412,
  STATUS_ENTITY_TOO_LARGE=413,
  STATUS_URI_TOO_LARGE=414,
  STATUS_UNSUPPORTED_MEDIA_TYPE=415,
  STATUS_RANGE_NOT_SATISFIABLE=416,
  STATUS_EXPECATION_FAILED=417,

  STATUS_SERVER_ERROR=500,
  STATUS_NOT_IMPLEMENTED=501,
  STATUS_BAD_GATEWAY=502,
  STATUS_SERVICE_UNAVAILABLE=503,
  STATUS_GATEWAY_TIMEOUT=504,
  STATUS_HTTP_VERSION_UNSUPPORTED=505
};

static constexpr const char * HTTP_METHOD_DELETE = "DELETE";
static constexpr const char * HTTP_METHOD_GET = "GET";
static constexpr const char * HTTP_METHOD_HEAD = "HEAD";
static constexpr const char * HTTP_METHOD_POST = "POST";
static constexpr const char * HTTP_METHOD_PATCH = "PATCH";
static constexpr const char * HTTP_METHOD_PUT = "PUT";

static constexpr const char * HTTP_HEADER_ACCEPT = "Accept";
static constexpr const char * HTTP_HEADER_CACHE_CONTROL = "Cache-Control";
static constexpr const char * HTTP_HEADER_CONNECTION = "Connection";
static constexpr const char * HTTP_HEADER_CONTENT_ENCODING = "Content-Encoding";
static constexpr const char * HTTP_HEADER_CONTENT_TYPE = "Content-Type";
static constexpr const char * HTTP_HEADER_CONTENT_LENGTH = "Content-Length";
static constexpr const char * HTTP_HEADER_HOST = "Host";
static constexpr const char * HTTP_HEADER_LOCATION = "Location";

// Values for Cache-Control
static constexpr const char * HTTP_CACHE_CONTROL_NO_CACHE = "no-cache";
static constexpr const char * HTTP_CACHE_CONTROL_NO_STORE = "no-store";
static constexpr const char * HTTP_CACHE_CONTROL_NO_TRANSFORM = "no-transform";
static constexpr const char * HTTP_CACHE_CONTROL_MAX_AGE = "max-age";
static constexpr const char * HTTP_CACHE_CONTROL_PUBLIC = "public";
static constexpr const char * HTTP_CACHE_CONTROL_PRIVATE = "private";
static constexpr const char * HTTP_CACHE_CONTROL_MUST_REVALIDATE = "must-revalidate";

// Values for Connection header
static constexpr const char * HTTP_CONNECTION_CLOSE = "close";
static constexpr const char * HTTP_CONNECTION_KEEP_ALIVE = "keep-alive";

// HTTP end of line characters
static constexpr const char * HTML_EOL = "\r\n";

// Common mime-types
static constexpr const char * MIME_TYPE_TEXT_CSS = "text/css";
static constexpr const char * MIME_TYPE_TEXT_HTML = "text/html";
static constexpr const char * MIME_TYPE_TEXT_PLAIN = "text/plain";
static constexpr const char * MIME_TYPE_TEXT_JAVASCRIPT = "text/javascript";
static constexpr const char * MIME_TYPE_IMAGE_GIF = "image/gif";
static constexpr const char * MIME_TYPE_IMAGE_PNG = "image/png";
static constexpr const char * MIME_TYPE_APPLICATION_JSON = "application/json";

static constexpr const char * HTTP_ENCODING_GZIP = "gzip";

class AbstractHttpResponse
{
public:
  AbstractHttpResponse(HTTP_STATUS_CODE code=STATUS_NOT_FOUND
                     , const std::string &mime_type=MIME_TYPE_TEXT_HTML);
  virtual ~AbstractHttpResponse();
  uint8_t *get_headers(size_t *len, bool keep_alive=false);
  virtual const uint8_t *get_body()
  {
    return nullptr;
  }
  virtual size_t get_body_length()
  {
    return 0;
  }
  std::string get_body_mime_type()
  {
    return mime_type_;
  }
  void add_header(const std::string &header, const std::string &value);
private:
  std::map<std::string, std::string> headers_;
  HTTP_STATUS_CODE code_;
  std::string mime_type_;
  std::string encoded_headers_;
};

class UriNotFoundResponse : public AbstractHttpResponse
{
public:
  UriNotFoundResponse(const std::string &uri)
    : body_(StringPrintf("URI %s was not found.", uri.c_str()))
  {
  }
  virtual const uint8_t *get_body() override final
  {
    return (const uint8_t *)(body_.c_str());
  }
  virtual size_t get_body_length() override final
  {
    return body_.length();
  }
private:
  const std::string body_;
};

class RedirectResponse : public AbstractHttpResponse
{
public:
  RedirectResponse(const std::string &target_uri);
};

class StaticBodyResponse : public AbstractHttpResponse
{
public:
  StaticBodyResponse(const uint8_t *payload, const size_t length
                   , const std::string type, const std::string encoding="")
    : AbstractHttpResponse(STATUS_OK, type), payload_(payload), length_(length)
  {
    if (encoding.length())
    {
      add_header(HTTP_HEADER_CONTENT_ENCODING, encoding);
    }
  }
  virtual const uint8_t *get_body() override final
  {
    return payload_;
  }
  virtual size_t get_body_length() override final
  {
    return length_;
  }
private:
  const uint8_t *payload_;
  const size_t length_;
  const std::string type_;
};

class StringResponse : public AbstractHttpResponse
{
public:
  StringResponse(const std::string &response, const std::string &mime_type);
  virtual const uint8_t *get_body() override final
  {
    return (uint8_t *)response_.c_str();
  }
  virtual size_t get_body_length() override final
  {
    return response_.length();
  }
private:
  std::string response_;
};

class HttpRequest
{
public:
  bool is_valid();
  void set_method(const std::string &value);
  const std::string &get_method();
  const std::string &get_uri();
  void set_uri(const std::string &value);
  void add_param(const std::pair<std::string, std::string> &value);
  void add_header(const std::pair<std::string, std::string> &value);
  bool has_header(const std::string &name);
  const std::string &get_header(const std::string &name);
  uint32_t get_header_uint32(const std::string &name);
  bool get_header_bool(const std::string &name);
  void reset();
  bool keep_alive();
  void set_error(bool value=true);
  bool has_error();
  std::string to_string();
private:
  const std::string blank_header_{""};
  std::map<std::string, std::string> headers_;
  std::map<std::string, std::string> params_;
  std::string method_;
  std::string uri_;
  bool error_;
};

typedef std::function<void(const HttpRequest *, std::shared_ptr<AbstractHttpResponse> &
                         , uint8_t *, uint32_t, uint32_t)> RequestProcessor;

class Httpd : public Service, public Singleton<Httpd>
{
public:
  Httpd(uint16_t port);
  virtual ~Httpd();
  void register_uri(const std::string &uri, RequestProcessor handler);
  void register_redirected_uri(const std::string &source_uri
                             , const std::string &target_uri);
  void register_static_uri(const std::string &uri, const uint8_t *payload
                         , const size_t len, const std::string &type
                         , const std::string &encoding="");
  RequestProcessor get_handler_for_uri(const std::string &uri);
  bool can_send_known_response(const std::string &uri);
  std::shared_ptr<AbstractHttpResponse> get_known_response(const std::string &uri);
  bool is_request_too_large(HttpRequest *req);
  bool is_known_uri(HttpRequest *req);
private:
  void on_new_connection(int fd);
  Executor<1> exec_;
  SocketListener listener_;
  std::map<std::string, RequestProcessor> handlers_;
  std::map<std::string, std::shared_ptr<AbstractHttpResponse>> static_uris_;
  std::map<std::string, std::shared_ptr<AbstractHttpResponse>> redirect_uris_;
  DISALLOW_COPY_AND_ASSIGN(Httpd);
};

class HttpdRequestFlow : public StateFlowBase
{
public:
  HttpdRequestFlow(Httpd *server, int fd);
  virtual ~HttpdRequestFlow();
private:
  StateFlowTimedSelectHelper helper_{this};
  Httpd *server_;
  int fd_;
  HttpRequest req_;
  // flag to close the socket on termination of the request
  bool close_{true};
  std::vector<uint8_t> buf_;
  size_t body_offs_;
  size_t body_len_;
  std::string raw_header_;
  size_t header_parse_pos_{0};
  std::shared_ptr<AbstractHttpResponse> res_;
  size_t response_body_offs_{0};
  uint64_t start_time_;
  uint8_t req_count_{0};

  STATE_FLOW_STATE(start_request);
  STATE_FLOW_STATE(read_more_data);
  STATE_FLOW_STATE(parse_header_data);
  STATE_FLOW_STATE(process_request);
  STATE_FLOW_STATE(process_body_chunk);
  STATE_FLOW_STATE(send_response);
  STATE_FLOW_STATE(send_response_headers);
  STATE_FLOW_STATE(send_response_body);
  STATE_FLOW_STATE(send_response_body_split);
  STATE_FLOW_STATE(request_complete);
  STATE_FLOW_STATE(upgrade_to_websocket);
};

class WebsocketFlow : public StateFlowBase
{
public:
  WebsocketFlow(Httpd *server, int fd) : StateFlowBase(server), fd_(fd)
  {
    start_flow(STATE(read_packet));
  }
private:
  int fd_;

  STATE_FLOW_STATE(read_packet);
  STATE_FLOW_STATE(send_packet);
  STATE_FLOW_STATE(packet_received);
};

static inline std::pair<std::string, std::string> break_string(std::string &str, const std::string& delim)
{
  size_t pos = str.find(delim);
  if (pos == std::string::npos)
  {
    return std::make_pair(str, "");
  }
  return std::make_pair(str.substr(0, pos), str.substr(pos + delim.length()));
}

template <class ContainerT>
std::string::size_type tokenize(const std::string& str, ContainerT& tokens
                              , const std::string& delimeter = " "
                              , bool keepIncomplete=true
                              , bool dropEmpty = false)
{
  std::string::size_type pos, lastPos = 0;

  using value_type = typename ContainerT::value_type;
  using size_type  = typename ContainerT::size_type;

  while(lastPos < str.length())
  {
    pos = str.find(delimeter, lastPos);
    if (pos == std::string::npos)
    {
      if (!keepIncomplete)
      {
        return lastPos;
      }
      pos = str.length();
    }

    if (pos != lastPos || !dropEmpty)
    {
      tokens.emplace_back(value_type(str.data() + lastPos
                                  , (size_type)pos - lastPos));
    }
    lastPos = pos + delimeter.length();
  }
  return lastPos;
}

static inline std::string string_join(const std::vector<std::string>& strings, const std::string& delim = "")
{
  std::string result;

  for (auto piece : strings)
  {
    if (!result.empty())
    {
      result += delim;
    }
    result += piece;
  }
  return result;
}

} // namespace httpd
} // namespace esp32cs

#endif // HTTPD_H_
