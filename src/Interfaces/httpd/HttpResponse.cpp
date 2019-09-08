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

static std::map<HTTP_STATUS_CODE, string> http_code_strings =
{
  {STATUS_CONTINUE, "Continue"},
  {STATUS_SWITCH_PROTOCOL, "Switching Protocols"},

  {STATUS_OK, "OK"},
  {STATUS_CREATED, "Created"},
  {STATUS_ACCEPTED, "Accepted"},
  {STATUS_NON_AUTH_INFO, "Non-Authoritative Information"},
  {STATUS_NO_CONTENT, "No Content"},
  {STATUS_RESET_CONTENT, "Reset Content"},
  {STATUS_PARTIAL_CONTENT, "Partial Content"},

  {STATUS_MULTIPLE_CHOICES, "Multiple Choices"},
  {STATUS_MOVED_PERMANENTLY, "Moved Permanently"},
  {STATUS_FOUND, "Found"},
  {STATUS_SEE_OTHER, "See Other"},
  {STATUS_NOT_MODIFIED, "Not Modified"},
  {STATUS_USE_PROXY, "Use Proxy"},
  {STATUS_TEMP_REDIRECT, "Temporary Redirect"},

  {STATUS_BAD_REQUEST, "Bad Request"},
  {STATUS_NOT_AUTHORIZED, "Unauthorized"},
  {STATUS_PAYMENT_REQUIRED, "Payment Required"},
  {STATUS_FORBIDDEN, "Forbidden"},
  {STATUS_NOT_FOUND, "Not Found"},
  {STATUS_NOT_ALLOWED, "Method Not Allowed"},
  {STATUS_NOT_ACCEPTABLE, "Not Acceptable"},
  {STATUS_PROXY_AUTH_REQ, "Proxy Authentication Required"},
  {STATUS_TIMEOUT, "Request Time-out"},
  {STATUS_CONFLICT, "Conflict"},
  {STATUS_GONE, "Gone"},
  {STATUS_LENGTH_REQ, "Length Required"},
  {STATUS_PRECOND_FAIL, "Precondition Failed"},
  {STATUS_ENTITY_TOO_LARGE, "Request Entity Too Large"},
  {STATUS_URI_TOO_LARGE, "Request-URI Too Large"},
  {STATUS_UNSUPPORTED_MEDIA_TYPE, "Unsupported Media Type"},
  {STATUS_RANGE_NOT_SATISFIABLE, "Requested range not satisfiable"},
  {STATUS_EXPECATION_FAILED, "Expectation Failed"},

  {STATUS_SERVER_ERROR, "Internal Server Error"},
  {STATUS_NOT_IMPLEMENTED, "Not Implemented"},
  {STATUS_BAD_GATEWAY, "Bad Gateway"},
  {STATUS_SERVICE_UNAVAILABLE, "Service Unavailable"},
  {STATUS_GATEWAY_TIMEOUT, "Gateway Time-out"},
  {STATUS_HTTP_VERSION_UNSUPPORTED, "HTTP Version not supported"}
};

AbstractHttpResponse::AbstractHttpResponse(HTTP_STATUS_CODE code
                                         , const string &mime_type)
                                         : code_(code), mime_type_(mime_type)
                                         , encoded_headers_("")
{
  // seed default headers
  add_header(HTTP_HEADER_CACHE_CONTROL, HTTP_CACHE_CONTROL_NO_CACHE);
}

AbstractHttpResponse::~AbstractHttpResponse()
{
  encoded_headers_.clear();
  headers_.clear();
}

uint8_t *AbstractHttpResponse::get_headers(size_t *len, bool keep_alive)
{
  encoded_headers_.assign(StringPrintf("HTTP/1.1 %d %s%s", code_
                                      , http_code_strings[code_].c_str()
                                      , HTML_EOL));
  for (auto &ent : headers_)
  {
    LOG(VERBOSE, "[resp-header] %s -> %s", ent.first.c_str()
      , ent.second.c_str());
    encoded_headers_.append(
      StringPrintf("%s: %s%s", ent.first.c_str(), ent.second.c_str()
                  , HTML_EOL));
  }
  LOG(VERBOSE, "[resp-header] %s -> %zu", HTTP_HEADER_CONTENT_LENGTH
    , get_body_length());
  encoded_headers_.append(
    StringPrintf("%s: %zu%s", HTTP_HEADER_CONTENT_LENGTH, get_body_length()
                , HTML_EOL));

  if (get_body_length())
  {
    LOG(VERBOSE, "[resp-header] %s -> %s", HTTP_HEADER_CONTENT_TYPE
      , get_body_mime_type().c_str());
    encoded_headers_.append(
      StringPrintf("%s: %s%s", HTTP_HEADER_CONTENT_TYPE
                  , get_body_mime_type().c_str(), HTML_EOL));
  }

  string connection = keep_alive ? HTTP_CONNECTION_CLOSE
                                 : HTTP_CONNECTION_KEEP_ALIVE;
  LOG(VERBOSE, "[resp-header] %s -> %s", HTTP_HEADER_CONNECTION, connection.c_str());
  encoded_headers_.append(
    StringPrintf("%s: %s%s", HTTP_HEADER_CONNECTION, connection.c_str()
                , HTML_EOL));

  // leave blank line after headers before the body
  encoded_headers_.append(HTML_EOL);

  *len = encoded_headers_.size();
  return (uint8_t *)encoded_headers_.c_str();
}

void AbstractHttpResponse::add_header(const string &header, const string &value)
{
  headers_[header] = std::move(value);
}

RedirectResponse::RedirectResponse(const string &target_uri)
  : AbstractHttpResponse(HTTP_STATUS_CODE::STATUS_FOUND)
{
  add_header(HTTP_HEADER_LOCATION, target_uri);
}

StringResponse::StringResponse(const string &response, const string &mime_type)
  : AbstractHttpResponse(HTTP_STATUS_CODE::STATUS_OK, mime_type)
  , response_(std::move(response))
{
}

} // namespace httpd
} // namespace esp32cs
