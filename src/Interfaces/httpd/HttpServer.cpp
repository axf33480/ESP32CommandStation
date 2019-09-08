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

Httpd::Httpd(uint16_t port) : Service(&exec_)
                            , exec_("httpd", config_httpd_server_priority()
                                  , config_httpd_server_stack_size())
                            , listener_(port
                                      , std::bind(&Httpd::on_new_connection
                                      , this, std::placeholders::_1))
{
}

Httpd::~Httpd()
{
  listener_.shutdown();
  exec_.shutdown();
}

void Httpd::on_new_connection(int fd)
{
  new HttpdRequestFlow(this, fd);
}

void Httpd::register_uri(const std::string &uri, RequestProcessor handler)
{
  handlers_.emplace(std::make_pair(std::move(uri), std::move(handler)));
}

RequestProcessor Httpd::get_handler_for_uri(const std::string &uri)
{
  LOG(VERBOSE, "[Httpd] Searching for URI handler: %s", uri.c_str());
  if (handlers_.find(uri) != handlers_.end())
  {
    return handlers_[uri];
  }
  LOG(VERBOSE, "[Httpd] No handler found");
  return nullptr;
}

void Httpd::register_redirected_uri(const string &source_uri
                                  , const string &target_uri)
{
  redirect_uris_.emplace(
    std::make_pair(std::move(source_uri)
                 , std::make_shared<RedirectResponse>(target_uri)));
}

void Httpd::register_static_uri(const string &uri, const uint8_t *payload
                              , const size_t length, const string &type
                              , const string &encoding)
{
  static_uris_.emplace(
    std::make_pair(std::move(uri)
                 , std::make_shared<StaticBodyResponse>(payload, length, type
                                                      , encoding)));
}

bool Httpd::can_send_known_response(const string &uri)
{
  return static_uris_.count(uri) || redirect_uris_.count(uri);
}

shared_ptr<AbstractHttpResponse> Httpd::get_known_response(const string &uri)
{
  if (static_uris_.count(uri))
  {
    return static_uris_[uri];
  }
  else if (redirect_uris_.count(uri))
  {
    return redirect_uris_[uri];
  }
  return nullptr;
}

bool Httpd::is_request_too_large(HttpRequest *req)
{
  HASSERT(req);
  uint32_t req_body_len = req->get_header_uint32(HTTP_HEADER_CONTENT_LENGTH);
  if (req_body_len > config_httpd_max_req_size())
  {
    LOG_ERROR("[Httpd uri:%s] Request body too large %d > %d!"
            , req->get_uri().c_str(), req_body_len
            , config_httpd_max_req_size());
    // request size too big
    return true;
  }
  return false;
}

bool Httpd::is_known_uri(HttpRequest *req)
{
  HASSERT(req);
  if (!req->get_method().compare(HTTP_METHOD_GET) &&
      can_send_known_response(req->get_uri()))
  {
    // check for known URI responses
    return true;
  }

  // default to checking if we have a handler
  return handlers_.count(req->get_uri());
}

} // namespace httpd
} // namespace esp32cs
