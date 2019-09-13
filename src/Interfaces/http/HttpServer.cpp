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
#include "interfaces/http/Http.h"

#ifdef ESP32

// this method is not exposed via the MDNS class today, declare it here so we
// can call it if needed. This is implemented inside Esp32WiFiManager.cxx.
void mdns_unpublish(const char *service);

#endif // ESP32

namespace esp32cs
{
namespace http
{

/// Callback for a newly accepted socket connection.
///
/// @param fd is the socket handle.
void incoming_http_connection(int fd)
{
  Singleton<Httpd>::instance()->new_connection(fd);
}

Httpd::Httpd(MDNS *mdns, uint16_t port, const string &name, const string service_name)
  : Service(&executor_)
  , name_(name)
  , mdns_(mdns)
  , mdns_service_(service_name)
  , executor_(name.c_str(), config_httpd_server_priority()
            , config_httpd_server_stack_size())
  , port_(port)
{
#ifdef ESP32
  // Hook into the Esp32WiFiManager to start/stop the listener automatically
  // based on the AP/Station interface status.
  Singleton<Esp32WiFiManager>::instance()->add_event_callback(
  [&](system_event_t *event)
  {
    if (event->event_id == SYSTEM_EVENT_STA_GOT_IP ||
        event->event_id == SYSTEM_EVENT_AP_START)
    {
      // If it is the SoftAP interface, start the dns server
      if (event->event_id == SYSTEM_EVENT_AP_START)
      {
        tcpip_adapter_ip_info_t ip_info;
        tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info);
        start_dns_listener(ntohl(ip_info.ip.addr));
      }
      start_http_listener();
    }
    else if (event->event_id == SYSTEM_EVENT_STA_LOST_IP ||
             event->event_id == SYSTEM_EVENT_AP_STOP)
    {
      stop_http_listener();
      stop_dns_listener();
    }
  });
#endif // ESP32
}

Httpd::~Httpd()
{
  stop_http_listener();
  stop_dns_listener();
  executor_.shutdown();
  handlers_.clear();
  static_uris_.clear();
  redirect_uris_.clear();
  websocket_uris_.clear();
}

void Httpd::uri(const std::string &uri, const size_t method_mask
              , RequestProcessor handler, StreamProcessor stream_handler)
{
  handlers_.insert(
    std::make_pair(uri, std::make_pair(method_mask, std::move(handler))));
  if (stream_handler)
  {
    stream_handlers_.insert(std::make_pair(uri, std::move(stream_handler)));
  }
}

void Httpd::uri(const std::string &uri, RequestProcessor handler)
{
  this->uri(uri, 0xFFFF, handler, nullptr);
}

void Httpd::redirect_uri(const string &source, const string &target)
{
  redirect_uris_.insert(
    std::make_pair(std::move(source)
                 , std::make_shared<RedirectResponse>(target)));
}

void Httpd::static_uri(const string &uri, const uint8_t *payload
                     , const size_t length, const string &mime_type
                     , const string &encoding)
{
  static_uris_.insert(
    std::make_pair(std::move(uri)
                 , std::make_shared<StaticResponse>(payload, length, mime_type
                                                  , encoding)));
}

void Httpd::websocket_uri(const string &uri, WebSocketHandler handler)
{
  websocket_uris_.insert(std::make_pair(std::move(uri), std::move(handler)));
}

void Httpd::send_websocket_binary(int id, uint8_t *data, size_t len)
{
  if (!websockets_.count(id))
  {
    LOG_ERROR("[Httpd] Attempt to send data to unknown websocket:%d, "
              "discarding.", id);
    return;
  }
  //websockets_[id]->send_binary(data, len);
}

void Httpd::send_websocket_text(int id, std::string &text)
{
  if (!websockets_.count(id))
  {
    LOG_ERROR("[Httpd] Attempt to send text to unknown websocket:%d, "
              "discarding text.", id);
    return;
  }
  websockets_[id]->send_text(text);
}

void Httpd::new_connection(int fd)
{
  sockaddr_in source;
  socklen_t source_len = sizeof(sockaddr_in);
  if (getpeername(fd, (sockaddr *)&source, &source_len))
  {
    source.sin_addr.s_addr = 0;
  }
  LOG(INFO, "[%s fd:%d/%s] Connected", name_.c_str(), fd
    , ipv4_to_string(ntohl(source.sin_addr.s_addr)).c_str());
  struct timeval tm;
  tm.tv_sec = 0;
  tm.tv_usec = MSEC_TO_USEC(config_httpd_socket_timeout_ms());
  ERRNOCHECK("setsockopt_recv_timeout",
      setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tm, sizeof(tm)));
  ERRNOCHECK("setsockopt_send_timeout",
      setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tm, sizeof(tm)));
  new HttpRequestFlow(this, fd, ntohl(source.sin_addr.s_addr));
}

void Httpd::captive_portal(string first_access_response
                         , string auth_uri, uint64_t auth_timeout)
{
  captive_response_.assign(std::move(first_access_response));
  captive_auth_uri_.assign(std::move(auth_uri));
  captive_timeout_ = auth_timeout;
  captive_active_ = true;
}

void Httpd::start_http_listener()
{
  if (http_active_)
  {
    return;
  }
  LOG(INFO, "[%s] Starting HTTP listener on port %d", name_.c_str(), port_);
  listener_.emplace(port_, incoming_http_connection);
  http_active_ = true;
  if (mdns_)
  {
    mdns_->publish(name_.c_str(), mdns_service_.c_str(), port_);
  }
}

void Httpd::start_dns_listener(uint32_t ip)
{
  if (dns_active_)
  {
    return;
  }
  LOG(INFO, "[%s] Starting DNS listener", name_.c_str());
  dns_.emplace(ip);
  dns_active_ = true;
}

void Httpd::stop_http_listener()
{
  if (http_active_)
  {
    LOG(INFO, "[%s] Shutting down HTTP listener", name_.c_str());
    listener_.reset();
    http_active_ = false;
#ifdef ESP32
    if (mdns_)
    {
      mdns_unpublish(mdns_service_.c_str());
    }
#endif
  }
}

void Httpd::stop_dns_listener()
{
  if (dns_active_)
  {
    LOG(INFO, "[%s] Shutting down HTTP listener", name_.c_str());
    dns_.reset();
    dns_active_ = false;
  }
}

void Httpd::add_websocket(int id, WebSocketFlow *ws)
{
  websockets_[id] = ws;
}

void Httpd::remove_websocket(int id)
{
  websockets_.erase(id);
}

bool Httpd::have_known_response(const string &uri)
{
  return static_uris_.count(uri) || redirect_uris_.count(uri);
}

shared_ptr<AbstractHttpResponse> Httpd::response(const string &uri)
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

  // if the request doesn't have the content-length header we can try to
  // process it but it likely will fail.
  if (req->has_header(HttpHeader::CONTENT_LENGTH))
  {
    uint32_t len = std::stoul(req->header(HttpHeader::CONTENT_LENGTH));
    if (len > config_httpd_max_req_size())
    {
      LOG_ERROR("[Httpd uri:%s] Request body too large %d > %d!"
              , req->uri().c_str(), len, config_httpd_max_req_size());
      // request size too big
      return true;
    }
  }
  else
  {
    LOG(VERBOSE, "[Httpd] Request does not have Content-Length header:\n%s"
      , req->to_string().c_str());
  }

  return false;
}

bool Httpd::is_servicable_uri(HttpRequest *req)
{
  HASSERT(req);

  // check if it is a GET of a known URI or if it is a Websocket URI
  if ((req->method() == HttpMethod::GET && have_known_response(req->uri())) ||
      websocket_uris_.count(req->uri()))
  {
    return true;
  }

  // check if it is a POST/PUT and there is a body payload
  if ((req->method() & (HttpMethod::POST | HttpMethod::PUT)) &&
      req->has_header(HttpHeader::CONTENT_LENGTH))
  {
    return stream_handler(req->uri()) != nullptr;
  }

  // Check if we have a handler for the provided URI
  return handler(req->method(), req->uri()) != nullptr;
}

RequestProcessor Httpd::handler(HttpMethod method, const std::string &uri)
{
  LOG(VERBOSE, "[Httpd uri:%s] Searching for URI handler", uri.c_str());
  if (handlers_.count(uri))
  {
    auto handler = handlers_[uri];
    if (method & handler.first)
    {
      return handler.second;
    }
  }
  LOG(VERBOSE, "[Httpd uri:%s] No suitable handler found", uri.c_str());
  return nullptr;
}

StreamProcessor Httpd::stream_handler(const std::string &uri)
{
  LOG(VERBOSE, "[Httpd uri:%s] Searching for URI stream handler", uri.c_str());
  if (stream_handlers_.count(uri))
  {
    return stream_handlers_[uri];
  }
  LOG(VERBOSE, "[Httpd uri:%s] No suitable stream handler found", uri.c_str());
  return nullptr;
}

WebSocketHandler Httpd::ws_handler(const string &uri)
{
  if (websocket_uris_.count(uri))
  {
    return websocket_uris_[uri];
  }
  return nullptr;
}

} // namespace http
} // namespace esp32cs
