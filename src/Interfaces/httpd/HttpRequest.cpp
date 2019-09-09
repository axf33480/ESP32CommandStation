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

bool HttpRequest::is_valid()
{
  return !method_.empty() && !uri_.empty();
}

void HttpRequest::set_method(const string &value)
{
  LOG(VERBOSE, "[HttpReq %p] Setting Method: %s", this, value.c_str());
  method_.assign(std::move(value));
}

const string &HttpRequest::get_method()
{
  return method_;
}

const string &HttpRequest::get_uri()
{
  return uri_;
}

void HttpRequest::set_uri(const string &value)
{
  LOG(VERBOSE, "[HttpReq %p] Setting URI: %s", this, value.c_str());
  uri_.assign(std::move(value));
}

void HttpRequest::add_param(const std::pair<string, string> &value)
{
  LOG(VERBOSE, "[HttpReq %p] Adding Param: %s: %s", this, value.first.c_str()
    , value.second.c_str());
  params_.emplace(std::move(value));
}

void HttpRequest::add_header(const std::pair<std::string, std::string> &value)
{
  LOG(VERBOSE, "[HttpReq %p] Adding Header: %s: %s", this, value.first.c_str()
    , value.second.c_str());
  headers_.emplace(value);
}

bool HttpRequest::has_header(const string &name)
{
  return headers_.find(name) != headers_.end();
}

const string &HttpRequest::get_header(const string &name)
{
  if (!has_header(name))
  {
    return blank_header_;
  }
  return headers_[name];
}

uint32_t HttpRequest::get_header_uint32(const string &name)
{
  if (!has_header(name))
  {
    return 0;
  }
  return std::stoul(headers_[name]);
}

bool HttpRequest::get_header_bool(const string &name)
{
  if (!has_header(name))
  {
    return false;
  }
  if (!headers_[name].compare("true"))
  {
    return true;
  }
  else if (!headers_[name].compare("false"))
  {
    return true;
  }
  return get_header_uint32(name);
}

void HttpRequest::reset()
{
  LOG(VERBOSE, "[HttpReq %p] Resetting to blank request", this);
  headers_.clear();
  params_.clear();
  method_.clear();
  uri_.clear();
  error_ = false;
}

bool HttpRequest::keep_alive()
{
  if (!has_header(HTTP_HEADER_CONNECTION))
  {
    return false;
  }
  return headers_[HTTP_HEADER_CONNECTION].compare(HTTP_CONNECTION_CLOSE);
}

void HttpRequest::set_error(bool value)
{
  LOG(VERBOSE, "[HttpReq %p] Setting error flag to %d", this, value);
  error_ = value;
}

bool HttpRequest::has_error()
{
  return error_;
}

string HttpRequest::to_string()
{
  string res = StringPrintf("[HttpReq %p] method:%s uri:%s,error:%d,"
                            "header-count:%zu,param-count:%zu"
                          , this, method_.c_str(), uri_.c_str(), error_
                          , headers_.size(), params_.size());
  for (auto &ent : headers_)
  {
    res.append(
      StringPrintf("\nheader: %s: %s%s", ent.first.c_str(), ent.second.c_str()
                  , HTML_EOL));
  }
  for (auto &ent : params_)
  {
    res.append(
      StringPrintf("\nparam: %s: %s%s", ent.first.c_str(), ent.second.c_str()
                  , HTML_EOL));
  }
  return res;
}

} // namespace httpd
} // namespace esp32cs
