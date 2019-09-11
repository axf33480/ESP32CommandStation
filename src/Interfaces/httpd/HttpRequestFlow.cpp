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
namespace http
{

HttpRequestFlow::HttpRequestFlow(Httpd *server, int fd
                               , uint32_t remote_ip)
                               : StateFlowBase(server)
                               , server_(server)
                               , fd_(fd)
                               , remote_ip_(remote_ip)
{
  start_flow(STATE(start_request));
  LOG(VERBOSE, "[Httpd fd:%d] Connected.", fd_);
}

HttpRequestFlow::~HttpRequestFlow()
{
  if (close_)
  {
    LOG(VERBOSE, "[Httpd fd:%d] Closed", fd_);
    ::close(fd_);
  }
}

StateFlowBase::Action HttpRequestFlow::start_request()
{
  LOG(VERBOSE, "[Httpd fd:%d] reading header", fd_);
  req_.reset();
  raw_header_.assign("");
  start_time_ = esp_timer_get_time();
  buf_.resize(config_httpd_header_chunk_size());
  return read_repeated_with_timeout(&helper_
                                  , MSEC_TO_NSEC(config_httpd_req_timeout_ms())
                                  , fd_, buf_.data()
                                  , config_httpd_header_chunk_size()
                                  , STATE(parse_header_data));
}

StateFlowBase::Action HttpRequestFlow::parse_header_data()
{
  if (helper_.hasError_)
  {
    req_.error(true);
    return yield_and_call(STATE(request_complete));
  }
  else if (helper_.remaining_ == config_httpd_header_chunk_size())
  {
    return yield_and_call(STATE(read_more_data));
  }

  raw_header_.append((char *)buf_.data()
                    , config_httpd_header_chunk_size() - helper_.remaining_);

  // if we don't have an EOL string in the data yet, get more data
  if (raw_header_.find(HTML_EOL) == string::npos)
  {
    return yield_and_call(STATE(read_more_data));
  }

  // parse the data we have into delimited lines of header data, this will
  // leave some data in the raw_header_ which will need to be pushed back
  // into the buf_ after we reach the body segment.
  vector<string> lines;
  size_t parsed = tokenize(raw_header_, lines, HTML_EOL, false);

  // drop whatever has been tokenized so we don't process it again
  raw_header_.erase(0, parsed);

  // the first line is always the request line
  if (req_.raw_method().empty())
  {
    vector<string> request;
    tokenize(lines[0], request, " ");
    if (request.size() != 3)
    {
      req_.error(true);
      LOG_ERROR("[Httpd fd:%d] Malformed request: %s.", fd_
              , lines[0].c_str());
      res_.reset(new AbstractHttpResponse(HttpStatusCode::STATUS_BAD_REQUEST));
      return yield_and_call(STATE(send_response_headers));
    }
    req_.method(request[0]);
    vector<string> uri_parts;
    tokenize(request[1], uri_parts, "?");
    req_.uri(uri_parts[0]);
    uri_parts.erase(uri_parts.begin());
    for (string &part : uri_parts)
    {
      vector<string> params;
      tokenize(part, params, "&");
      for (auto param : params)
      {
        req_.param(break_string(param, "="));
      }
    }
    
    if (server_->is_request_too_large(&req_))
    {
      LOG_ERROR("[Httpd fd:%d,uri:%s] Request body is too large, "
                "aborting with status 400", fd_, req_.uri().c_str());
      res_.reset(new AbstractHttpResponse(HttpStatusCode::STATUS_BAD_REQUEST));
      return yield_and_call(STATE(send_response_headers));
    }
    
    if (!server_->is_servicable_uri(&req_))
    {
      LOG_ERROR("[Httpd fd:%d,uri:%s] Unknown URI, sending 404", fd_
              , req_.uri().c_str());
      res_.reset(new UriNotFoundResponse(req_.uri()));
      return yield_and_call(STATE(send_response_headers));
    }

    // remove the first line since we processed it
    lines.erase(lines.begin());
  }

  // process any remaining lines as headers until we reach a blank line
  for (auto &line : lines)
  {
    // check if we have reached a blank line, this is immediately after the
    // last header in the request.
    if (line.empty())
    {
      return yield_and_call(STATE(process_request));
    }
    else
    {
      // it appears to be a header entry, break it and store it in the request
      req_.header(break_string(line, ": "));
    }
  }

  return yield_and_call(STATE(read_more_data));
}

StateFlowBase::Action HttpRequestFlow::process_request()
{
  LOG(VERBOSE, "[Httpd fd:%d] %s", fd_, req_.to_string().c_str());

  if (!req_.header(HttpHeader::UPGRADE).compare(HTTP_UPGRADE_HEADER_WEBSOCKET))
  {
    // upgrade to websocket!
    if (!req_.has_header(HttpHeader::WS_VERSION) ||
        !req_.has_header(HttpHeader::WS_KEY))
    {
      LOG_ERROR("[Httpd fd:%d,uri:%s] Missing required websocket header(s)\n%s"
              , fd_, req_.uri().c_str(), req_.to_string().c_str());
      res_.reset(new AbstractHttpResponse(HttpStatusCode::STATUS_BAD_REQUEST));
      return yield_and_call(STATE(send_response_headers));
    }
    return yield_and_call(STATE(upgrade_to_websocket));
  }

  // if it is a PUT or POST and there is a content-length header we need to
  // read the body of the message in chunks and pass it off to the request
  // handler.
  if ((req_.method() == HttpMethod::POST ||
       req_.method() == HttpMethod::PUT) &&
      std::stoul(req_.header(HttpHeader::CONTENT_LENGTH)))
  {
    // resize the buffer for max chunk size
    buf_.resize(config_httpd_body_chunk_size());
    buf_.clear();

    // extract the body length from the content length header
    body_len_ = std::stoul(req_.header(HttpHeader::CONTENT_LENGTH));
    size_t requested_size = config_httpd_body_chunk_size();
    if (body_len_ < requested_size)
    {
      requested_size = body_len_;
    }
    if (!raw_header_.empty())
    {
      size_t offs = raw_header_.length();
      // we have some of the body already read in
      requested_size -= offs;
      // move the raw unparsed header bits back to the buffer
      std::move(raw_header_.begin(), raw_header_.end()
              , std::back_inserter(buf_));
      // we have received the entire body already, start processing it
      if (requested_size <= 0)
      {
        return yield_and_call(STATE(process_body_chunk));
      }
      // we don't have the payload, try to get more data before processing
      return read_repeated_with_timeout(&helper_
                                      , MSEC_TO_NSEC(config_httpd_req_timeout_ms())
                                      , fd_, buf_.data() + offs, requested_size
                                      , STATE(process_body_chunk));
    }
    // read the payload and process it in chunks
    return read_repeated_with_timeout(&helper_
                                    , MSEC_TO_NSEC(config_httpd_req_timeout_ms())
                                    , fd_, buf_.data(), requested_size
                                    , STATE(process_body_chunk));
  }
  else if (server_->have_known_response(req_.uri()))
  {
    res_ = server_->response(req_.uri());
    LOG(VERBOSE, "[Httpd fd:%d,uri:%s] Processing response (%d bytes)", fd_
      , req_.uri().c_str(), res_->get_body_length());
  }
  else
  {
    RequestProcessor handler = server_->handler(req_.uri());
    res_.reset(handler(&req_, nullptr, 0));
  }

  return yield_and_call(STATE(send_response_headers));
}

StateFlowBase::Action HttpRequestFlow::read_more_data()
{
  // we need more data to parse the request
  LOG(VERBOSE, "[Httpd fd:%d] Requesting more data to process request", fd_);
  return read_repeated_with_timeout(&helper_
                              , MSEC_TO_NSEC(config_httpd_req_timeout_ms())
                              , fd_, buf_.data()
                              , config_httpd_header_chunk_size()
                              , STATE(parse_header_data));
}

StateFlowBase::Action HttpRequestFlow::process_body_chunk()
{
  if (helper_.hasError_)
  {
    req_.error(true);
    return yield_and_call(STATE(request_complete));
  }
  size_t data_len = config_httpd_body_chunk_size() - helper_.remaining_;
  // if we received some data pass it on to the handler
  if (data_len)
  {
    RequestProcessor handler = server_->handler(req_.uri());
    body_offs_ += data_len;
    res_.reset(handler(&req_, buf_.data(), data_len));
  }
  if (body_offs_ < body_len_)
  {
    size_t requested_size = config_httpd_body_chunk_size();
    if ((body_len_ - body_offs_) < requested_size)
    {
      requested_size = body_len_ - body_offs_;
    }
    return read_repeated_with_timeout(&helper_
                                    , MSEC_TO_NSEC(config_httpd_req_timeout_ms())
                                    , fd_, buf_.data(), requested_size
                                    , STATE(process_body_chunk));
  }
  return yield_and_call(STATE(send_response_headers));
}

StateFlowBase::Action HttpRequestFlow::send_response_headers()
{
  if (!res_)
  {
    LOG_ERROR("[Httpd fd:%d] No response created, terminating request", fd_);
    return yield_and_call(STATE(request_complete));
  }
  size_t len = 0;
  bool keep_alive = req_.keep_alive() &&
                    req_count_ < config_httpd_max_req_per_connection();
  uint8_t * payload = res_->get_headers(&len, keep_alive);
  LOG(VERBOSE, "[Httpd fd:%d,uri:%s] Sending %d bytes", fd_
    , req_.uri().c_str(), len);
  return write_repeated(&helper_, fd_, payload, len
                      , STATE(send_response_body));
}

StateFlowBase::Action HttpRequestFlow::send_response_body()
{
  if (req_.method() == HttpMethod::HEAD)
  {
    LOG(VERBOSE, "[Httpd fd:%d,uri:%s] HEAD request, no body required", fd_
      , req_.uri().c_str());
  }
  else if (res_->get_body_length())
  {
    // check if we can send the entire response in one call or not.
    if (res_->get_body_length() > config_httpd_response_chunk_size())
    {
      LOG(VERBOSE, "[Httpd fd:%d,uri:%s] Converting to chunked response", fd_
        , req_.uri().c_str());
      response_body_offs_ = 0;
      LOG(VERBOSE, "[Httpd fd:%d,uri:%s] Sending [%d-%d/%d]", fd_
        , req_.uri().c_str(), response_body_offs_
        , config_httpd_response_chunk_size(), res_->get_body_length());
      return write_repeated(&helper_, fd_, res_->get_body()
                          , config_httpd_response_chunk_size()
                          , STATE(send_response_body_split));
    }

    LOG(VERBOSE, "[Httpd fd:%d,uri:%s] Sending %d bytes", fd_
      , req_.uri().c_str(), res_->get_body_length());
    return write_repeated(&helper_, fd_, res_->get_body()
                        , res_->get_body_length(), STATE(request_complete));
  }
  return yield_and_call(STATE(request_complete));
}

StateFlowBase::Action HttpRequestFlow::send_response_body_split()
{
  // check if there has been an error and abort if needed
  if (helper_.hasError_)
  {
    LOG(VERBOSE, "[Httpd fd:%d,uri:%s] Error reported during write", fd_
      , req_.uri().c_str());
    req_.error(true);
    return yield_and_call(STATE(request_complete));
  }
  response_body_offs_ += (config_httpd_response_chunk_size() - helper_.remaining_);
  if (response_body_offs_ >= res_->get_body_length())
  {
    // the body has been sent fully
    return yield_and_call(STATE(request_complete));
  }
  size_t remaining = res_->get_body_length() - response_body_offs_;
  if (remaining > config_httpd_response_chunk_size())
  {
    remaining = config_httpd_response_chunk_size();
  }
  LOG(VERBOSE, "[Httpd fd:%d,uri:%s] Sending [%d-%d/%d]", fd_
    , req_.uri().c_str(), response_body_offs_
    , response_body_offs_ + remaining, res_->get_body_length());
  return write_repeated(&helper_, fd_, res_->get_body() + response_body_offs_
                      , remaining, STATE(send_response_body_split));
}

StateFlowBase::Action HttpRequestFlow::request_complete()
{
  uint32_t proc_time = (esp_timer_get_time() - start_time_) / 1000ULL;
  if (!req_.uri().empty())
  {
    LOG(INFO, "[Httpd fd:%d,uri:%s] Processed in %d ms.", fd_
      , req_.uri().c_str(), proc_time);
  }
  req_count_++;
  if (!req_.keep_alive() || req_.error() ||
      req_count_ >= config_httpd_max_req_per_connection() ||
      req_.uri().empty())
  {
    req_.reset();
    return delete_this();
  }
  return yield_and_call(STATE(start_request));
}

StateFlowBase::Action HttpRequestFlow::upgrade_to_websocket()
{
  // keep the socket open since we will reuse it as the websocket
  close_ = false;
  new WebSocketFlow(server_, fd_, remote_ip_, req_.header(HttpHeader::WS_KEY)
                  , req_.header(HttpHeader::WS_VERSION)
                  , server_->ws_handler(req_.uri()));
  req_.reset();
  return delete_this();
}

} // namespace http
} // namespace esp32cs
