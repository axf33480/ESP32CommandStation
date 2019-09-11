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

/// @file Definitions of the esp32cs::http namespace members.

#ifndef HTTPD_H_
#define HTTPD_H_

#include <map>
#include <stdint.h>

#include <mbedtls/sha1.h>

#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>
#include <utils/Base64.hxx>
#include <utils/constants.hxx>
#include <utils/Singleton.hxx>
#include <utils/socket_listener.hxx>
#include <utils/Uninitialized.hxx>

/// Namespace for all ESP32 Command Station functionality.
namespace esp32cs
{
/// Namespace for HTTP related functionality.
namespace http
{

/// FreeRTOS task stack size for the httpd Executor.
DECLARE_CONST(httpd_server_stack_size);

/// FreeRTOS task priority for the httpd Executor.
DECLARE_CONST(httpd_server_priority);

/// Max number of bytes to read in a single chunk when reading the HTTP request
/// headers.
DECLARE_CONST(httpd_header_chunk_size);

/// Max number of bytes to read in a single chunk when reading the HTTP request
/// body payload.
DECLARE_CONST(httpd_body_chunk_size);

/// Max number of bytes to write in a single chunk when sending the HTTP
/// response to the client.
DECLARE_CONST(httpd_response_chunk_size);

/// Maximum size of the HTTP request body payload. Any request which exceeds
/// this limit will be forcibly aborted.
DECLARE_CONST(httpd_max_req_size);

/// This is the maximum number of HTTP requests which should be processed for a
/// single connection with keep-alive active before the connection will be
/// closed with the "Connection: close" header.
DECLARE_CONST(httpd_max_req_per_connection);

/// This is the maximum wait time for receiving a single chunk of data from the
/// HTTP request.
DECLARE_CONST(httpd_req_timeout_ms);

/// This is the number of milliseconds to use for the socket send and receive
/// timeouts.
DECLARE_CONST(httpd_socket_timeout_ms);

/// This is the number of milliseconds to use as the read timeout for all
/// websocket connections. When this limit and the max_XX_attempts limit have
/// been exceeded the send/receive attempt is aborted and the inverse operation
/// will be attempted.
DECLARE_CONST(httpd_websocket_timeout_ms);

/// This is the maximum data size to send/receive in a single operation. If a
/// websocket frame is received exceeding this limit it will be processed in
/// chunks of this size.
DECLARE_CONST(httpd_websocket_max_frame_size);

/// This controls how many attempts will be allowed to receive a websocket
/// frame before attempting to send out a websocket frame.
DECLARE_CONST(httpd_websocket_max_read_attempts);

/// Commonly used HTTP status codes.
/// @enum HttpStatusCode
enum HttpStatusCode
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

/// Default HTTP listener port
static constexpr uint16_t DEFAULT_HTTP_PORT = 80;

/// Commonly used and well-known HTTP methods.
enum HttpMethod
{
  /// Request is for deleting a resource.
  DELETE,
  /// Request is for retrieving a resource.
  GET,
  /// Request is for retrieving the headers for a resource. 
  HEAD,
  /// Request is for creating a resource.
  POST,
  /// Request is for patching an existing resource.
  PATCH,
  /// Request is for applying an update to an existing resource.
  PUT,
  /// Request type was not understood by the server.
  UNKNOWN,
};

/// Commonly used and well-known HTTP headers
enum HttpHeader
{
  ACCEPT,
  CACHE_CONTROL,
  CONNECTION,
  CONTENT_ENCODING,
  CONTENT_TYPE,
  CONTENT_LENGTH,
  HOST,
  LOCATION,
  ORIGIN,
  UPGRADE,
  WS_VERSION,
  WS_KEY,
  WS_ACCEPT,
};

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
static constexpr const char * HTTP_CONNECTION_UPGRADE = "Upgrade";

static constexpr const char * HTTP_UPGRADE_HEADER_WEBSOCKET = "websocket";

// HTTP end of line characters
static constexpr const char * HTML_EOL = "\r\n";

// Common mime-types
static constexpr const char * MIME_TYPE_NONE = "";
static constexpr const char * MIME_TYPE_TEXT_CSS = "text/css";
static constexpr const char * MIME_TYPE_TEXT_HTML = "text/html";
static constexpr const char * MIME_TYPE_TEXT_PLAIN = "text/plain";
static constexpr const char * MIME_TYPE_TEXT_JAVASCRIPT = "text/javascript";
static constexpr const char * MIME_TYPE_IMAGE_GIF = "image/gif";
static constexpr const char * MIME_TYPE_IMAGE_PNG = "image/png";
static constexpr const char * MIME_TYPE_APPLICATION_JSON = "application/json";

static constexpr const char * HTTP_ENCODING_NONE = "";
static constexpr const char * HTTP_ENCODING_GZIP = "gzip";

/// Forward declaration of the WebSocketFlow so it can access internal methods
/// of various classes.
class WebSocketFlow;

/// Forward declaration of the HttpdRequestFlow so it can access internal
/// methods of various classes.
class HttpRequestFlow;

/// Forward declaration of the Httpd so it can access internal methods of
/// various classes.
class Httpd;

/// This is the base class for an HTTP response.
class AbstractHttpResponse
{
public:
  /// Constructor.
  ///
  /// @param code is the @ref HttpStatusCode to use for the
  /// response.
  /// @param mime_type is the mime type to send as part of the Content-Type
  /// header.
  AbstractHttpResponse(HttpStatusCode code=STATUS_NOT_FOUND
                     , const std::string &mime_type=MIME_TYPE_NONE);

  /// Destructor.
  virtual ~AbstractHttpResponse();

  /// Encodes the HTTP response headers for transmission to the client.
  ///
  /// @param len is the length of the headers after encoding. This is an
  /// OUTPUT parameter.
  /// @param keep_alive is used to control the "Connection: [keep-alive|close]"
  /// header.
  /// @param add_keep_alive is used to control if the Connection header will be
  /// included in the response.
  ///
  /// @return a buffer containing the HTTP response. This buffer is owned by
  /// the @ref AbstractHttpResponse and does not need to be
  /// freed by the caller.
  uint8_t *get_headers(size_t *len, bool keep_alive=false
                     , bool add_keep_alive=true);

  /// Encodes the HTTP response into a string which can be printed. This is
  /// used by @ref get_headers internally with include_body set to false.
  ///
  /// @param include_body will include the body of the response as well as any
  /// headers.
  /// @param keep_alive is used to control the "Connection: [keep-alive|close]"
  /// header.
  /// @param add_keep_alive is used to control if the Connection header will be
  /// included in the response.
  ///
  /// @return the request as a printable string.
  std::string to_string(bool include_body=false, bool keep_alive=false
                      , bool add_keep_alive=true);

  /// @return the response body as a buffer which can be streamed. This buffer
  /// is owned by the @ref AbstractHttpResponse and does not
  /// need to be freed by the caller.
  ///
  /// Note: this method should be overriden by sub-classes to supply the
  /// response body.
  virtual const uint8_t *get_body()
  {
    return nullptr;
  }

  /// @return the size of the body payload.
  ///
  /// Note: this method should be overriden by sub-classes to supply the
  /// response body.
  virtual size_t get_body_length()
  {
    return 0;
  }

  /// @return the mime type to include in the HTTP response header.
  ///
  /// Note: this method should be overriden by sub-classes to supply the
  /// response body.
  std::string get_body_mime_type()
  {
    return mime_type_;
  }

protected:
  /// Adds an arbitrary HTTP header to the response object.
  ///
  /// @param header is the HTTP header name to add.
  /// @param value is the HTTP header value to add.
  void header(const std::string &header, const std::string &value);

  /// Adds a well-known HTTP header to the response object.
  ///
  /// @param header is the HTTP header name to add.
  /// @param value is the HTTP header value to add.
  void header(const HttpHeader header, const std::string &value);

private:
  /// Collection of headers and values for this HTTP response.
  std::map<std::string, std::string> headers_;

  /// @ref HttpStatusCode for this HTTP response.
  HttpStatusCode code_;

  /// Content-Type header value.
  std::string mime_type_;

  /// Temporary storage for the HTTP response headers in an encoded format
  /// that is ready for transmission to the client.
  std::string encoded_headers_;

  /// Gives @ref WebSocketFlow access to protected/private
  /// members.
  friend class WebSocketFlow;

  /// Gives @ref HttpRequestFlow access to protected/private
  /// members.
  friend class HttpRequestFlow;
};

/// HTTP Response object for URI not found.
class UriNotFoundResponse : public AbstractHttpResponse
{
public:
  /// Constructor.
  ///
  /// @param uri is the URI that was requested that is not known by the server.
  UriNotFoundResponse(const std::string &uri)
    : body_(StringPrintf("URI %s was not found.", uri.c_str()))
  {
  }

  /// @return the pre-formatted body of this response.
  virtual const uint8_t *get_body() override final
  {
    return (const uint8_t *)(body_.c_str());
  }

  /// @return the size of the pre-formatted body of this response.
  virtual size_t get_body_length() override final
  {
    return body_.length();
  }

private:

  /// Temporary storage for the response body.
  const std::string body_;
};

/// HTTP Response object used to redirect the client to a different location
/// via the HTTP Header: "Location: uri".
class RedirectResponse : public AbstractHttpResponse
{
public:
  /// Constructor.
  ///
  /// @param target_url is the target which the client should be redirected to.
  RedirectResponse(const std::string &target_url);
};

/// HTTP Response object which is used to return a static payload to a client
/// when the URI is accessed.
///
/// Note: The payload must remain alive as long as the URI is referenced by the
/// @ref Httpd server.
class StaticResponse : public AbstractHttpResponse
{
public:
  /// Constructor.
  ///
  /// @param payload is the body of the response to send.
  /// @param length is the length of the body payload.
  /// @param mime_type is the value to send in the Content-Type HTTP header.
  /// @param encoding is the optional encoding to send in the Content-Encoding
  /// HTTP Header.
  StaticResponse(const uint8_t *payload, const size_t length
               , const std::string mime_type
               , const std::string encoding = HTTP_ENCODING_NONE)
    : AbstractHttpResponse(STATUS_OK, mime_type), payload_(payload)
    , length_(length)
  {
    if (!encoding.empty())
    {
      header(HttpHeader::CONTENT_ENCODING, encoding);
    }
  }

  /// @return the pre-formatted body of this response.
  virtual const uint8_t *get_body() override final
  {
    return payload_;
  }

  /// @return the size of the pre-formatted body of this response.
  virtual size_t get_body_length() override final
  {
    return length_;
  }

private:
  /// Pointer to the payload to return for this URI.
  const uint8_t *payload_;

  /// Length of the payload to return for this URI.
  const size_t length_;
};

/// HTTP Response object which can be used to return a string based response to
/// a given URI.
class StringResponse : public AbstractHttpResponse
{
public:
  /// Constructor.
  ///
  /// @param response is the response string to send as the HTTP response body.
  /// @param mime_type is the value to use for the Content-Type HTTP header.
  ///
  /// Note: The ownership of the response object passed into this method will
  /// be transfered to this class instance and will be cleaned up after it has
  /// been sent to the client.
  StringResponse(const std::string &response, const std::string &mime_type);

  /// @return the pre-formatted body of this response.
  virtual const uint8_t *get_body() override final
  {
    return (uint8_t *)response_.c_str();
  }

  /// @return the size of the pre-formatted body of this response.
  virtual size_t get_body_length() override final
  {
    return response_.length();
  }

private:
  /// Temporary storage of the response payload.
  std::string response_;
};

/// Runtime state of an HTTP Request.
class HttpRequest
{
public:
  /// @return the parsed well-known @ref HttpMethod.
  HttpMethod method();

  /// @return the unparsed HTTP method.
  const std::string &raw_method();

  /// @return the URI that this request is for.
  const std::string &uri();

  /// @return true if the named HTTP Header exists.
  /// @param name of the parameter to check for.
  bool has_header(const std::string &name);

  /// @return true if the well-known @ref HttpHeader exists.
  bool has_header(const HttpHeader name);

  /// @return the value of the named HTTP Header or a blank string if it does
  /// not exist.
  const std::string &header(const std::string name);

  /// @return the value of the well-known @ref HttpHeader or a blank string if
  /// it does not exist.
  const std::string &header(const HttpHeader name);

  /// @return true if the well-known @ref HttpHeader::CONNECTION header exists
  /// with a value of "keep-alive".
  bool keep_alive();

  /// @return true if the request could not be parsed successfully.
  bool error();

  /// @return string form of the request, this is headers only.
  std::string to_string();

private:
  /// Gives @ref HttpRequestFlow access to protected/private members.
  friend class HttpRequestFlow;

  /// Sets the @ref HttpMethod if it is well-known, otherwise only the unparsed
  /// value will be available.
  ///
  /// @param value is the raw value parsed from the first line of the HTTP
  /// request stream.
  void method(const std::string &value);

  /// Sets the URI of the HttpRequest.
  ///
  /// @param value is the value of the URI.
  void uri(const std::string &value);

  /// Adds a URI parameter to the request.
  ///
  /// @param value is a pair<string, string> of the key:value pair.
  void param(const std::pair<std::string, std::string> &value);

  /// Adds an HTTP Header to the request.
  ///
  /// @param value is a pair<string, string> of the key:value pair.
  void header(const std::pair<std::string, std::string> &value);

  /// Resets the internal state of the @ref HttpRequest to defaults so it can
  /// be reused for subsequent requests.
  void reset();

  /// Sets/Resets the parse error flag.
  ///
  /// @param value should be true if there is a parse failure, false otherwise.
  void error(bool value);

  /// default return value when a requested header or parameter is not known.
  const std::string no_value_{""};

  /// Collection of HTTP Headers that have been parsed from the HTTP request
  /// stream.
  std::map<std::string, std::string> headers_;

  /// Collection of parameters supplied with the HTTP Request after the URI.
  std::map<std::string, std::string> params_;

  /// Parsed @ref HttpMethod for this @ref HttpRequest.
  HttpMethod method_;
  
  /// Raw unparsed HTTP method for this @ref HttpRequest.
  std::string raw_method_;
  
  /// Parsed URI of this @ref HttpRequest.
  std::string uri_;

  /// Parse error flag.
  bool error_;
};

/// WebSocket events for the @ref WebSocketHandler callback.
typedef enum
{
  /// A new Websocket connection has been established.
  WS_EVENT_CONNECT,
  
  /// A WebSocket connection has been closed.
  WS_EVENT_DISCONNECT,

  /// A TEXT message has been received from a WebSocket. Note that it may be
  /// sent to the handler in pieces.
  WS_EVENT_TEXT,
  
  /// A BINARY message has been received from a WebSocket. Note that it may be
  /// sent to the handler in pieces.
  WS_EVENT_BINARY
} WebSocketEvent;

/// URI processing handler.
///
/// The method that implements this interface will be called for any access to
/// the assigned URI. When the request has a body payload it will be streamed
/// to this handler in chunks based on the constant httpd_body_chunk_size.
///
/// The return value from this function call will be sent as the response to
/// the request. A value of nullptr will result in no response being sent.
typedef std::function<AbstractHttpResponse *(const HttpRequest *  /* request */
                                           , const uint8_t *      /* data */
                                           , size_t               /* length */
                                           )> RequestProcessor;

/// WebSocket processing Handler.
///
/// This method will be invoked when there is an event to be processed.
///
/// When @ref WebSocketEvent is @ref WebSocketEvent::WS_EVENT_CONNECT or
/// @ref WebSocketEvent::WS_EVENT_DISCONNECT data will be
/// nullptr and data length will be zero.
/// 
/// When @ref WebSocketEvent is @ref WebSocketEvent::WS_EVENT_TEXT or
/// @ref WebSocketEvent::WS_EVENT_BINARY the data parameter
/// will be a buffer of data length bytes of text or binary data to be
/// processed by the handler.
///
/// The handler can invoke the @ref WebSocketFlow parameter to retrieve
/// additional details about the WebSocket client, queue response text or
/// binary data for delivery at next available opportunity, or request the
/// WebSocket connection to be closed.
typedef std::function<void(WebSocketFlow *  /* websocket */
                         , WebSocketEvent   /* event */
                         , uint8_t *        /* data */
                         , size_t           /* data length */
                         )> WebSocketHandler;

/// HTTP Server implementation
class Httpd : public Service, public Singleton<Httpd>
{
public:
  /// Constructor.
  ///
  /// @param name is the name to use for the executor.
  /// @param port is the port to listen for HTTP requests on.
  Httpd(const std::string &name = "httpd", uint16_t port = DEFAULT_HTTP_PORT);
  
  /// Destructor.
  virtual ~Httpd();
  
  /// Registers a URI with the provided handler.
  ///
  /// @param uri is the URI to call the provided handler for.
  /// @param handler is the @ref RequestProcessor to invoke when this URI is
  /// requested.
  void uri(const std::string &uri, RequestProcessor handler);

  /// Registers a URI to redirect to another location.
  ///
  /// @param source is the URI which should trigger the redirect.
  /// @param target is where the request should be routed instead.
  ///
  /// Note: This will result in the client receiving an HTTP 302 response
  /// with an updated Location value provided.
  void redirect_uri(const std::string &source, const std::string &target);

  /// Registers a static response URI.
  ///
  /// @param uri is the URI to serve the static content for.
  /// @param content is the content to send back to the client when this uri is
  /// requested. Note that large content blocks will be broken into smaller
  /// pieces for transmission to the client and thus must remain in memory.
  /// @param length is the length of the content to send to the client.
  /// @param mime_type is the Content-Type parameter to return to the client.
  /// @param encoding is the encoding for the content, if not specified the
  /// Content-Encoding header will not be transmitted.
  void static_uri(const std::string &uri, const uint8_t *content
                , const size_t length, const std::string &mime_type
                , const std::string &encoding = HTTP_ENCODING_NONE);

  /// Registers a WebSocket handler for a given URI.  ///
  /// @param uri is the URI to process as a WebSocket endpoint.
  /// @param handler is the @ref WebSocketHandler to invoke when this URI is
  /// requested.
  void websocket_uri(const std::string &uri, WebSocketHandler handler);

  /// Sends a binary message to a single WebSocket.
  ///
  /// @param id is the ID of the WebSocket to send the data to.
  /// @param data is the binary data to send to the websocket client.
  /// @param length is the length of the binary data to send to the websocket
  /// client.
  ///
  /// Note: this is currently unimplemented.
  void send_websocket_binary(int id, uint8_t *data, size_t length);

  /// Sends a text message to a single WebSocket.
  /// 
  /// @param id is the ID of the WebSocket to send the text to.
  /// @param text is the text to send to the WebSocket client.
  void send_websocket_text(int id, std::string &text);

private:
  /// Gives @ref WebSocketFlow access to protected/private members.
  friend class WebSocketFlow;

  /// Gives @ref HttpRequestFlow access to protected/private members.
  friend class HttpRequestFlow;

  /// Callback for a newly accepted socket connection.
  ///
  /// @param fd is the socket handle.
  void on_new_connection(int fd);

  /// Registers a new @ref WebSocketFlow with the server to allow sending
  /// text or binary messages based on the WebSocket ID.
  ///
  /// @param id is the ID of the WebSocket client.
  /// @param ws is the @ref WebSocketFlow managing the WebSocket client.
  void add_websocket(int id, WebSocketFlow *ws);

  /// Removes a previously registered WebSocket client.
  ///
  /// @param id of the WebSocket client to remove.
  void remove_websocket(int id);

  /// @return the @ref RequestProcessor for a URI.
  /// @param uri is the URI to retrieve the @ref RequestProcessor for.
  RequestProcessor handler(const std::string &uri);

  /// @return the @ref WebSocketHandler for the provided URI.
  /// @param uri is the URI to retrieve the @ref WebSocketHandler for.
  WebSocketHandler ws_handler(const std::string &uri);

  /// @return true if there is a @ref AbstractHttpResponse for the URI.
  /// @param uri is the URI to check.
  bool have_known_response(const std::string &uri);

  /// @return the @ref AbstractHttpResponse for a given @param uri if available.
  std::shared_ptr<AbstractHttpResponse> response(const std::string &uri);

  /// @return true if the @param request is too large to be processed. Size
  /// is configured via httpd_max_req_size.
  bool is_request_too_large(HttpRequest *request);

  /// @return true if the @param request can be serviced by this @ref Httpd.
  bool is_servicable_uri(HttpRequest *request);

  /// @ref Executor that manages all @ref StateFlow for the @ref Httpd server.
  Executor<1> executor_;

  /// TCP/IP port to listen for HTTP requests on.
  uint16_t port_;

  /// @ref SocketListener that will accept() the socket connections and call
  /// the @ref Httpd when a new client is available.
  uninitialized<SocketListener> listener_;

  /// Internal state flag for the listener_ being active.
  bool active_{false};

  /// Internal map of all registered @ref RequestProcessor handlers.
  std::map<std::string, RequestProcessor> handlers_;

  /// Internal map of all registered static URIs.
  std::map<std::string, std::shared_ptr<AbstractHttpResponse>> static_uris_;

  /// Internal map of all redirected URIs.
  std::map<std::string, std::shared_ptr<AbstractHttpResponse>> redirect_uris_;

  /// Internal map of all registered @ref WebSocketHandler URIs.
  std::map<std::string, WebSocketHandler> websocket_uris_;

  /// Internal map of active @ref WebSocketFlow instances.
  std::map<int, WebSocketFlow *> websockets_;

  DISALLOW_COPY_AND_ASSIGN(Httpd);
};

/// HTTP Request parser that implements the @ref StateFlowBase interface.
class HttpRequestFlow : private StateFlowBase
{
public:
  /// Constructor.
  ///
  /// @param server is the @ref Httpd server owning this request.
  /// @param fd is the socket handler.
  /// @param remote_ip is the remote IP address of the client.
  HttpRequestFlow(Httpd *server, int fd, uint32_t remote_ip);

  /// Destructor.
  virtual ~HttpRequestFlow();
private:
  /// @ref StateFlowTimedSelectHelper which assists in reading/writing of the
  /// request data stream.
  StateFlowTimedSelectHelper helper_{this};

  /// @ref Httpd instance that owns this request.
  Httpd *server_;

  /// Underlying socket handler for this request.
  int fd_;

  /// Remote client IP (if known).
  uint32_t remote_ip_;

  /// @ref HttpRequest data holder.
  HttpRequest req_;

  /// Flag to indicate that the underlying socket handler should be closed when
  /// this @ref HttpRequestFlow is deleted. In the case of a WebSocket the
  /// socket needs to be preserved.
  bool close_{true};

  /// Temporary buffer used for reading the HTTP request.
  std::vector<uint8_t> buf_;

  /// Index into @ref buf_ for partial reads.
  size_t body_offs_;
  
  /// Total size of the request body.
  size_t body_len_;

  /// Temporary accumulator for the HTTP header data as it is being parsed.
  std::string raw_header_;

  /// @ref AbstractHttpResponse that represents the response to this request.
  std::shared_ptr<AbstractHttpResponse> res_;

  /// Index into the response body payload.
  size_t response_body_offs_{0};

  /// Request start time.
  uint64_t start_time_;

  /// Current request number for this client connection.
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

/// WebSocket processor implementing the @ref StateFlowBase interface.
class WebSocketFlow : private StateFlowBase
{
public:
  /// Constructor.
  ///
  /// @param server is the @ref Httpd server owning this WebSocket.
  /// @param fd is the socket handle.
  /// @param remote_ip is the remote IP address (if known).
  /// @param ws_key is the "Sec-WebSocket-Key" HTTP Header from the initial
  /// request.
  /// @param ws_version is the "Sec-WebSocket-Version" HTTP header from the
  /// initial request.
  /// @param handler is the @ref WebSocketHandler that will process the events
  /// as they are raised.
  WebSocketFlow(Httpd *server, int fd, uint32_t remote_ip
              , const std::string &ws_key, const std::string &ws_version
              , WebSocketHandler handler);

  /// Destructor.
  ~WebSocketFlow();

  /// Sends text to this WebSocket at the next possible interval.
  ///
  /// @param text is the text to send.
  void send_text(std::string &text);

  /// @return the ID of the WebSocket.
  int get_id();

  /// @return the IP address of the remote side of the WebSocket.
  ///
  /// Note: This may return zero in which case the remote IP address is not
  /// known.
  uint32_t get_remote_ip();

  /// This will trigger an orderly shutdown of the WebSocket at the next
  /// opportunity. This will trigger the @ref WebSocketHandler with the
  /// @ref WebSocketEvent set to @ref WebSocketEvent::WS_EVENT_DISCONNECT.
  void request_close();

private:
  /// @ref StateFlowTimedSelectHelper which assists in reading/writing of the
  /// request data stream.
  StateFlowTimedSelectHelper helper_{this};

  /// @ref Httpd instance that owns this request.
  Httpd *server_;

  /// Underlying socket handler for this request.
  int fd_;
  
  /// Remote client IP (if known).
  uint32_t remote_ip_;

  /// WebSocket read/write timeout for a data frame.
  const uint64_t timeout_;

  /// Maximum size to read/write of a frame in one call.
  const uint64_t max_frame_size_;

  /// Temporary buffer used for reading/writing WebSocket frame data.
  uint8_t *data_;

  /// Size of the used data in the temporary buffer.
  size_t data_size_;
  
  /// Temporary holder for the WebSocket handshake response.
  std::string handshake_;

  /// @ref WebSocketHandler to be invoked when a @ref WebSocketEvent needs to be
  /// processed.
  WebSocketHandler handler_;

  /// internal frame header data
  uint16_t header_;

  /// Parsed op code from the header data.
  uint8_t opcode_;

  /// When true the frame data is XOR masked with a 32bit XOR mask.
  bool masked_;

  /// internal flag indicating the frame length type.
  uint8_t frameLenType_;

  /// Temporary holder for 16bit frame length data.
  uint16_t frameLength16_;

  /// Length of the WebSocket Frame data.
  uint64_t frameLength_;

  /// 32bit XOR mask to apply to the data when @ref masked_ is true.
  uint32_t maskingKey_;

  /// Lock for the @ref textToSend_ buffer.
  OSMutex textLock_;

  /// Buffer of raw text message(s) to send to the client. Multiple messages
  /// can be sent as one frame if they are sent to this client rapidly.
  std::string textToSend_;

  /// When set to true the @ref WebSocketFlow will attempt to shutdown the
  /// WebSocket connection at it's next opportunity.
  bool close_requested_{false};

  // helper for fully reading a data block with a timeout so we can close the
  // socket if there are too many timeout errors
  uint8_t *buf_{nullptr};
  size_t buf_size_;
  size_t buf_offs_;
  size_t buf_remain_;
  uint8_t buf_attempts_;
  Callback buf_next_;
  Callback buf_next_timeout_;
  Action read_fully_with_timeout(void *buf, size_t size, size_t attempts
                               , Callback success, Callback timeout);
  STATE_FLOW_STATE(data_received);

  /// Internal state flow for reading a websocket frame of data and sending
  /// back a response, possibly broken into chunks.
  STATE_FLOW_STATE(send_handshake);
  STATE_FLOW_STATE(handshake_sent);
  STATE_FLOW_STATE(read_frame_header);
  STATE_FLOW_STATE(frame_header_received);
  STATE_FLOW_STATE(frame_data_len_received);
  STATE_FLOW_STATE(start_recv_frame_data);
  STATE_FLOW_STATE(recv_frame_data);
  STATE_FLOW_STATE(shutdown_connection);
  STATE_FLOW_STATE(send_frame_header);
  STATE_FLOW_STATE(frame_sent);
};

/// Helper method to break a string into a pair<string, string> based on a delimeter.
///
/// @param str is the string to break.
/// @param delim is the delimeter to break the string on.
///
/// @return the pair of string objects.
static inline std::pair<std::string, std::string> break_string(std::string &str
                                                             , const std::string& delim)
{
  size_t pos = str.find(delim);
  if (pos == std::string::npos)
  {
    return std::make_pair(str, "");
  }
  return std::make_pair(str.substr(0, pos), str.substr(pos + delim.length()));
}

/// Helper which will break a string into multiple pieces based on a provided
/// delimeter.
///
/// @param str is the string to tokenize.
/// @param tokens is the container which will receive the tokenized strings.
/// @param delimeter to tokenize the string on.
/// @param keep_incomplete will take the last token of the input string and
/// insert it to the container as the last element when this is true. When
/// this is false the last token will not be inserted to the container.
/// @param discard_empty will discard empty tokens when set to true.
template <class ContainerT>
std::string::size_type tokenize(const std::string& str, ContainerT& tokens
                              , const std::string& delimeter = " "
                              , bool keep_incomplete = true
                              , bool discard_empty = false)
{
  std::string::size_type pos, lastPos = 0;

  using value_type = typename ContainerT::value_type;
  using size_type  = typename ContainerT::size_type;

  while(lastPos < str.length())
  {
    pos = str.find(delimeter, lastPos);
    if (pos == std::string::npos)
    {
      if (!keep_incomplete)
      {
        return lastPos;
      }
      pos = str.length();
    }

    if (pos != lastPos || !discard_empty)
    {
      tokens.emplace_back(value_type(str.data() + lastPos
                                  , (size_type)pos - lastPos));
    }
    lastPos = pos + delimeter.length();
  }
  return lastPos;
}

/// Helper which joins a vector<string> with a delimeter.
///
/// @param strings is the vector<string> to join
/// @param delimeter is the string to join the segments with.
/// @return the joined string.
static inline std::string string_join(const std::vector<std::string>& strings
                                    , const std::string& delimeter = "")
{
  std::string result;

  for (auto piece : strings)
  {
    if (!result.empty())
    {
      result += delimeter;
    }
    result += piece;
  }
  return result;
}

} // namespace http
} // namespace esp32cs

#endif // HTTPD_H_
