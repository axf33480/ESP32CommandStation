#include "ESPAsyncDNSServer.h"
#include <lwip/def.h>
#include <esp32-hal-log.h>

AsyncDNSServer::AsyncDNSServer() : _ttl(htonl(DNS_DEFAULT_TTL)), _errorReplyCode(AsyncDNSReplyCode::NonExistentDomain) {
}

bool AsyncDNSServer::start(const uint16_t port, const String &domainName,
                           const ip4_addr_t resolvedIP) {
  _port = port;
  _domainName = domainName;
  _domainName.toLowerCase();
  _domainName.replace("www.", "");
  //printf("DNS-IP: " IPSTR "\n", IP2STR(&resolvedIP));
  _resolvedIP = resolvedIP;
  if(_udp.listen(_port)) {
    _udp.onPacket(std::bind(&AsyncDNSServer::processRequest, this, std::placeholders::_1));
    return true;
  }
  return false;
}

void AsyncDNSServer::stop() {
  _udp.close();
}

void AsyncDNSServer::processRequest(AsyncUDPPacket &packet) {
  if (packet.length() >= sizeof(DNSHeader)) {
    auto buffer = packet.data();
    DNSHeader dnsHeader{};
    DNSQuestion dnsQuestion{};
    memcpy(&dnsHeader, buffer, DNS_HEADER_SIZE);

    if (ntohs(dnsHeader.QDCount) == 1 && dnsHeader.ANCount == 0 && dnsHeader.NSCount == 0 && dnsHeader.ARCount == 0) {
      // The QName has a variable length, maximum 255 bytes and is comprised of multiple labels.
      // Each label contains a byte to describe its length and the label itself. The list of 
      // labels terminates with a zero-valued byte. In "github.com", we have two labels "github" & "com"
      // Iterate through the labels and copy them as they come into a single buffer (for simplicity's sake)
      while (buffer[DNS_HEADER_SIZE + dnsQuestion.QNameLength] != 0) {
        memcpy((void*)&dnsQuestion.QName[dnsQuestion.QNameLength],
               (void*)&buffer[DNS_HEADER_SIZE + dnsQuestion.QNameLength],
               buffer[DNS_HEADER_SIZE + dnsQuestion.QNameLength] + 1);
        dnsQuestion.QNameLength += buffer[DNS_HEADER_SIZE + dnsQuestion.QNameLength] + 1;
      }
      dnsQuestion.QName[dnsQuestion.QNameLength] = 0;
      dnsQuestion.QNameLength++;

      // Copy the QType and QClass 
      memcpy(&dnsQuestion.QType, (void*)&buffer[DNS_HEADER_SIZE + dnsQuestion.QNameLength], sizeof(dnsQuestion.QType));
      memcpy(&dnsQuestion.QClass, (void*)&buffer[DNS_HEADER_SIZE + dnsQuestion.QNameLength + sizeof(dnsQuestion.QType)], sizeof(dnsQuestion.QClass));
    }

    String domainNameWithoutWwwPrefix = getDomainNameWithoutWwwPrefix(buffer);

    if (dnsHeader.QR == DNS_QR_QUERY && dnsHeader.OPCode == DNS_OPCODE_QUERY &&
        (_domainName == "*" || domainNameWithoutWwwPrefix == _domainName)
       ) {
      AsyncUDPMessage msg(packet.length() + 12 + sizeof(_resolvedIP));
      DNSHeader responseHeader = {0};
      memcpy(&responseHeader, &dnsHeader, sizeof(DNSHeader));
      // Change the type of message to a response and set the number of answers equal to 
      // the number of questions in the header
      responseHeader.QR      = DNS_QR_RESPONSE;
      responseHeader.ANCount = responseHeader.QDCount;
      // Write the dns header
      msg.write((uint8_t *)&responseHeader, DNS_HEADER_SIZE);
      // Write the question
      msg.write((uint8_t *)&dnsQuestion.QName, dnsQuestion.QNameLength);
      msg.write((uint8_t *)&dnsQuestion.QType, sizeof(uint16_t));
      msg.write((uint8_t *)&dnsQuestion.QClass, sizeof(uint16_t));
      // Write the answer 
      // Use DNS name compression : instead of repeating the name in this RNAME occurence,
      // set the two MSB of the byte corresponding normally to the length to 1. The following
      // 14 bits must be used to specify the offset of the domain name in the message 
      // (<255 here so the first byte has the 6 LSB at 0) 
      msg.write(0xC0);
      msg.write((uint8_t)DNS_OFFSET_DOMAIN_NAME);
      // DNS type A : host address, DNS class IN for INternet, returning an IPv4 address 
      uint16_t answerType = htons(DNS_TYPE_A), answerClass = htons(DNS_CLASS_IN), answerIPv4 = htons(DNS_RDLENGTH_IPV4);
      msg.write((uint8_t *)&answerType, 2);
      msg.write((uint8_t *)&answerClass, 2);
      msg.write((uint8_t *)&_ttl, 4);
      msg.write((uint8_t *)&answerIPv4, 2);
      msg.write((uint8_t *)&_resolvedIP.addr, sizeof(_resolvedIP.addr));
      packet.send(msg);
      log_d("[DNS] %s -> IP: " IPSTR "\n", domainNameWithoutWwwPrefix.c_str(), IP2STR(&_resolvedIP));
    } else if (dnsHeader.QR == DNS_QR_QUERY) {
      log_d("[DNS] Reply with Code %d\n", _errorReplyCode);
      AsyncUDPMessage msg(packet.length());
      msg.write(packet.data(), packet.length());
      DNSHeader * responseHeader = (DNSHeader *)msg.data();
      responseHeader->QR = DNS_QR_RESPONSE;
      responseHeader->RCode = (uint8_t)_errorReplyCode;
      responseHeader->QDCount = 0;
      packet.send(msg);
    }
  }
}

String AsyncDNSServer::getDomainNameWithoutWwwPrefix(unsigned char *buffer)
{
  // Error checking : if the buffer containing the DNS request is a null pointer, return an empty domain
  String parsedDomainName = "";
  if (buffer == NULL) {
    return parsedDomainName;
  }

  // Set the start of the domain just after the header (12 bytes). If equal to null character, return an empty domain
  unsigned char *start = buffer + DNS_OFFSET_DOMAIN_NAME;
  if (*start == 0) {
    return parsedDomainName;
  }

  int pos = 0;
  while(true) {
    unsigned char labelLength = *(start + pos);
    for(int i = 0; i < labelLength; i++) {
      pos++;
      parsedDomainName += (char)*(start + pos);
    }
    pos++;
    if (*(start + pos) == 0) {
      parsedDomainName.toLowerCase();
      parsedDomainName.replace("www.", "");
      return parsedDomainName;
    } else {
      parsedDomainName += ".";
    }
  }
}