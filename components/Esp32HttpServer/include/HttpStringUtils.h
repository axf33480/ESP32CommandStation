/**********************************************************************
ESP32 HTTP Server

COPYRIGHT (c) 2019-2020 Mike Dunston

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

/// @file std::string utility methods.

#ifndef STRINGUTILS_H_
#define STRINGUTILS_H_

#include <string>
#include <vector>

namespace http
{

using std::string;
using std::vector;
using std::pair;
using std::make_pair;

/// Helper method to break a string into a pair<string, string> based on a delimeter.
///
/// @param str is the string to break.
/// @param delim is the delimeter to break the string on.
///
/// @return the pair of string objects.
static inline pair<string, string> break_string(string &str
                                              , const string& delim)
{
  size_t pos = str.find_first_of(delim);
  if (pos == string::npos)
  {
    return make_pair(str, "");
  }
  size_t end_pos = pos + delim.length();
  if (end_pos > str.length())
  {
    end_pos = pos;
  }
  return make_pair(str.substr(0, pos), str.substr(end_pos));
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
string::size_type tokenize(const string& str, ContainerT& tokens
                         , const string& delimeter = " "
                         , bool keep_incomplete = true
                         , bool discard_empty = false)
{
  string::size_type pos, lastPos = 0;

  using value_type = typename ContainerT::value_type;
  using size_type  = typename ContainerT::size_type;

  while(lastPos < str.length())
  {
    pos = str.find_first_of(delimeter, lastPos);
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
static inline string string_join(const vector<string>& strings
                               , const string& delimeter = "")
{
  string result;
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

/// Helper which joins a vector<string> using a first and last iterator.
///
/// @param first is the starting iterator position.
/// @param last is the starting iterator position.
/// @param delimeter is the string to join the segments with.
/// @return the joined string.
static inline string string_join(const vector<string>::iterator first
                               , const vector<string>::iterator last
                               , const string& delimeter = "")
{
  vector<string> vec(first, last);
  return string_join(vec, delimeter);
}

/// Helper which decodes urlencoded strings as described in RFC-1738 sec. 2.2.
///
/// @param source is the string to be decoded.
/// @return the decoded string.
///
/// NOTE: only two portions of RFC-1738 section 2.2 are currently implemented:
/// 1. '+' replaced by ' ' (space).
/// 2. '%HH' replaced by the hex decoded character HH.
static inline string url_decode(const string source)
{
  string decoded = source;

  // replace + with space
  std::replace(decoded.begin(), decoded.end(), '+', ' ');

  // search and replace %{hex}{hex} with hex decoded character
  while (decoded.find("%") != string::npos)
  {
    // find the % character
    auto pos = decoded.find("%");
    if (pos + 2 < decoded.size())
    {
      // decode the character
      auto sub = decoded.substr(pos + 1, 2);
      char ch = std::stoi(sub, nullptr, 16);
      // insert the replacement
      decoded.insert(pos, 1, ch);
      // remove the decoded piece
      decoded.erase(pos + 1, 3);
    }
    else
    {
      // the % is not followed by at least two characters.
      break;
    }
  }
  return decoded;
}

} // namespace http

#endif // STRINGUTILS_H_