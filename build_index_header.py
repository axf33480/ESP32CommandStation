#######################################################################
# ESP32 COMMAND STATION
#
# COPYRIGHT (c) 2017-2019 Mike Dunston
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses
#######################################################################

Import("env")
import gzip
import os
import struct
import cStringIO

def compress(sourceFile, targetFile, prefix):
    print 'Reading %s...' % sourceFile
    gzFile = cStringIO.StringIO()
    with open(sourceFile) as f, gzip.GzipFile(mode='wb', fileobj=gzFile) as gz:
        gz.writelines(f)
    gzFile.seek(0, os.SEEK_END)
    gzLen = gzFile.tell()
    gzFile.seek(0, os.SEEK_SET)
    print 'Compressed %s file is %d bytes' % (prefix, gzLen)
    print 'Writing to %s...' % targetFile
    with open(targetFile, 'w') as f:
        f.write("#pragma once\n")
        f.write("const size_t %s_size = %s;\n" % (prefix, gzLen))
        f.write("const uint8_t %s[] PROGMEM = {\n" % prefix)
        while True:
            block = gzFile.read(16)
            if len(block) < 16:
                if len(block):
                    f.write("\t")
                    for b in block:
                        # Python 2/3 compat
                        if type(b) is str:
                            b = ord(b)
                        f.write("0x{:02X}, ".format(b))
                    f.write("\n")
                break
            f.write("\t0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X}, "
                    "0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X}, "
                    "0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X}, "
                    "0x{:02X}, 0x{:02X}, 0x{:02X}, 0x{:02X},\n"
                    .format(*struct.unpack("BBBBBBBBBBBBBBBB", block)))
        f.write("};\n")

def build_index_html_h(source, target, env):
    compress('%s/data/index.html' % env.subst('$PROJECT_DIR'),
             '%s/include/index_html.h' % env.subst('$PROJECT_DIR'),
             'indexHtmlGz')
    compress('%s/data/jquery.min.js' % env.subst('$PROJECT_DIR'),
             '%s/include/jquery_min_js.h' % env.subst('$PROJECT_DIR'),
             'jqueryJsGz')
    compress('%s/data/jquery.mobile-1.5.0-rc1.min.js' % env.subst('$PROJECT_DIR'),
             '%s/include/jquery_mobile_js.h' % env.subst('$PROJECT_DIR'),
             'jqueryMobileJsGz')
    compress('%s/data/jquery.mobile-1.5.0-rc1.min.css' % env.subst('$PROJECT_DIR'),
             '%s/include/jquery_mobile_css.h' % env.subst('$PROJECT_DIR'),
             'jqueryMobileCssGz')
    compress('%s/data/jquery.simple.websocket.min.js' % env.subst('$PROJECT_DIR'),
             '%s/include/jquery_simple_websocket.h' % env.subst('$PROJECT_DIR'),
             'jquerySimpleWebSocketGz')
    compress('%s/data/jqClock-lite.min.js' % env.subst('$PROJECT_DIR'),
             '%s/include/jq_clock.h' % env.subst('$PROJECT_DIR'),
             'jqClockGz')
    compress('%s/data/ajax-loader.gif' % env.subst('$PROJECT_DIR'),
             '%s/include/ajax_loader.h' % env.subst('$PROJECT_DIR'),
             'ajaxLoaderGz')

env.AddPreAction('$BUILD_DIR/src/Interfaces/WebServer.cpp.o', build_index_html_h)
