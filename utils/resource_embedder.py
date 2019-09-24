import gzip
import os
import struct
import cStringIO

def serialize(sourceFile, targetFile, inputStream, length, prefix):
    print 'Converting %s to %s...' % (sourceFile, targetFile)
    with open(targetFile, 'w') as f:
        f.write("#pragma once\n")
        f.write("const size_t %s_size = %s;\n" % (prefix, length))
        f.write("const uint8_t %s[] PROGMEM = {\n" % prefix)
        while True:
            block = inputStream.read(16)
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

def compress(sourceFile, outputFileName, prefix):
  gzFile = cStringIO.StringIO()
  with open(sourceFile) as f, gzip.GzipFile(mode='wb', fileobj=gzFile) as gz:
    gz.writelines(f)
  gzFile.seek(0, os.SEEK_END)
  length = gzFile.tell()
  gzFile.seek(0, os.SEEK_SET)
  serialize(sourceFile, outputFileName, gzFile, length, prefix)

def embed(sourceFile, outputFileName, prefix):
  with open(sourceFile, "rb") as f:
    f.seek(0, os.SEEK_END)
    length = f.tell()
    f.seek(0, os.SEEK_SET)
    serialize(sourceFile, outputFileName, f, length, prefix)

def process_all(sourceDirectory, outputDirectory):
  if not os.path.exists(outputDirectory):
    os.mkdir(outputDirectory)
  compress(os.path.join(sourceDirectory, 'index.html'),
           os.path.join(outputDirectory, 'index_html.h'), 'indexHtmlGz')
  compress(os.path.join(sourceDirectory, 'jquery.min.js'),
           os.path.join(outputDirectory, 'jquery_min_js.h'), 'jqueryJsGz')
  compress(os.path.join(sourceDirectory, 'jquery.mobile-1.5.0-rc1.min.js'),
           os.path.join(outputDirectory, 'jquery_mobile_js.h'), 'jqueryMobileJsGz')
  compress(os.path.join(sourceDirectory, 'jquery.mobile-1.5.0-rc1.min.css'),
           os.path.join(outputDirectory, 'jquery_mobile_css.h'), 'jqueryMobileCssGz')
  compress(os.path.join(sourceDirectory, 'jquery.simple.websocket.min.js'),
           os.path.join(outputDirectory, 'jquery_simple_websocket.h'), 'jquerySimpleWebSocketGz')
  compress(os.path.join(sourceDirectory, 'jqClock-lite.min.js'),
           os.path.join(outputDirectory, 'jq_clock.h'), 'jqClockGz')
  embed(os.path.join(sourceDirectory, 'ajax-loader.gif'),
        os.path.join(outputDirectory, 'ajax_loader.h'), 'ajaxLoader')
  embed(os.path.join(sourceDirectory, 'loco-32x32.png'),
        os.path.join(outputDirectory, 'loco_32x32.h'), 'loco32x32')
