import gzip
import os
import struct
import cStringIO

def serialize(sourceFile, targetFile, file, length, prefix):
    print 'Converting %s to %s...' % (sourceFile, targetFile)
    with open(targetFile, 'w') as f:
        f.write("#pragma once\n")
        f.write("const size_t %s_size = %s;\n" % (prefix, length))
        f.write("const uint8_t %s[] PROGMEM = {\n" % prefix)
        while True:
            block = file.read(16)
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

def compress(sourceFile, targetFile, prefix):
    gzFile = cStringIO.StringIO()
    with open(sourceFile) as file, gzip.GzipFile(mode='wb', fileobj=gzFile) as gz:
        gz.writelines(file)
    gzFile.seek(0, os.SEEK_END)
    length = gzFile.tell()
    gzFile.seek(0, os.SEEK_SET)
    serialize(sourceFile, targetFile, gzFile, length, prefix)

def embed(sourceFile, targetFile, prefix):
    with open(sourceFile, "rb") as file:
        file.seek(0, os.SEEK_END)
        length = file.tell()
        file.seek(0, os.SEEK_SET)
        serialize(sourceFile, targetFile, file, length, prefix)
