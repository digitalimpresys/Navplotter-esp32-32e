# Pre-build step: embeds web/index.html into the firmware as a gzip-compressed byte array (src/web_page.h).
import gzip, os

try:
    Import("env")  # provided by PlatformIO
    root = env["PROJECT_DIR"]
except NameError:
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

src = os.path.join(root, "web", "index.html")
dst = os.path.join(root, "src", "web_page.h")

with open(src, "rb") as f:
    data = gzip.compress(f.read(), 9, mtime=0)

lines = ["// Generated from web/index.html by tools/embed_web.py - do not edit.",
         "#pragma once", "#include <Arduino.h>", "",
         "const size_t INDEX_HTML_GZ_LEN = %d;" % len(data),
         "const uint8_t INDEX_HTML_GZ[] PROGMEM = {"]
for i in range(0, len(data), 20):
    lines.append("  " + ",".join(str(b) for b in data[i:i + 20]) + ",")
lines.append("};")
text = "\n".join(lines) + "\n"

if not os.path.exists(dst) or open(dst).read() != text:
    with open(dst, "w") as f:
        f.write(text)
    print("embed_web: %s -> %d bytes gzip" % (os.path.basename(src), len(data)))
