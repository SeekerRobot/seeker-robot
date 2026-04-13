#!/usr/bin/env python3
"""
Proxies the ESP32 MJPEG camera stream to localhost so vision_core.py can
consume it without a direct route to the device IP.

Default:  http://192.168.8.50/cam  →  http://localhost:8080/stream

The proxy parses individual JPEG frames out of the upstream multipart stream
and re-serves them in a clean, well-formed MJPEG response that OpenCV/FFmpeg
can reliably decode.

Usage:
    python3 cam_proxy.py
    python3 cam_proxy.py --source http://192.168.8.50/cam --port 8080
"""

import argparse
import http.server
import sys
import urllib.error
import urllib.request

_SOURCE_DEFAULT = "http://192.168.8.50/cam"
_HOST_DEFAULT = "localhost"
_PORT_DEFAULT = 8080
_BOUNDARY = b"frame"
_READ = 4096


def _iter_frames(resp):
    """
    Yield raw JPEG bytes from a multipart/x-mixed-replace HTTP response.

    Parses Content-Length headers so each frame is extracted exactly — no
    reliance on JPEG SOI/EOI markers. Buffers across read() calls so partial
    chunks never stall the iterator.
    """
    ct = resp.headers.get("Content-Type", "")
    boundary = None
    for segment in ct.split(";"):
        segment = segment.strip()
        if segment.startswith("boundary="):
            boundary = segment[len("boundary="):].strip('"').encode()
            break
    if boundary is None:
        return

    sep = b"--" + boundary
    buf = bytearray()

    while True:
        chunk = resp.read(_READ)
        if not chunk:
            break
        buf.extend(chunk)

        while True:
            idx = buf.find(sep)
            if idx == -1:
                # Keep enough tail to detect a boundary split across reads
                del buf[:-len(sep)]
                break

            after_sep = idx + len(sep)
            # Skip the boundary terminator (\r\n or --)
            eol = buf.find(b"\r\n", after_sep)
            if eol == -1:
                break
            headers_start = eol + 2

            # Find end of part headers
            headers_end = buf.find(b"\r\n\r\n", headers_start)
            if headers_end == -1:
                break

            raw_headers = buf[headers_start:headers_end].decode("latin-1")
            content_length = -1
            for line in raw_headers.split("\r\n"):
                if line.lower().startswith("content-length:"):
                    content_length = int(line.split(":", 1)[1].strip())
                    break

            data_start = headers_end + 4
            if content_length < 0 or len(buf) < data_start + content_length:
                break  # wait for more data

            yield bytes(buf[data_start:data_start + content_length])
            del buf[:data_start + content_length]


class _ProxyHandler(http.server.BaseHTTPRequestHandler):
    source_url: str  # injected on the class before serving

    def log_message(self, fmt, *args):
        pass  # suppress per-request noise; errors are printed explicitly

    def do_GET(self):
        if self.path != "/stream":
            self.send_error(404, "Only /stream is available")
            return

        try:
            upstream = urllib.request.urlopen(self.source_url)
        except urllib.error.URLError as exc:
            print(f"[proxy] upstream error: {exc.reason}", flush=True)
            self.send_error(502, f"Cannot reach upstream: {exc.reason}")
            return

        self.send_response(200)
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace;boundary={_BOUNDARY.decode()}")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()

        try:
            for frame in _iter_frames(upstream):
                part = (
                    b"--" + _BOUNDARY + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(frame)).encode() + b"\r\n"
                    b"\r\n" + frame + b"\r\n"
                )
                self.wfile.write(part)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass  # client disconnected cleanly
        finally:
            upstream.close()


def main():
    parser = argparse.ArgumentParser(description="MJPEG proxy for the ESP32 camera")
    parser.add_argument("--source", default=_SOURCE_DEFAULT,
                        help=f"Upstream MJPEG URL (default: {_SOURCE_DEFAULT})")
    parser.add_argument("--host", default=_HOST_DEFAULT,
                        help=f"Bind address (default: {_HOST_DEFAULT})")
    parser.add_argument("--port", type=int, default=_PORT_DEFAULT,
                        help=f"Listen port (default: {_PORT_DEFAULT})")
    args = parser.parse_args()

    _ProxyHandler.source_url = args.source

    server = http.server.ThreadingHTTPServer((args.host, args.port), _ProxyHandler)
    print(f"Proxying {args.source} -> http://{args.host}:{args.port}/stream",
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nProxy stopped.")
        sys.exit(0)


if __name__ == "__main__":
    main()
