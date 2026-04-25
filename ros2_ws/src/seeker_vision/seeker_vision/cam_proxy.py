#!/usr/bin/env python3
"""
Proxies the ESP32 MJPEG camera stream to localhost so vision_core.py can
consume it without a direct route to the device IP.

Default:  http://192.168.8.51/cam  →  http://localhost:8080/stream
(camera lives on the satellite ESP32 at .51; main hexapod is .50)

The proxy parses individual JPEG frames out of the upstream multipart stream
and re-serves them in a clean, well-formed MJPEG response that OpenCV/FFmpeg
can reliably decode. With --flip the proxy decodes each frame, rotates 180°,
and re-encodes as JPEG — used when the camera is mounted upside down (YOLO's
COCO weights are trained on upright images and regress badly otherwise).

Usage:
    python3 cam_proxy.py
    python3 cam_proxy.py --source http://192.168.8.51/cam --port 8080 --flip
"""

import argparse
import http.server
import sys
import urllib.error
import urllib.request

_SOURCE_DEFAULT = "http://192.168.8.51/cam"
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


def _flip_jpeg(jpeg_bytes):
    """Decode a JPEG, rotate 180°, re-encode. Returns None on failure."""
    import cv2
    import numpy as np
    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return None
    rotated = cv2.rotate(img, cv2.ROTATE_180)
    ok, enc = cv2.imencode(".jpg", rotated, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if not ok:
        return None
    return enc.tobytes()


class _ProxyHandler(http.server.BaseHTTPRequestHandler):
    source_url: str  # injected on the class before serving
    flip: bool = False

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
                if self.flip:
                    flipped = _flip_jpeg(frame)
                    if flipped is not None:
                        frame = flipped
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
    parser.add_argument("--flip", action="store_true",
                        help="Rotate each frame 180° before re-serving (camera upside down)")
    args = parser.parse_args()

    _ProxyHandler.source_url = args.source
    _ProxyHandler.flip = args.flip

    server = http.server.ThreadingHTTPServer((args.host, args.port), _ProxyHandler)
    print(f"Proxying {args.source} -> http://{args.host}:{args.port}/stream"
          f"{' (flipped 180°)' if args.flip else ''}",
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nProxy stopped.")
        sys.exit(0)


if __name__ == "__main__":
    main()
