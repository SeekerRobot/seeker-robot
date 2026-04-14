"""
lcd_http_server — shared helper for serving SSD1306 framebuffers over HTTP.

Provides start_lcd_server(port, frame_queue, log_fn) which starts a daemon
thread serving GET /lcd_out as a persistent raw byte stream of 1024-byte
SSD1306 framebuffers.  Suitable for any node or script that drives the OLED
display via the HTTP transport.
"""

import queue
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Callable, Optional


def start_lcd_server(
    port: int,
    frame_queue: "queue.Queue[bytes]",
    log_fn: Optional[Callable[[str], None]] = None,
) -> None:
    """Start a persistent HTTP server serving SSD1306 framebuffers.

    Listens on 0.0.0.0:port.  For GET /lcd_out the server streams frames
    from *frame_queue* as a continuous raw byte stream until the client
    disconnects.  Non-blocking: runs in a daemon thread.

    Args:
        port:        TCP port to listen on.
        frame_queue: Queue[bytes] of 1024-byte SSD1306 framebuffers.
                     maxsize=1 is recommended so the ESP32 always gets the
                     latest frame (stale frames are dropped).
        log_fn:      Optional callable(str) for connect/disconnect messages.
    """
    def _log(msg: str) -> None:
        if log_fn:
            log_fn(msg)

    class _LcdHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/lcd_out":
                self._serve_stream()
            else:
                self.send_response(404)
                self.end_headers()

        def _serve_stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.end_headers()
            _log("LCD stream connected")
            try:
                while True:
                    try:
                        frame = frame_queue.get(timeout=1.0)
                    except queue.Empty:
                        continue
                    self.wfile.write(frame)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            _log("LCD stream disconnected")

        def log_message(self, fmt, *args):
            pass  # suppress per-request logs

    server = HTTPServer(("0.0.0.0", port), _LcdHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    _log(f"HTTP LCD server listening on :{port}")
