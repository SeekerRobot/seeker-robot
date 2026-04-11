#!/usr/bin/env python3
"""
oled_sine.py — Serve an animated sine wave to the OLED display via HTTP.

Usage (inside the container, with workspace sourced):
    python3 ~/scripts/oled_sine.py

The sine wave scrolls across the 128x64 SSD1306 display at ~10 Hz.
The ESP32 fetches frames over HTTP (no micro-ROS agent required).
"""

import math
import queue
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node

# SSD1306 native column-major layout:
#   8 pages × 128 columns = 1024 bytes
#   Each byte = 8 vertical pixels, LSB = topmost row of that page
WIDTH  = 128
HEIGHT = 64
PAGES  = HEIGHT // 8
LCD_PORT = 8384


def set_pixel(fb: bytearray, x: int, y: int) -> None:
    if 0 <= x < WIDTH and 0 <= y < HEIGHT:
        fb[(y // 8) * WIDTH + x] |= 1 << (y % 8)


class OledSineNode(Node):
    def __init__(self):
        super().__init__("oled_sine")
        self._lcd_queue: queue.Queue[bytes] = queue.Queue(maxsize=1)
        self._t = 0.0
        # Publish at 10 Hz
        self._timer = self.create_timer(0.1, self._tick)
        self._start_lcd_server()
        self.get_logger().info(
            f"oled_sine started — serving on :{LCD_PORT}/lcd_out at 10 Hz"
        )

    def _tick(self):
        fb = bytearray(WIDTH * PAGES)

        # Sine wave: amplitude 28 px, centred at row 32
        for x in range(WIDTH):
            y = int(32 + 28 * math.sin(x * 0.15 + self._t))
            set_pixel(fb, x, y)
            set_pixel(fb, x, y - 1)
            set_pixel(fb, x, y + 1)

        try:
            self._lcd_queue.put_nowait(bytes(fb))
        except queue.Full:
            pass
        self._t += 0.2

    def _start_lcd_server(self):
        node = self

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
                node.get_logger().info("LCD stream connected")
                try:
                    while True:
                        try:
                            frame = node._lcd_queue.get(timeout=1.0)
                        except queue.Empty:
                            continue
                        self.wfile.write(frame)
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                node.get_logger().info("LCD stream disconnected")

            def log_message(self, fmt, *args):
                pass

        server = HTTPServer(("0.0.0.0", LCD_PORT), _LcdHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f"HTTP LCD server listening on :{LCD_PORT}")


def main():
    rclpy.init()
    node = OledSineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
