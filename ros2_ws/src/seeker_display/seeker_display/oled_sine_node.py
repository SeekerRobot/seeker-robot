"""
oled_sine_node — Serve an animated sine wave to the OLED display via HTTP.

Generates a 1024-byte SSD1306 framebuffer at 10 Hz and serves it as a
persistent raw stream on GET /lcd_out (port lcd_serve_port, default 8390).
The ESP32 connects as an HTTP client and reads 1024-byte frames continuously.
No micro-ROS agent required.

SSD1306 framebuffer format (page-major):
  8 pages × 128 columns = 1024 bytes
  Each byte = 8 vertical pixels, LSB = topmost row of that page
  Pages run top-to-bottom (page 0 = rows 0-7, page 7 = rows 56-63)

Parameters
  lcd_serve_port  int  8390  HTTP port for the LCD stream
"""

import math
import queue

import rclpy
from rclpy.node import Node

from seeker_display.lcd_http_server import start_lcd_server

WIDTH = 128
HEIGHT = 64
PAGES = HEIGHT // 8  # 8


def set_pixel(fb: bytearray, x: int, y: int) -> None:
    if 0 <= x < WIDTH and 0 <= y < HEIGHT:
        fb[(y // 8) * WIDTH + x] |= 1 << (y % 8)


class OledSineNode(Node):
    def __init__(self):
        super().__init__("oled_sine")

        self.declare_parameter("lcd_serve_port", 8390)
        self._lcd_serve_port = (
            self.get_parameter("lcd_serve_port").get_parameter_value().integer_value
        )

        self._lcd_queue: queue.Queue[bytes] = queue.Queue(maxsize=1)
        self._t = 0.0

        # 10 Hz matches the OLED update rate
        self._timer = self.create_timer(0.1, self._tick)

        start_lcd_server(self._lcd_serve_port, self._lcd_queue, self.get_logger().info)
        self.get_logger().info(
            f"oled_sine: serving sine wave on :{self._lcd_serve_port}/lcd_out at 10 Hz"
        )

    def _tick(self):
        fb = bytearray(WIDTH * PAGES)

        for x in range(WIDTH):
            y = int(32 + 28 * math.sin(x * 0.15 + self._t))
            set_pixel(fb, x, y - 1)
            set_pixel(fb, x, y)
            set_pixel(fb, x, y + 1)

        try:
            self._lcd_queue.put_nowait(bytes(fb))
        except queue.Full:
            pass  # ESP32 hasn't consumed last frame — drop

        self._t += 0.2


def main(args=None):
    rclpy.init(args=args)
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
