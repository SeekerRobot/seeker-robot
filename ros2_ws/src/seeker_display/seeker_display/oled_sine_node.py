"""
oled_sine_node — Publish an animated sine wave to /mcu/lcd.

SSD1306 framebuffer format (column-major, from OledFrame.msg):
  - 8 pages × 128 columns = 1024 bytes
  - Each byte = 8 vertical pixels, LSB = topmost row of that page
  - Pages run top-to-bottom (page 0 = rows 0-7, page 7 = rows 56-63)
"""

import math

import rclpy
from mcu_msgs.msg import OledFrame
from rclpy.node import Node

WIDTH = 128
HEIGHT = 64
PAGES = HEIGHT // 8  # 8


def set_pixel(fb: bytearray, x: int, y: int) -> None:
    if 0 <= x < WIDTH and 0 <= y < HEIGHT:
        fb[(y // 8) * WIDTH + x] |= 1 << (y % 8)


class OledSineNode(Node):
    def __init__(self):
        super().__init__("oled_sine")
        self._pub = self.create_publisher(OledFrame, "/mcu/lcd", 10)
        self._t = 0.0
        # 10 Hz matches the OLED bridge's hard rate cap
        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info("oled_sine: publishing to /mcu/lcd at 10 Hz")

    def _tick(self):
        fb = bytearray(WIDTH * PAGES)

        for x in range(WIDTH):
            y = int(32 + 28 * math.sin(x * 0.15 + self._t))
            set_pixel(fb, x, y - 1)
            set_pixel(fb, x, y)
            set_pixel(fb, x, y + 1)

        msg = OledFrame()
        msg.framebuffer = list(fb)
        self._pub.publish(msg)
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
