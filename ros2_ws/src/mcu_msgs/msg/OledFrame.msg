# /mcu/lcd — raw framebuffer for the SSD1306 128x64 OLED display.
#
# Format: SSD1306 native column-major layout.
#   - 8 "pages", each 128 bytes wide.
#   - Each byte represents 8 vertical pixels (LSB = topmost pixel in the page).
#   - Pages run top-to-bottom (page 0 = rows 0-7, page 7 = rows 56-63).
#   - Total: 1024 bytes per frame.
#
# The MCU queues incoming frames and renders them at a hard-capped 10 Hz.
# Publishing faster than 10 Hz is allowed but extra frames may be dropped.
# For high-rate streaming, use a webserver transport instead.

uint8[] framebuffer
