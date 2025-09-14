# minimal_uart_test.py
# Minimal UART RX test for Pico W (UART1, RX=GPIO 5)
from machine import UART, Pin
import time

# Set up UART1 with RX on GPIO 5
uart = UART(1, baudrate=420000, rx=Pin(5))

print("Listening for UART data on GPIO 5 (UART1 RX)...")

# Helper to decode CRSF RC channels from a payload
def decode_crsf_channels(payload):
    channels = []
    bits = 0
    value = 0
    for b in payload:
        value |= b << bits
        bits += 8
        if bits >= 11:
            channels.append(value & 0x7FF)
            value >>= 11
            bits -= 11
    return channels[:16]

buffer = b''

while True:
    data = uart.read()
    if data:
        buffer += data
        # Look for CRSF RC channel packet (0xC8, length, 0x16)
        while len(buffer) >= 24:  # 2 header + 22 payload
            if buffer[0] == 0xC8 and buffer[2] == 0x16:
                length = buffer[1]
                if len(buffer) >= length + 2:
                    payload = buffer[3:3+22]  # 22 bytes for 16 channels
                    channels = decode_crsf_channels(payload)
                    print("Channels:", channels)
                    buffer = buffer[length+2:]
                else:
                    break
            else:
                buffer = buffer[1:]
    time.sleep(0.01)