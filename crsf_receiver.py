# crsf_receiver.py
# MicroPython CRSF parser for Pico W with LED status indication
# Reads CRSF packets from UART, parses RC channels, allows channel callbacks, and sends telemetry
# LED flashes unique codes for receiver status

from machine import UART, Pin, PWM
import neopixel
import time
import math

CRSF_ADDRESS = 0xC8
CRSF_FRAME_RC_CHANNELS_PACKED = 0x16
CRSF_MAX_CHANNELS = 16

class CRSFParser:
    def __init__(self, uart_id=1, rx_pin=5, tx_pin=4, baudrate=420000):
        self.uart = UART(uart_id, baudrate=baudrate, rx=Pin(rx_pin), tx=Pin(tx_pin))
        self.callbacks = [None] * CRSF_MAX_CHANNELS
        self.last_channels = [0] * CRSF_MAX_CHANNELS
        self.buffer = b''
        self.last_packet_time = time.ticks_ms()
        self.last_rc_update = time.ticks_ms()
        self.rc_updated = False

    def register_callback(self, channel, func):
        if 0 <= channel < CRSF_MAX_CHANNELS:
            self.callbacks[channel] = func

    def parse(self):
        data = self.uart.read()
        if data:
            self.buffer += data
        # Process any valid CRSF packet (minimum 4 bytes: addr, len, type, crc)
        while len(self.buffer) >= 4:
            length = self.buffer[1]
            total_len = length + 2  # addr + len + payload + crc
            if len(self.buffer) < total_len:
                break  # Wait for more data
            packet = self.buffer[:total_len]
            self.handle_packet(packet)
            self.buffer = self.buffer[total_len:]

    def handle_packet(self, packet):
        if len(packet) < 4:
            return
        length = packet[1]
        if len(packet) != length + 2:
            return
        addr = packet[0]
        type_ = packet[2]
        self.last_packet_time = time.ticks_ms()
        if addr == CRSF_ADDRESS and type_ == CRSF_FRAME_RC_CHANNELS_PACKED:
            self.handle_rc_channels(packet[3:-1])
        else:
            pass

    def handle_rc_channels(self, payload):
        # 16 channels, 11 bits each, packed
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
        # Only update if changed
        for i, ch in enumerate(channels[:CRSF_MAX_CHANNELS]):
            if ch != self.last_channels[i]:
                self.last_channels[i] = ch
                if self.callbacks[i] is not None:
                    self.callbacks[i](ch) # type: ignore
                self.last_rc_update = time.ticks_ms()
                self.rc_updated = True

    def send_telemetry(self, payload):
        addr = 0xEA
        length = len(payload) + 2  # type + payload
        type_ = 0x1C  # Device info, for example
        packet = bytes([addr, length, type_]) + payload
        crc = self.crc8(packet)
        packet += bytes([crc])
        self.uart.write(packet)

    def crc8(self, data):
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0xD5
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

# Example usage with LED status indication
if __name__ == "__main__":
    ARM_CHANNEL = 4  # Change to your arm channel index (0-based)
    ARM_THRESHOLD = 1000  # Value above which is considered "armed"
    THROTTLE_CHANNEL = 2  # Throttle channel index (0-based)
    THROTTLE_HIGH = 600  # Value above which is considered "high throttle"
    THROTTLE_HOLD_MS = 1000  # How long throttle must be high to turn off LEDs

    # WS2812 (NeoPixel) setup
    NEOPIXEL_PIN = 15  # Change as needed
    NUM_PIXELS = 6     # Set to your number of LEDs
    np = neopixel.NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)

    # Use onboard LED for status (digital on/off only)
    onboard_led = Pin("LED", Pin.OUT)
    led_state = "Off"

    parser = CRSFParser()

    # AETR mapping: 0=Roll, 1=Pitch, 2=Throttle, 3=Rudder
    def on_roll(val):
        pass

    def on_pitch(val):
        pass

    def update_leds():
        pass

    def on_throttle(val):
        pass

    def on_arm(val):
        pass

    def on_rudder(val):
        pass

    def display_mode_update():
        pass

    parser.register_callback(0, on_roll)
    parser.register_callback(1, on_pitch)
    parser.register_callback(2, on_throttle)
    parser.register_callback(3, on_rudder)
    parser.register_callback(ARM_CHANNEL, on_arm)

    # LED status patterns
    def led_flash_pattern(pattern, duration_ms=1000):
        # pattern: list of (on, ms) tuples
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
            for on, ms_ in pattern:
                onboard_led.value(on)
                time.sleep_ms(ms_)

    # --- Non-blocking display mode state ---
    display_mode = 0  # 0=rainbow, 1=pulse
    display_step = 0
    display_last_update = time.ticks_ms()
    display_wait_ms = 20
    pulse_up = True
    pulse_steps = 50
    pulse_color = (0, 0, 255)

    # Main loop with status detection
    last_print = time.ticks_ms()
    PRINT_INTERVAL = 500  # ms
    while True:
        parser.parse()
        now = time.ticks_ms()
        # Periodic print of first 8 channel values
        if time.ticks_diff(now, last_print) > PRINT_INTERVAL:
            print("Channels:", parser.last_channels[:8])
            last_print = now
        