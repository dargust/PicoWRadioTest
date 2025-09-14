# crsf_receiver.py
# MicroPython CRSF parser for Pico W with LED status indication
# Reads CRSF packets from UART, parses RC channels, allows channel callbacks, and sends telemetry
# LED flashes unique codes for receiver status

from machine import UART, Pin, PWM
# Detect if USB is connected (VBUS sense on GPIO24)
usb_connected = Pin(24, Pin.IN).value() == 1

# Safe print: only prints if USB is connected
def safe_print(*args, **kwargs):
    if usb_connected:
        print(*args, **kwargs)
import neopixel
import time
import math

CRSF_ADDRESS = 0xC8
CRSF_FRAME_RC_CHANNELS_PACKED = 0x16
CRSF_MAX_CHANNELS = 16

class CRSFParser:
    def __init__(self, uart_id=1, rx_pin=5, tx_pin=4, baudrate=420000):
        # You can try lowering baudrate (e.g., 115200) if you suspect timing issues
        # e.g., baudrate=115200
        self.uart = UART(uart_id, baudrate=baudrate, rx=Pin(rx_pin), tx=Pin(tx_pin), rxbuf=512)
        self.buffer_timeout_ms = 20  # Max time to wait for a full packet
        self.last_buffer_time = time.ticks_ms()
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
        now = time.ticks_ms()
        if data:
            self.buffer += data
            self.last_buffer_time = now
        # Buffer timeout: if buffer is stuck for too long, reset
        if self.buffer and time.ticks_diff(now, self.last_buffer_time) > self.buffer_timeout_ms:
            safe_print("CRSF buffer timeout/reset. Clearing buffer.")
            self.buffer = b''
        # Process any valid CRSF packet (minimum 4 bytes: addr, len, type, crc)
        while len(self.buffer) >= 4:
            length = self.buffer[1]
            total_len = length + 2  # addr + len + payload + crc
            if len(self.buffer) < total_len:
                break  # Wait for more data
            packet = self.buffer[:total_len]
            # Defensive: check for obviously bad length
            if length > 64:
                safe_print(f"CRSF: Malformed packet, length={length}, discarding 1 byte.")
                self.buffer = self.buffer[1:]
                continue
            self.handle_packet(packet)
            self.buffer = self.buffer[total_len:]

    def handle_packet(self, packet):
        if len(packet) < 4:
            safe_print("CRSF: Packet too short, skipping.")
            return
        length = packet[1]
        if len(packet) != length + 2:
            safe_print(f"CRSF: Packet length mismatch (expected {length+2}, got {len(packet)}), skipping.")
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
    THROTTLE_HOLD_MS = 4000  # How long throttle must be high to turn off LEDs

    # WS2812 (NeoPixel) setup
    NEOPIXEL_PIN = 15  # Change as needed
    NUM_PIXELS = 6 * 4     # Set to your number of LEDs
    np = neopixel.NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)

    # Use onboard LED for status (digital on/off only)
    onboard_led = Pin("LED", Pin.OUT)
    led_state = "Off"

    parser = CRSFParser()

    def hsv_to_rgb(h, s, v):
        # h in [0,1], s in [0,1], v in [0,1]
        i_h = int(h * 6)
        f = (h * 6) - i_h
        p = int(255 * v * (1 - s))
        q = int(255 * v * (1 - f * s))
        t = int(255 * v * (1 - (1 - f) * s))
        v_ = int(255 * v)
        i_h = i_h % 6
        if i_h == 0:
            return v_, t, p
        elif i_h == 1:
            return q, v_, p
        elif i_h == 2:
            return p, v_, t
        elif i_h == 3:
            return p, q, v_
        elif i_h == 4:
            return t, p, v_
        else:
            return v_, p, q

    def display_mode_update(display_mode=0, target_float=0.0, colour_hue=0.0):
        # Animate LEDs with a looping effect: distance wraps around the ends
        if display_mode == 0:
            target_pos = target_float * (NUM_PIXELS - 1)
            for i in range(NUM_PIXELS):
                # Calculate shortest distance on a ring (loop)
                dist = min(abs(i - target_pos), NUM_PIXELS - abs(i - target_pos))
                # Stronger falloff: increase the exponent denominator
                brightness = int(100 * math.exp(-dist * 2.0))
                brightness = max(0, min(255, brightness))
                # Convert hue (0..1) to RGB using helper
                r, g, b = hsv_to_rgb(colour_hue % 1.0, 1.0, brightness / 255.0)
                np[i] = (r, g, b)
        elif display_mode == 1:
            # All leds slowly pulse red when armed
            brightness = int((math.sin(time.ticks_ms() / 500) + 1) / 2 * 255)
            for i in range(NUM_PIXELS):
                np[i] = (brightness, 0, 0)
        elif display_mode == 2:
            # All leds solid green when armed + throttle
            for i in range(NUM_PIXELS):
                np[i] = (0, 255, 0)
        elif display_mode == 3:
            # All leds off after takeoff
            for i in range(NUM_PIXELS):
                np[i] = (0, 0, 0)
        np.write()

    def on_arm(val):
        global armed, display_mode, post_takeoff
        armed = val > ARM_THRESHOLD
        if not armed:
            # Disarm: turn off post_takeoff mode
            post_takeoff = False
        update_display_mode()

    def on_throttle(val):
        global throttle_high, display_mode, throttle_high_start, was_throttle_high, armed
        throttle_high = val > THROTTLE_HIGH
        # Only start timer when armed and throttle just went high
        if armed and throttle_high and not was_throttle_high:
            throttle_high_start = time.ticks_ms()
        # If throttle drops or disarmed, reset timer
        if not (armed and throttle_high):
            throttle_high_start = time.ticks_ms()
        was_throttle_high = throttle_high
        update_display_mode()

    parser.register_callback(2, on_throttle)
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
    display_mode = 0  # 0=idle animation, 1=armed, 2=armed+throttle, 3=post takeoff
    armed = False
    throttle_high = False
    post_takeoff = False
    display_last_update = time.ticks_ms()
    display_wait_ms = 40  # Update LEDs every 40ms for better performance
    display_target = 0.0
    display_colour_hue = 0.0
    throttle_high_start = time.ticks_ms()
    was_throttle_high = False
    

    def update_display_mode():
        global display_mode, armed, throttle_high, post_takeoff
        if post_takeoff:
            display_mode = 3
        elif armed and throttle_high:
            display_mode = 2
        elif armed:
            display_mode = 1
        else:
            display_mode = 0

    # Main loop with status detection
    last_print = time.ticks_ms()
    PRINT_INTERVAL = 500  # ms
    while True:
        parser.parse()
        now = time.ticks_ms()
        # Periodic print of first 8 channel values
        if time.ticks_diff(now, last_print) > PRINT_INTERVAL:
            safe_print("Channels:", parser.last_channels[:8])
            last_print = now
        # Periodic display mode update
        if time.ticks_diff(now, display_last_update) > display_wait_ms:
            # Scale increments to match new update interval (was 0.01/0.002 per 10ms, now 0.04/0.008 per 40ms)
            display_target += 0.04
            if display_target > 1.0:
                display_target = 0.0
            display_colour_hue += 0.008
            if display_colour_hue > 1.0:
                display_colour_hue = 0.0
            display_mode_update(display_mode, display_target, display_colour_hue)
            display_last_update = now
        # Check that armed and throttle high is held for a set amount of time
        # then set post_takeoff mode
        if armed and throttle_high:
            if time.ticks_diff(now, throttle_high_start) > THROTTLE_HOLD_MS:
                post_takeoff = True
                update_display_mode()
        # Yield to allow UART/other background tasks
        time.sleep_ms(1)
