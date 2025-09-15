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
    THROTTLE_MIN = 200   # Approx. min raw value for 0% throttle (tune)
    THROTTLE_MAX = 1800  # Approx. max raw value for 100% throttle (tune)
    # New: roll/pitch channels and stick behavior
    ROLL_CHANNEL = 0
    PITCH_CHANNEL = 1
    STICK_MID = 1024  # CRSF 11-bit center
    STICK_DEADZONE = 300  # Deflection required to enable direction mode
    INVERT_ROLL = False   # Set True if left/right appear swapped
    INVERT_PITCH = False  # Set True if forward/back appear swapped
    # Directional display tuning
    DIR_SIGMA = 0.85      # Gaussian width in LEDs (controls 2-3 LED fade)
    DIR_MAX_BRIGHT = 255  # Max brightness at full deflection
    MIN_DEADZONE_GLOW = 0.02  # Fraction of throttle color used when stick is centered
    THROTTLE_HOLD_MS = 1500  # How long throttle must be high to turn off LEDs

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

    # --- Sticky idle sub-mode selection ---
    submode_selected = 0

    # --- Persistent color state for color wipe ---
    color_wipe_state = [(0,0,0)] * NUM_PIXELS

    # --- Persistent state for takeoff red fade ---
    fade_start_ms = None

    def display_mode_update(display_mode=0, target_float=0.0, colour_hue=0.0):
        global submode_selected
        if display_mode == 0:
            rx = roll_val - STICK_MID
            ry = pitch_val - STICK_MID
            SUBMODE_DEADZONE = 200
            # Only update submode_selected if stick is outside deadzone
            if abs(rx) > SUBMODE_DEADZONE or abs(ry) > SUBMODE_DEADZONE:
                if abs(rx) > abs(ry):
                    if rx > SUBMODE_DEADZONE:
                        submode_selected = 1  # right: color wipe
                    elif rx < -SUBMODE_DEADZONE:
                        submode_selected = 3  # left: segment pulse
                else:
                    if ry > SUBMODE_DEADZONE:
                        submode_selected = 0  # forward: comet
                    elif ry < -SUBMODE_DEADZONE:
                        submode_selected = 2  # back: twinkle

            # --- Idle sub-mode 0: Comet (forward) ---
            if submode_selected == 0:
                comet_len = 5
                comet_speed = 0.5  # rotations/sec
                t = (time.ticks_ms() / 1000.0) * comet_speed
                comet_pos = (t % 1.0) * NUM_PIXELS
                for i in range(NUM_PIXELS):
                    d = min(abs(i - comet_pos), NUM_PIXELS - abs(i - comet_pos))
                    fade = math.exp(-d * 1.2)
                    brightness = int(200 * fade)
                    if brightness < 2:
                        np[i] = (0, 0, 0)
                    else:
                        r, g, b = hsv_to_rgb((colour_hue + 0.1) % 1.0, 1.0, brightness / 255.0)
                        np[i] = (r, g, b)

            # --- Idle sub-mode 1: Color Wipe (right) ---
            elif submode_selected == 1:
                global color_wipe_state
                wipe_speed = 0.25  # rotations/sec
                t = (time.ticks_ms() / 1000.0) * wipe_speed
                wipe_pos = (t % 1.0) * NUM_PIXELS
                wipe_width = 1  # single point; increase for a tail
                # When wipe passes over an LED, update its color
                for i in range(NUM_PIXELS):
                    d = (i - wipe_pos + NUM_PIXELS) % NUM_PIXELS
                    if 0 <= d < wipe_width:
                        # Overwrite with new color at full brightness
                        r, g, b = hsv_to_rgb(colour_hue, 1.0, 1.0)
                        color_wipe_state[i] = (r, g, b)
                # Draw all LEDs at half brightness of their stored color
                for i in range(NUM_PIXELS):
                    r, g, b = color_wipe_state[i]
                    np[i] = (r//2, g//2, b//2)
                # Draw the wipe at full brightness (overwriting half-bright)
                for i in range(NUM_PIXELS):
                    d = (i - wipe_pos + NUM_PIXELS) % NUM_PIXELS
                    if 0 <= d < wipe_width:
                        r, g, b = color_wipe_state[i]
                        np[i] = (r, g, b)

            # --- Idle sub-mode 2: Twinkle (back) ---
            elif submode_selected == 2:
                twinkle_count = 4  # Number of 'on' LEDs in the pattern
                twinkle_period = 0.5  # seconds per pattern shift
                t = int(time.ticks_ms() / (twinkle_period * 1000))
                for i in range(NUM_PIXELS):
                    # Create a moving pattern: every Nth LED is on, pattern shifts by t
                    if (i + t) % (NUM_PIXELS // twinkle_count) == 0:
                        r, g, b = hsv_to_rgb((colour_hue + i / NUM_PIXELS) % 1.0, 0.2, 1.0)
                        np[i] = (r, g, b)
                    else:
                        np[i] = (0, 0, 0)

            # --- Idle sub-mode 3: Segment Pulse (left) ---
            elif submode_selected == 3:
                segs = [range(0,6), range(6,12), range(12,18), range(18,24)]
                pulse_speed = 1.0  # cycles/sec
                t = (time.ticks_ms() / 1000.0) * pulse_speed
                seg_idx = int((t % 4))
                for i in range(NUM_PIXELS):
                    if i in segs[seg_idx]:
                        r, g, b = hsv_to_rgb((colour_hue + seg_idx * 0.25) % 1.0, 1.0, 1.0)
                        np[i] = (r, g, b)
                    else:
                        np[i] = (0, 0, 0)
        elif display_mode == 1:
            # Armed: slow pulsed red (off to half brightness)
            pulse_speed = 2000  # ms for a full pulse cycle
            t = (time.ticks_ms() % pulse_speed) / pulse_speed  # 0..1
            # Sine wave for smooth pulse (0..1)
            pulse = (math.sin(2 * math.pi * t - math.pi / 2) + 1) / 2
            brightness = int(128 * pulse)  # 0..128 (half brightness)
            for i in range(NUM_PIXELS):
                np[i] = (brightness, 0, 0)
        elif display_mode == 3:
            # Post-takeoff: directional lobe colored by throttle; deadzone shows faint throttle-colored glow
            rx = roll_val - STICK_MID
            ry = pitch_val - STICK_MID
            if INVERT_ROLL:
                rx = -rx
            if INVERT_PITCH:
                ry = -ry
            # Stick magnitude for lobe brightness
            mag = (math.sqrt(rx*rx + ry*ry) - STICK_DEADZONE)
            if mag < 0:
                mag = 0
            mag = mag / (STICK_MID - STICK_DEADZONE)
            if mag > 1:
                mag = 1
            # Throttle normalization 0..1 and color ramp green->orange->red
            t = (throttle_val - THROTTLE_MIN) / (THROTTLE_MAX - THROTTLE_MIN)
            if t < 0:
                t = 0
            if t > 1:
                t = 1
            if t <= 0.5:
                u = t / 0.5  # 0..1
                base_r = int(0 + (255 - 0) * u)
                base_g = int(255 + (165 - 255) * u)
                base_b = 0
            else:
                u = (t - 0.5) / 0.5  # 0..1
                base_r = 255
                base_g = int(165 + (0 - 165) * u)
                base_b = 0
            # Deadzone glow color (always available for blending)
            glow_r = int(base_r * MIN_DEADZONE_GLOW)
            glow_g = int(base_g * MIN_DEADZONE_GLOW)
            glow_b = int(base_b * MIN_DEADZONE_GLOW)
            # Angle mapping to LED target around ring (align +X to right center)
            ang = math.atan2(ry, rx)
            if ang < 0:
                ang += 2 * math.pi
            target_pos = (ang * (NUM_PIXELS / (2 * math.pi))) + 8.5
            target_pos = target_pos % NUM_PIXELS
            # Gaussian lobe across ring
            two_sigma2 = 2 * DIR_SIGMA * DIR_SIGMA
            # Smooth fade between glow (0) and lobe (1)
            blend = mag * mag * (3 - 2 * mag)  # smoothstep
            inv_blend = 1.0 - blend
            for i in range(NUM_PIXELS):
                d = abs(i - target_pos)
                d = min(d, NUM_PIXELS - d)
                w = math.exp(-(d * d) / two_sigma2)
                # Lobe intensity (do not multiply by mag; blend handles strength)
                lr = int(base_r * w)
                lg = int(base_g * w)
                lb = int(base_b * w)
                r = int(glow_r * inv_blend + lr * blend)
                g = int(glow_g * inv_blend + lg * blend)
                b = int(glow_b * inv_blend + lb * blend)
                # Clamp
                if r < 0: r = 0
                if g < 0: g = 0
                if b < 0: b = 0
                if r > 255: r = 255
                if g > 255: g = 255
                if b > 255: b = 255
                np[i] = (r, g, b)
        elif display_mode == 2:
            # Takeoff: flash pairs of side LEDs from back to front, and fade out red on 0,5,12,17
            global fade_start_ms
            pairs = [(23,6), (22,7), (21,8), (20,9), (19,10), (18,11)]
            num_pairs = len(pairs)
            fade_leds = [0, 5, 12, 17]
            now_ms = time.ticks_ms()
            # Latch fade start time when entering takeoff mode
            if fade_start_ms is None:
                fade_start_ms = now_ms
            fade_elapsed = now_ms - fade_start_ms
            if fade_elapsed < THROTTLE_HOLD_MS:
                fade_phase = fade_elapsed / THROTTLE_HOLD_MS
                fade_val = int(255 * (1.0 - fade_phase))
            else:
                fade_val = 0
            # Green wave: always fast
            flash_speed = 20.0  # pairs per second (adjust as desired)
            t = (now_ms / 1000.0) * flash_speed
            idx = int(t) % num_pairs
            on_leds = pairs[idx]
            for i in range(NUM_PIXELS):
                if i in on_leds:
                    np[i] = (0, 255, 0)
                elif i in fade_leds:
                    np[i] = (fade_val, 0, 0)
                else:
                    np[i] = (0, 0, 0)
        # Reset fade latch when not in takeoff mode
        else:
            global fade_start_ms
            fade_start_ms = None
        if display_mode == 4:
            # Precise directional highlight with per-LED fade around ring
            # LED layout: Back[0..5], Right[6..11], Front[12..17], Left[18..23]
            # Compute deflection vector and normalize
            rx = roll_val - STICK_MID
            ry = pitch_val - STICK_MID
            if INVERT_ROLL:
                rx = -rx
            if INVERT_PITCH:
                ry = -ry
            # Magnitude for brightness scaling (0..1)
            mag = (math.sqrt(rx*rx + ry*ry) - STICK_DEADZONE)
            if mag < 0:
                mag = 0
            mag = mag / (STICK_MID - STICK_DEADZONE)
            if mag > 1:
                mag = 1
            if mag == 0:
                for i in range(NUM_PIXELS):
                    np[i] = (0, 0, 0)
                np.write()
                return
            # Angle of vector (rx, ry), atan2 returns angle from +X
            ang = math.atan2(ry, rx)
            if ang < 0:
                ang += 2 * math.pi
            # Map angle to LED index around ring; ensure +X (right) -> center of right segment (8.5)
            target_pos = (ang * (NUM_PIXELS / (2 * math.pi))) + 8.5
            # Wrap into [0, NUM_PIXELS)
            target_pos = target_pos % NUM_PIXELS
            # Gaussian falloff around target_pos with circular distance
            two_sigma2 = 2 * DIR_SIGMA * DIR_SIGMA
            max_b = int(DIR_MAX_BRIGHT * mag)
            for i in range(NUM_PIXELS):
                d = abs(i - target_pos)
                d = min(d, NUM_PIXELS - d)
                w = math.exp(-(d * d) / two_sigma2)
                b = int(max_b * w)
                if b < 2:
                    np[i] = (0, 0, 0)
                else:
                    # Use cool white for clarity
                    np[i] = (b, b, b)
        np.write()

    def on_arm(val):
        global armed, display_mode, post_takeoff, fade_start_ms
        armed = val > ARM_THRESHOLD
        if armed:
            fade_start_ms = None  # Reset fade on arming
        if not armed:
            # Disarm: turn off post_takeoff mode
            post_takeoff = False
        update_display_mode()

    def on_throttle(val):
        global throttle_high, display_mode, throttle_high_start, was_throttle_high, armed, throttle_val
        throttle_val = val
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

    # Track roll/pitch for directional idle mode
    roll_val = STICK_MID
    pitch_val = STICK_MID
    throttle_val = THROTTLE_MIN

    def on_roll(val):
        global roll_val
        roll_val = val
        update_display_mode()

    def on_pitch(val):
        global pitch_val
        pitch_val = val
        update_display_mode()

    parser.register_callback(ROLL_CHANNEL, on_roll)
    parser.register_callback(PITCH_CHANNEL, on_pitch)

    # LED status patterns
    def led_flash_pattern(pattern, duration_ms=1000):
        # pattern: list of (on, ms) tuples
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
            for on, ms_ in pattern:
                onboard_led.value(on)
                time.sleep_ms(ms_)

    # --- Non-blocking display mode state ---
    display_mode = 0  # 0=idle animation, 1=armed, 2=armed+throttle, 3=post takeoff, 4=idle directional
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
            # Idle: switch to directional when stick deflected beyond deadzone
            rx = roll_val - STICK_MID
            ry = pitch_val - STICK_MID
            if INVERT_ROLL:
                rx = -rx
            if INVERT_PITCH:
                ry = -ry
            if (abs(rx) > STICK_DEADZONE) or (abs(ry) > STICK_DEADZONE):
                display_mode = 4
            else:
                display_mode = 0

    # Main loop with status detection
    last_print = time.ticks_ms()
    PRINT_INTERVAL = 500  # ms
    while True:
        parser.parse()
        now = time.ticks_ms()
        # Continuously re-evaluate display mode to catch stick changes in idle
        update_display_mode()
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
