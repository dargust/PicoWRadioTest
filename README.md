copy main.py into a micropython flashed raspberry pi pico w

led pin is dio24

elrs is using uart 1 (DIO4/5)

setup to use 24 w2821 leds by default, can be changed

reacts to arm/throttle/roll/pitch inputs

default in standby until throttle>50% (single red led for power on, second led for elrs status, red not communicating, blue comms but no throttle, green all good)

then in idle 1 (comet trail) if stick in deadzone
push stick in any cardinal direction to select different modes
 - forward (comet)
 - right (solid colour overwrite)
 - back (pattern)
 - left (side)

arm to change from idle to armed mode (slow red pulse)
throttle up while armed to change from armed mode to race start (green runway strip)
hold throttle up to enter race input mode (colour depends on throttle, position depending on roll/pitch)
