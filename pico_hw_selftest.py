import time
from machine import Pin, ADC

# Onboard LED test
try:
    led = Pin("LED", Pin.OUT)
    for _ in range(3):
        led.value(1)
        time.sleep(0.2)
        led.value(0)
        time.sleep(0.2)
    print("Onboard LED: OK")
except Exception as e:
    print("Onboard LED: FAIL", e)

# Internal temperature sensor (ADC4)
try:
    sensor_temp = ADC(4)
    reading = sensor_temp.read_u16()
    print("Internal temperature ADC reading:", reading)
except Exception as e:
    print("Internal temperature sensor: FAIL", e)

# WiFi test (Pico W only)
try:
    import network
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    print("WiFi MAC:", wlan.config('mac'))
    print("WiFi: OK")
except Exception as e:
    print("WiFi: FAIL", e)

# Internal flash test
try:
    with open('test_flash.txt', 'w') as f:
        f.write('flash test')
    with open('test_flash.txt', 'r') as f:
        data = f.read()
    print("Internal flash read/write: OK" if data == 'flash test' else "Internal flash: FAIL")
except Exception as e:
    print("Internal flash: FAIL", e)
