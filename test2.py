from machine import Pin, PWM
import time

led = PWM(Pin(25))
led.freq(1000)

while True:
    for duty in range(0, 65536, 4096):
        led.duty_u16(duty)
        time.sleep(0.05)