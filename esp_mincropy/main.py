from machine import Pin
from time import sleep

# Set up GPIO2 as output (usually connected to onboard LED)
led = Pin(2, Pin.OUT)

while True:
    led.value(1)  # LED ON
    sleep(1)      # Wait 1 second
    led.value(0)  # LED OFF
    sleep(1)      # Wait 1 second
