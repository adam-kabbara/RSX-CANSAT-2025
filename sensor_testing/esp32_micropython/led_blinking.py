import machine
import time

# Set up the LED pin (usually GPIO2)
led = machine.Pin(2, machine.Pin.OUT)

while True:
    led.value(1)  # Turn the LED on
    print("LED is ON")
    time.sleep(1)  # Wait for 1 second
    led.value(0)  # Turn the LED off
    print("LED is OFF")
    time.sleep(1)  # Wait for 1 second