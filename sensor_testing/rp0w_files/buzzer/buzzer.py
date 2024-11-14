import RPi.GPIO as GPIO
import time

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin to which the buzzer is connected
buzzer_pin = 18

# Set up the GPIO pin as an output
GPIO.setup(buzzer_pin, GPIO.OUT)

try:
    while True:
        # Turn on the buzzer
        GPIO.output(buzzer_pin, GPIO.HIGH)
        print("Buzzer ON")
        
        # Wait for 1 second
        time.sleep(1)
        
        # Turn off the buzzer
        GPIO.output(buzzer_pin, GPIO.LOW)
        print("Buzzer OFF")
        
        # Wait for 1 second
        time.sleep(1)

except KeyboardInterrupt:
    # If the user presses Ctrl+C, cleanup the GPIO settings
    GPIO.cleanup()
