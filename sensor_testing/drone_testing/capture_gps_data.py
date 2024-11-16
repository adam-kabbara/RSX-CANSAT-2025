import serial
import os
import time

# Set up the serial connection
serial_port = '/dev/tty.usbserial-0001'  # Update this with your ESP32 serial port
baud_rate = 115200
timeout = 1  # Set timeout (in seconds) for serial communication

# Path to save the GPS data file
downloads_folder = os.path.expanduser('~/Downloads')
file_path = os.path.join(downloads_folder, 'gps_data.txt')

# Open the serial port with a timeout to prevent blocking
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=timeout)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

print("Reading data from ESP32...")

# Open the file to save GPS data
with open(file_path, 'w') as file:
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()  # Read line from serial input and decode
            if line:  # Only write if the line has data
                file.write(line + "\n")  # Save the line to the file with a newline character
                print(line)  # Print the data to the console
            else:
                print("No data received in this iteration.")

            # Optional: Stop after a set number of lines or time
            # if some_condition:
            #     break

        except serial.SerialException as e:
            print(f"Serial error: {e}")
            break
        except UnicodeDecodeError as e:
            print(f"Decoding error: {e}")
            continue  # Skip this line if it's not decodable
        except Exception as e:
            print(f"Unexpected error: {e}")
            break

print("DONE! :)")
