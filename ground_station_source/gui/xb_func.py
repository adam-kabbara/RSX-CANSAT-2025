import serial.tools.list_ports
from serial import Serial, SerialException

# -------- #
BAUD_RATE = 57600
# -------- #

def getPorts():
    return list(serial.tools.list_ports.comports())
    
def portToSerial(port):
    try:
        ser = Serial(port, BAUD_RATE, timeout=0)
        if not ser.is_open:
            return None, "ERROR: Serial connection failed to open"
        else:
            return ser, "OK"
    except SerialException as e:
        return None, f"ERROR: {e}"
    except Exception as e:
        return None, f"ERROR: {e}"
    
def 