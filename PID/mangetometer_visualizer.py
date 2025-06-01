from vpython import canvas, vector, cylinder, cone, rate
import math
import serial

#===========================================
# RIGHT NOW THIS CODE READS SERIAL DATA TO GET THE VISUAL STUFF OFF OF COM4, YOU CAN JUST ASK CHAT TO CHANGE ITS INPUT TO DATA FROM AN ANTENNA
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


# === Setup Serial Port ===
# Replace 'COM3' with your actual port and set the correct baud rate
ser = serial.Serial('COM4', 115200, timeout=1)

# === Setup canvas ===
scene = canvas(title='Live Rocket Orientation', width=800, height=600)
scene.camera.pos = vector(0, 0, 10)
scene.camera.axis = vector(0, 0, -10)

# === Create rocket ===
rocket_body = cylinder(pos=vector(0, -2, 0), axis=vector(0, 4, 0), radius=0.3, color=vector(0.8, 0.1, 0.1))
rocket_nose = cone(pos=vector(0, 2, 0), axis=vector(0, 1, 0), radius=0.3, color=vector(0.9, 0.5, 0.1))

# === Orientation function ===
def set_orientation(yaw, pitch, roll):
    yaw = math.radians(yaw)
    pitch = math.radians(pitch)
    roll = math.radians(roll)

    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    R = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr]
    ]

    up = vector(R[0][2], R[1][2], R[2][2])
    axis = vector(R[0][1], R[1][1], R[2][1])

    rocket_body.axis = axis * 4
    rocket_body.up = up
    rocket_nose.pos = rocket_body.pos + rocket_body.axis
    rocket_nose.axis = rocket_body.axis.norm()
    rocket_nose.up = up

# === NaN-safe parser ===
def safe_parse(value, fallback):
    try:
        v = float(value)
        if math.isnan(v):
            return fallback
        return v
    except:
        return fallback

# === Store last good values ===
last_yaw = 0.0
last_pitch = 0.0
last_roll = 0.0

# === Main loop ===
print("Reading serial data. Press Ctrl+C to exit.")
try:
    while True:
        rate(30)  # 30 FPS
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue

        parts = line.split(',')
        if len(parts) != 3:
            continue

        yaw = safe_parse(parts[0], last_yaw)
        pitch = safe_parse(parts[1], last_pitch)
        roll = safe_parse(parts[2], last_roll)
        
        print(yaw, pitch, roll)
        
        # Save last good values
        last_yaw = yaw
        last_pitch = pitch
        last_roll = roll

        # Update orientation
        set_orientation(yaw, pitch, roll)

except KeyboardInterrupt:
    print("Exiting simulation.")
    ser.close()
