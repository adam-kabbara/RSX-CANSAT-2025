import machine
import os
import sdcard
import uos

# Define pins based on your ESP32
cs = machine.Pin(5, machine.Pin.OUT)   # Chip select
sck = machine.Pin(39, machine.Pin.OUT)
mosi = machine.Pin(38, machine.Pin.OUT)
miso = machine.Pin(40, machine.Pin.IN)
spi = machine.SPI(2, baudrate=1000000, sck=sck, mosi=mosi, miso=miso)

write80_buf = bytearray(2)
write80_buf[0] = 0x80
write80_buf[1] = 0x80

cs.off()

spi.write(write80_buf)

cs.on()

'''
# Initialize the SD card
sd = sdcard.SDCard(spi, sd_cs)

# Mount the filesystem
vfs = uos.VfsFat(sd)
uos.mount(vfs, '/sd')

# List files on the SD card
print("Files on SD card:")
print(uos.listdir('/sd'))

# Rmead a file
try:
    with open('/sd/yourfile.txt', 'r') as f:
        print(f.read())
except OSError as e:
    print("Error reading file:", e)

# Unmount the filesystem when done
uos.umount('/sd')
'''