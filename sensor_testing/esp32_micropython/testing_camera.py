from camera import Camera, GrabMode, PixelFormat, FrameSize, GainCeiling
import uos
import machine
from machine import Pin, SPI, WDT, SDCard
#import sdcard
#from sdcard import SDCard

# Camera construction and initialization
# These pins are just examples and if you use them just like that will get a watchdog error. Adapt them to your board!
camera = Camera(
    data_pins=[0,1,2,3,4,5,6,7],
    vsync_pin=6,
    href_pin=7,
    sda_pin=4,
    scl_pin=5,
    pclk_pin=13,
    xclk_pin=15,
    xclk_freq=20000000,
    powerdown_pin=-1,
    reset_pin=-1,
    pixel_format=PixelFormat.RGB565,
    frame_size=FrameSize.QVGA,
    jpeg_quality=15,
    fb_count=1,
    grab_mode=GrabMode.WHEN_EMPTY
)

# Capture and save the image to the SD card
img = camera.capture()

# Initialize the SDCard (substitute pins if needed based on your board)
sd = SDCard(slot=2, sck=Pin(39), mosi=Pin(38), miso=Pin(40), cs=Pin(5), freq=20000000)

#cs = Pin(5, machine.Pin.OUT)

#spi = SPI(1, polarity=0, phase=0, baudrate=1000000, sck=Pin(39), mosi=Pin(38), miso=Pin(40)) #baudrate=400000

#sd = sdcard.SDCard(spi, cs)#baudrate=8000000 #43 gave no res.

#sd = machine.SDCard(slot=2)
#vfs=os.VfsFat(sd)
#os.mount(vfs, "/sd")  # mount

#fn = open('/sd/textsd.txt', 'w')
#fn.write('some data')
#fn.close()

#os.listdir('/sd')    # list directory contents

print(uos.listdir("/"))
# Mount the SD card to the filesystem
uos.mount(sd, "/sd")

# Writing a file to the SD card
with open("/sd/image.jpg", "wb") as file:
    file.write(img)

# Confirm the file has been written by reading it
with open("/sd/image.jpg", "r") as file:
    print(file.read())

# Unmount the SD card when done
uos.umount("/sd")
print('TEST DONE')