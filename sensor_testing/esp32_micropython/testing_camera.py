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


# Capture an image
img = camera.capture()
if img is None:
    print("Image capture failed!")
    machine.reset()

# SD card setup
cs_pin = Pin(5, Pin.OUT)
cs_pin.value(1)  # Start with CS high (deselected)
sd = SDCard(slot=2, sck=Pin(39), mosi=Pin(40), miso=Pin(38), cs=cs_pin, freq=20000000)

# Mount the SD card and write image
try:
    cs_pin.value(0)  # Select the SD card
    uos.mount(sd, "/sd")
    print("SD card mounted successfully")

    # Write the image to the SD card
    with open("/sd/image.jpg", "wb") as file:
        file.write(img)
    print("Image written successfully")

    # Read back the file to confirm
    with open("/sd/image.jpg", "rb") as file:
        data = file.read()
        print("Read back image size:", len(data))

except Exception as e:
    print("Error:", e)

finally:
    uos.umount("/sd")
    cs_pin.value(1)  # Deselect the SD card
    print("SD card unmounted")

print("TEST DONE")
