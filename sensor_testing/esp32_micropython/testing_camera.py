from camera import Camera, GrabMode, PixelFormat, FrameSize, GainCeiling

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

#Camera construction using defaults (if you specified them in mpconfigboard.h)
#camera = Camera(data_pins)

# Capture image
img = camera.capture()

# Camera reconfiguration 
camera.reconfigure(pixel_format=PixelFormat.JPEG,frame_size=FrameSize.QVGA,grab_mode=GrabMode.LATEST, fb_count=2)
camera.set_quality(10)