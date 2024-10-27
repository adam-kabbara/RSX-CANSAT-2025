# Micropython Camera on ESP32 S3 WROOM
## Downloading firmware 
1. downlaod the micro python .bin file from https://micropython.org/download/ESP32_GENERIC_S3/ this link is specific to our esp32 type (Firmware (Support for Octal-SPIRAM)).
2. to get the camera module, you either install the driver for the camera using thoney onto the esp32 or use the firmware provided by the git repo that has the drivers pre installed in the firmware. https://github.com/cnadler86/micropython-camera-API/releases (this is the method we chose in this documentation)

## Downloading `esptool`
1. Just `pip install esptool`. Gonna use this tool to flash firmware onto the esp32

## Flashing
1. Ensure that your ESP32 is in flash mode while you’re trying to upload the firmware. To enter flash mode, hold down the `BOOT` button while resetting the board or powering it on.
2. After you have pluged in, on windows, to check what port the esp32 is using go to `Device Manager` and look under the `Ports` drop down menue. In the commands below, `COM6` was the port used.
3. Look for where `esptools` was installed and `cd` there. For my case I installed it in a conda env and it was found in `C:\Users\adamk\anaconda3\envs\cansat\Scripts>`. The commads below use `esptool.exe` but for you it might be `esptool.py`
4. Remove the old fiirmware by running
```
esptool.exe --port COM6 erase_flash
```
5. Install the new firmware by running
```
esptool.exe --chip esp32s3 --baud 115200 --port COM6 write_flash -z 0 "dir_to_firmmware_.bin_file"
```
Now this should take a min or two and you should have the miropython firmware on the esp32.

## Programming
1. Download Thonny https://thonny.org/
2. Set it up by going into `Tools > Options > Interpreter` Then select `Micropython (ESP32)` and select what port it is using in my case `COM6`
3. Test if everything is working by running the [led_blinkning](testing_camera.py)
4. check out the datasheet and change the vars in the testing_camera.py file 