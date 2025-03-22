# platformio project info

- using the official `platformio/platform-espressif32` platform doesn't provide the necessary libraries so we have to use `pioarduino/platform-espressif32`
- the board config is a custom board/json for the ESP32S3 Dev Board with 16MB flash and 8MB psram.
- UPDATE: core v3.1.0 seems to have broken the frame buffer when trying to get images, resulting in it being streched vertically with horizontal black lines.
    - Reverted to release #187e1cbaed5888158d65c5dc24100c6d3cc65cb2 (v3.0.4) since thats the last version I checked that it worked on. Newer versons before v3.1.0 might also work.
    