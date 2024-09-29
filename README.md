# ESP projects

This repository contains my hobby projects, created mostly using ESP32-CAM or [ESP32-S2 Mini](https://www.wemos.cc/en/latest/s2/s2_mini.html) boards and Arduino IDE.

The mode of operation for all non-ESPHome projects involves using deep sleep: the device wakes up, performs some operations, and goes back to sleep. For some projects that use sensors, external wake-up can be enabled.

All non-ESPHome projects use my library [HAMQDisco](https://github.com/aanikei/HAMQDisco) as they are integrated with Home Assistant (HA) through MQTT. Note that since the devices use sleep mode, they check commands from HA only after waking up and may execute them only on the next wake-up.

ESP32-CAM projects allow taking images that are then uploaded to an FTP server and displayed in HA. Video can be viewed (using the IP address of the device) after the corresponding button is pressed and the device is awakened.

All of the projects were assembled on prototyping boards using components I had on hand and are powered by 5V phone chargers. Note that depending on the charger and cable length, the brownout detector may need to be disabled.

> [!NOTE]
> ESP32-CAMs need a small modification. Due to the board design, the video frame rate is very low at high resolution, and GPIO0 cannot be used for purposes other than the camera if the camera is needed. Placing copper tape as shown in the picture below and soldering it to GND pin resolves this issue. However, the camera modules themselves may not be very stable and may fail to initialize. A complete power-off may or may not resolve this issue for a particular module. It seems their performance may degrade over time or depend on the light level.

> ![Image of ESP32-CAM modification.](/ESP32_CAM_mod.jpg)

Here is the list of projects:

...

D13: ESP32-CAM, watering indoor plants (1/1, 1/1) (means 2 sets of one pump and one soil moisture sensor).

...