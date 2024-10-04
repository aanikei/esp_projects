# ESP projects

This repository contains my hobby projects, created mostly using ESP32-CAM or [ESP32-S2 Mini](https://www.wemos.cc/en/latest/s2/s2_mini.html) boards and Arduino IDE.

The mode of operation for all non-ESPHome projects involves using deep sleep: the device wakes up, performs some operations, and goes back to sleep. For some projects that use sensors, external wake-up can be enabled.

All non-ESPHome projects use my library [HAMQDisco](https://github.com/aanikei/HAMQDisco) as they are integrated with Home Assistant (HA) through MQTT. Note that since the devices use sleep mode, they check commands from HA only after waking up and may execute them only on the next wake-up.

All of the projects were assembled on prototyping boards using components I had on hand and are powered by 5V phone chargers. Note that depending on the charger and cable length, the brownout detector may need to be disabled.

There are several files that are shared across various projects. They are placed in the `include` directory in the root Arduino folder:
- `Camera.h` contains functions for working with the camera.
- `Prefs.h` contains functions to facilitate reading and saving preferences.
- `ULPpwm.h` contains functions to control light using PWM and the Ultra Low Power coprocessor (ULP) (only one light can be controlled with the current implementation).
- `Utils.h` contains various helper functions for working with FTP and MQTT.

## Camera

ESP32-CAM projects allow taking images that are then uploaded to an FTP server and displayed in HA. Video can be viewed (using the IP address of the device) after the corresponding button is pressed and the device is awakened.

> [!NOTE]
> ESP32-CAMs need a small modification. Due to the board design, the video frame rate is very low at high resolution, and GPIO0 cannot be used for purposes other than the camera if the camera is needed. Placing copper tape as shown in the picture below and soldering it to GND pin resolves this issue. However, the camera modules themselves may not be very stable and may fail to initialize. A device power cycle may or may not resolve this issue for a particular module. It seems their performance may degrade over time or depend on the light level.

> ![Image of ESP32-CAM modification.](/ESP32_CAM_mod.jpg)

## Built-in white LED

ESP32-CAM boards contain a white LED that can be used as a flash for the camera; however, in my projects, it is used as a light. The ULP is used to control its state and brightness, so it works even in deep sleep. Note that at full brightness, the LED gets hot.

## Settings.h

Every non-ESPHome project has a `Settings.h` file where you should specify Wi-Fi credentials, the IP address of the MQTT and FTP server, and the users and passwords for MQTT, FTP, and OTA. The rest of the variables in this file are self-explanatory.

## Sketch upload

Next settings are used in Arduino IDE:
- CPU Frequency: 160 MHz (I found that Wi-Fi connection success was more likely after deep sleep with this frequency).
- Core Debug Level: None (TX (GPIO1) and RX (GPIO3) pins may be used in a project. You will need to comment out any code that may use them before changing the debug level to debug the code).
- Partition Scheme: Minimal SPIFFS.
- PSRAM: Enabled.
The rest of the settings can be left as is.

## Watering

The main use case for the majority of the projects is to water indoor plants. For that, my library [Watering](https://github.com/aanikei/Watering) is used.

Capacitive soil moisture sensors monitor moisture levels and watering is triggered when the level falls below a set threshold. You may want to check this [video](https://www.youtube.com/watch?v=IGP38bz-K48) to ensure that your sensors are working properly. As these sensors are also affected by corrosion over long periods of time, I only power them once an hour. In ESP32-CAMs, GPIO0 is used for that purpose. Since this pin is pulled up on the board, an inverting circuit is needed. I use [these](https://electronics.stackexchange.com/questions/641558/is-it-possible-to-make-a-high-side-pnp-switch-circuit-active-low-with-less-than) for my projects.

As for the pumps, diaphragm water pumps like the R385 can be used, or smaller ones. If higher voltage is needed for the R385, I recommend using a step-up DC-DC converter like the one in the image below.
> ![Converter.](/ESP32_CAM_mod.jpg)

As one pump may be used to water more than one plant, moisture sensors can be placed in different pots, and the water pressure can be regulated using airline valves for fish tanks.

In ESP32-CAMs, only GPIO12 and GPIO14 can be used with capacitive soil moisture sensors as is, since the rest of the ADC-capable pins are pulled up. An op-amp can be used to overcome this limitation. Note that GPIO12 is a strapping pin, and boot will fail if it is pulled high.

While listing my projects below, I will indicate the specific configuration for each project, such as 1/1, 1/1 (meaning 2 sets of one pump and one soil moisture sensor) or 1/2 (meaning 1 set of one pump and two soil moisture sensors).

Note that the `Watering` library contains abstract pump and sensor classes that have basic implementations in the ConcretePump and ConcreteSensor classes. If you need a different implementation, you will need to create it yourself and place the files in the project’s directory or in the `src` directory of the `Watering` library.

### The list of projects:

...

[D13](/D13): ESP32-CAM, watering (1/1, 1/1).

...