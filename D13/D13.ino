#include <WiFi.h>
#include <WiFiClient.h>
#include <ESP32Time.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <esp_task_wdt.h>
#include <driver/adc.h>
#include <soc/rtc_io_reg.h>
#include <soc/rtc_io_channel.h>
#include "soc/rtc_cntl_reg.h"
#include <HAMQDisco.h>
#include <Watering.h>
#include "Settings.h"
#include "ConcreteSensor.h"
#include "ConcretePump.h"
#include "../include/Prefs.h"
#include "../include/Camera.h"
#include "../include/ULPpwm.h"
#include "../include/Utils.h"

#define VAR_NAME(variable) (#variable)

#ifdef CONFIG_IDF_TARGET_ESP32
#define ADC_WIDTH   ADC_WIDTH_BIT_12
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define ADC_WIDTH   ADC_WIDTH_BIT_13   //ESP32S2 only support 13 bit width
#endif

#define ADC_ATTEN   ADC_ATTEN_DB_11

//ESP state and control
DiscoverySensor sensorState(device_name, "state", "state", {{"exp_aft", "3600"}});
DiscoverySensor sensorIP(device_name, "ip", "IP address", {{"exp_aft", "3600"}});
DiscoverySensor wifiSignalStrength(device_name, "rssi", "WiFi signal strength", {{"unit_of_meas", "dBm"}, {"dev_cla", "signal_strength"}, {"exp_aft", "3600"}});
DiscoverySensor sensorResetReason(device_name, "rr", "reset reason", {{"exp_aft", "3600"}}); //1 for first power on, 8 for wake up from deep sleep, 3 for software restart, all others mean something is wrong
DiscoverySensor sensorError(device_name, "error", "error", {{"exp_aft", "3600"}});
DiscoveryButton buttonRestart(device_name, "restart", "restart", {{"dev_cla", "restart"}, {"ent_cat", "config"}});
DiscoveryButton buttonOTA(device_name, "ota", "OTA", {{"dev_cla", "update"}, {"ent_cat", "config"}});
//camera
DiscoveryButton buttonPicture(device_name, "pic", "take picture", {{"icon", "mdi:camera"}});
DiscoveryButton buttonVideo(device_name, "video", "video", {{"icon", "mdi:video"}});
DiscoverySensor sensorLastPicture(device_name, "lastpic", "last picture");
DiscoveryButton buttonCameraPowerCycle(device_name, "powcycl", "camera power cycle", {{"icon", "mdi:power-cycle"}});
//built-in white LED
DiscoveryLight light0(device_name, "light0", "built-in LED", {"brightness"});
//watering
DiscoveryNumber numbersThreshold(device_name, "threshold", "threshold (%)", {{"mode", "slider"}, {"min", "50"}, {"max", "95"}, {"icon", "mdi:waves-arrow-up"}});
DiscoveryNumber numbersPumpTime(device_name, "pumptime", "duration (s)", {{"mode", "slider"}, {"min", "5"}, {"max", "180"}, {"step", "5"}, {"icon", "mdi:timer-settings-outline"}});
DiscoveryNumber numbersInterval(device_name, "interval", "interval (days)", {{"mode", "slider"}, {"min", "1"}, {"max", "7"}, {"icon", "mdi:timer-settings-outline"}});
DiscoveryButton buttonWater(device_name, "water", "watering", {{"icon", "mdi:watering-can"}});
std::vector<std::string> options = {"Automatic", "Manual"};
DiscoverySelect selectMode(device_name, "mode", "watering mode", options);
DiscoverySensor sensorMoisture(device_name, "moisture", "soil moisture", {{"exp_aft", "3700"}, {"unit_of_meas", "%"}});
DiscoveryBinarySensor binarySensorPump(device_name, "pumpstate", "pump state", {{"off_delay", "1"}, {"icon", "mdi:water-pump"}});

gpio_num_t PWR_PIN = GPIO_NUM_0;

adc_atten_t ConcreteSensor::atten = ADC_ATTEN;
adc_bits_width_t ConcreteSensor::width = ADC_WIDTH;
int32_t WateringSet::start_hour = irrigation_start_hour;
int32_t WateringSet::end_hour = irrigation_end_hour;
std::vector<WateringSet> sets;
RTC_DATA_ATTR bool allowSensor;

//set 1
RTC_DATA_ATTR WateringData data1;
ConcreteSensor sensorSet1({{GPIO_NUM_12, ADC2_CHANNEL_5, 2750, 900}}); //ADC channels https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/adc.html
gpio_num_t PUMP1_PIN = GPIO_NUM_2;
ConcretePump pump1(PUMP1_PIN);
WateringSet wateringSet1(data1, &pump1, &sensorSet1);

//set 2
RTC_DATA_ATTR WateringData data2;
ConcreteSensor sensorSet2({{GPIO_NUM_14, ADC2_CHANNEL_6, 2750, 990}});
gpio_num_t PUMP2_PIN = GPIO_NUM_15;
ConcretePump pump2(PUMP2_PIN);
WateringSet wateringSet2(data2, &pump2, &sensorSet2);

//red led will indicate that device is awake
gpio_num_t LED_PIN = GPIO_NUM_33;

//onboard white led
gpio_num_t LGT_PIN = GPIO_NUM_4;
int32_t PWM_BIT = RTCIO_GPIO4_CHANNEL + 14;
RTC_NOINIT_ATTR bool light0St; //current state of light
RTC_NOINIT_ATTR bool light0En; //enabled / disabled
RTC_NOINIT_ATTR int32_t light0Br; // brightness

//camera
RTC_NOINIT_ATTR bool pic;
bool video = false; //won't be preserved after wake up if camera did not initialize
bool cam_res = false;
bool powerCycle = false;

//NTP & time
const char* ntpServer = "europe.pool.ntp.org";
const int32_t  gmtOffset_sec = 3600 * offset_hours;
RTC_NOINIT_ATTR int32_t prevHour; // used to request time sync
ESP32Time rtc(gmtOffset_sec);  //offset in seconds
const uint32_t uS_TO_S_FACTOR = 1000000ULL;  //Conversion factor for microseconds to seconds

//FTP
ESP32_FTPClient ftp(server, ftp_user, ftp_pass, 40000, 2);

bool restart = false;
bool ota = false;
bool mqttDiscovery = false;
uint32_t startTime = 0;
esp_reset_reason_t r;
std::string error = "";

WiFiClient espClient;
PubSubClient client(espClient);
WebServer httpServer(80);
HTTPUpdateServer httpUpdater;

Preferences preferences;
const char* prefName = "prefs";

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  //enable WDT
  esp_task_wdt_config_t config = {
    .timeout_ms = 60 * 1000,
    .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&config);
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  pinConfig();

  esp_sleep_enable_timer_wakeup(sleep_time * uS_TO_S_FACTOR);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  sets.push_back(wateringSet1);
  sets.push_back(wateringSet2);

  //https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/misc_system_api.html#_CPPv418esp_reset_reason_t
  r = esp_reset_reason();
  log_i("esp_reset_reason: %d", r);
  if (r != ESP_RST_DEEPSLEEP) {
    time_t epoch_ts = 30000;
    struct timeval tv_ts = {.tv_sec = epoch_ts};
    settimeofday(&tv_ts, NULL);

    prevHour = -1;
    pic = false;
    video = false;

    log_i("Free entries: %d", preferences.freeEntries());
    readPrefs(&preferences, prefName, light0En, VAR_NAME(light0En), false);
    readPrefs(&preferences, prefName, light0Br, VAR_NAME(light0Br), 1);
    light0St = false;

    allowSensor = true;
    for (WateringSet& i : sets) {
      readPrefs(&preferences, prefName, i.threshold(), WateringSet::thresholdName, 75);
      readPrefs(&preferences, prefName, i.pumpTime(), WateringSet::pumpTimeName, 15);
      readPrefs(&preferences, prefName, i.interval(), WateringSet::intervalName, 2);
      readPrefs(&preferences, prefName, i.mode(), WateringSet::modeName, 0);
      i.initWateringData();
    }
  }

  for (const WateringSet& i : sets) {
    log_i("threshold %d: %d", i.getNumber(), i.threshold());
    log_i("pumptime %d: %d", i.getNumber(), i.pumpTime());
    log_i("interval %d: %d", i.getNumber(), i.interval());
    log_i("mode %d: %s", i.getNumber(), options[i.mode()].c_str());
  }

  if (allowSensor && r != ESP_RST_POWERON) {
    gpio_reset_pin(PWR_PIN);
    gpio_set_direction(PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PWR_PIN, 0);

    std::vector<bool> results;
    for (WateringSet& i : sets) {
      results.push_back(i.check(rtc.getEpoch()));
    }

    gpio_set_level(PWR_PIN, 1);
    gpio_reset_pin(PWR_PIN);
    allowSensor = false;

    if (std::any_of(results.begin(), results.end(), [](bool i){ return i; })) {
      allowSensor = true;
    }
  }
  
  config = {
    .timeout_ms = 240 * 1000,
    .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&config);

  bool wifiConnected = connectToWifi();

  if (wifiConnected) {
    if (setTime(prevHour, rtc, ntpServer, mqttDiscovery)) {
      allowSensor = true;
    }

    connectToMQTT();
    
    if (client.connected()) {
      checkMQTT(client);

      client.publish(sensorIP.getStateTopic(), WiFi.localIP().toString().c_str());
      client.publish(sensorResetReason.getStateTopic(), std::to_string(r).c_str());
      client.publish(wifiSignalStrength.getStateTopic(), std::to_string(WiFi.RSSI()).c_str());
      client.publish(light0.getBrightnessStateTopic(), std::to_string(light0Br).c_str());
      client.publish(light0.getStateTopic(), light0En ? "ON" : "OFF");

      for (WateringSet& i : sets) {
        client.publish(numbersThreshold.getStateTopic(i.getNumber()), std::to_string(i.threshold()).c_str());
        client.publish(numbersPumpTime.getStateTopic(i.getNumber()), std::to_string(i.pumpTime()).c_str());
        client.publish(numbersInterval.getStateTopic(i.getNumber()), std::to_string(i.interval()).c_str());
        client.publish(selectMode.getStateTopic(i.getNumber()), options[i.mode()].c_str());
        
        if (!i.getSensorsReported()) {
          std::vector<int32_t> s = i.getSensorValues();
          if (!s.empty()) {
            if (s.size() > 1) {
              for (int32_t j = 0; j < s.size(); j++) {
                client.publish(sensorMoisture.getStateTopic(i.getNumber(), j + 1), std::to_string(s[j]).c_str());
              }
            } else {
              client.publish(sensorMoisture.getStateTopic(i.getNumber()), std::to_string(s[0]).c_str());
            }
          }

          i.setSensorsReported();
        }

        if (!i.getPumpReported()) {
          client.publish(binarySensorPump.getStateTopic(i.getNumber()), "ON");
          i.setPumpReported();
          log_i("setPumpReported %d", i.getNumber());
        } else {
          client.publish(binarySensorPump.getStateTopic(i.getNumber()), "OFF");
        }
      }

      checkMQTT(client);
    }

    if (video || pic) {
      cam_res = configCamera(video);
  
      if (cam_res) {
        if (!video) {
          camera_fb_t* fb = NULL;
          getPicture(fb);
    
          if (!fb) {
            error += "Camera capture failed. ";
            log_e("Error: %s", error);
          } else {
            esp_task_wdt_reset();
            const char* filename = rtc.getTime("%Y%m%d_%H%M%S.jpg").c_str();
            sendDataToFTP(ftp, (unsigned char*)fb->buf, fb->len, NULL, filename);
            pic = false;
          }

          esp_camera_fb_return(fb);

          if (client.connected()) {
            if (!pic) {
              client.publish(sensorLastPicture.getStateTopic(), (rtc.getTime("%Y.%m.%d %H.%M")).c_str());
            }
          }
        } else {
          startCameraServer();
        }
      } else {
        error += "cam_res != ESP_OK. ";
        log_e("Error: %s", error);
      }
    }

    if (!video && ota) {
      httpUpdater.setup(&httpServer, "/update", ota_user, ota_pass);
      httpServer.begin();
    }
  }

  log_i("wifiConnected: %d", wifiConnected);
  log_i("mqttConnected: %d", client.connected());
  log_i("pic: %d", pic);
  log_i("video: %d", video);

  ulp_light(LGT_PIN, PWM_BIT, light0Br, light0En, light0St);

  if (powerCycle) {
    gpio_reset_pin(GPIO_NUM_32);
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_32, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_NUM_32, 0);
    gpio_reset_pin(GPIO_NUM_32);
  }
    
  if (restart) {
    log_i("Restart");
    esp_restart();
  } else if (!video && !ota) {
    sleep();
  }
 
  config = {
    .timeout_ms = wdt_timeout * 1000,
    .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&config);

  startTime = millis();
  log_i("End of setup()");
}

void loop() {
  if (ota) {
    httpServer.handleClient();
    if (startTime + ota_time * 1000 < millis()) {
      sleep();
    }
  } else if (!ota && video) {
    if (startTime + video_time * 1000 < millis()) {
      sleep();
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    sleep();
  }
}

void sleep() {
  if (client.connected()) {
    client.publish(sensorError.getStateTopic(), error.empty() ? "OK" : error.c_str());
    client.publish(sensorState.getStateTopic(), "0");
    checkMQTT(client);
  }

  WiFi.disconnect();
    
  log_i("Going to sleep");
  esp_deep_sleep_start();
}

void pinConfig() {
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_PIN, 0); //inverted

  gpio_reset_pin(PUMP1_PIN);
  gpio_set_direction(PUMP1_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(PUMP1_PIN, 0);

  gpio_reset_pin(PUMP2_PIN);
  gpio_set_direction(PUMP2_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(PUMP2_PIN, 0);
}

void connectToMQTT() {
  client.setBufferSize(512);
  client.setServer(server, 1883);
  client.setCallback(callback);
  client.setKeepAlive(1200);

  for (int32_t i = 0; i < 5; i++) {
    log_i("Connecting to MQTT, attempt %d", i);
    client.connect(device_name.c_str(), mqtt_user, mqtt_pass, sensorState.getStateTopic(), 1, true, "0", false);
    if (client.connected()) {
      if (mqttDiscovery) {
        publishDiscoveryMessage(client, sensorState);
        publishDiscoveryMessage(client, sensorIP);
        publishDiscoveryMessage(client, sensorResetReason);
        publishDiscoveryMessage(client, wifiSignalStrength);
        publishDiscoveryMessage(client, sensorError);
        publishDiscoveryMessage(client, buttonRestart);
        publishDiscoveryMessage(client, buttonOTA);
        publishDiscoveryMessage(client, buttonPicture);
        publishDiscoveryMessage(client, buttonVideo);
        publishDiscoveryMessage(client, sensorLastPicture);
        publishDiscoveryMessage(client, buttonCameraPowerCycle);
        publishDiscoveryMessage(client, light0);

        for (const WateringSet& i : sets) {
          publishDiscoveryMessage(client, numbersThreshold, i.getNumber());
          publishDiscoveryMessage(client, numbersPumpTime, i.getNumber());
          publishDiscoveryMessage(client, numbersInterval, i.getNumber());
          publishDiscoveryMessage(client, buttonWater, i.getNumber());
          publishDiscoveryMessage(client, selectMode, i.getNumber());
          publishDiscoveryMessage(client, binarySensorPump, i.getNumber());

          int32_t count = i.getSensorCount();
          if (count > 1) {
            for (int32_t j = 0; j < count; j++) {
              publishDiscoveryMessage(client, sensorMoisture, i.getNumber(), j + 1);
            }
          } else {
            publishDiscoveryMessage(client, sensorMoisture, i.getNumber());
          }
        }
      }
      
      client.subscribe(buttonRestart.getCommandTopic(), true);
      client.subscribe(buttonOTA.getCommandTopic(), true);
      client.subscribe(buttonPicture.getCommandTopic(), true);
      client.subscribe(buttonVideo.getCommandTopic(), true);
      client.subscribe(buttonCameraPowerCycle.getCommandTopic(), true);
      client.subscribe(light0.getCommandTopic());
      client.subscribe(light0.getBrightnessCommandTopic());
      
      for (const WateringSet& i : sets) {
        client.subscribe(numbersThreshold.getCommandTopic(i.getNumber()));
        client.subscribe(numbersPumpTime.getCommandTopic(i.getNumber()));
        client.subscribe(numbersInterval.getCommandTopic(i.getNumber()));
        client.subscribe(buttonWater.getCommandTopic(i.getNumber()), true);
        client.subscribe(selectMode.getCommandTopic(i.getNumber()));
      }

      client.publish(sensorState.getStateTopic(), "1");

      log_i("Connected to MQTT");
      break;
    }

    log_i("Failed to connected to MQTT: %d", client.state());
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void callback(char* topic, byte* payload, uint32_t length) {
  std::string message((char*)payload, length);
  std::string setTopic(topic);

  log_i("----------------------------");
  log_i("Message: %s", message.c_str());
  log_i("Topic: %s", topic);

  if (setTopic == buttonRestart.getCommandTopic()) {
    restart = true;

  } else if (setTopic == buttonOTA.getCommandTopic()) {
    ota = true;

  } else if (setTopic == buttonPicture.getCommandTopic()) {
    pic = true;

  } else if (setTopic == buttonVideo.getCommandTopic()) {
    video = true;

  } else if (setTopic == buttonCameraPowerCycle.getCommandTopic()) {
    powerCycle = true;
  
  } else if (setTopic == light0.getBrightnessCommandTopic()) {
    int32_t command = std::stoi(message);
    writePrefs(&preferences, prefName, light0Br, VAR_NAME(light0Br), command);

    if (light0En) {
      ulp_light(LGT_PIN, PWM_BIT, light0Br, false, light0St);
      vTaskDelay(10 / portTICK_PERIOD_MS);
      ulp_light(LGT_PIN, PWM_BIT, light0Br, true, light0St);
    }

  } else if (setTopic == light0.getCommandTopic()) {
    bool command = message.find("ON") != std::string::npos ? true : false;
    writePrefs(&preferences, prefName, light0En, VAR_NAME(light0En), command);

  } else if (setTopic.find(numbersThreshold.getShortName()) != std::string::npos) {
    int32_t command = std::stoi(message);
    int32_t index = getIndex(setTopic);
    if (0 <= index) {
      writePrefs(&preferences, prefName, sets[index].threshold(), WateringSet::thresholdName, command);
    }

  } else if (setTopic.find(numbersPumpTime.getShortName()) != std::string::npos) {
    int32_t command = std::stoi(message);
    int32_t index = getIndex(setTopic);
    if (0 <= index) {
      writePrefs(&preferences, prefName, sets[index].pumpTime(), WateringSet::pumpTimeName, command);
    }

  } else if (setTopic.find(numbersInterval.getShortName()) != std::string::npos) {
    int32_t command = std::stoi(message);
    int32_t index = getIndex(setTopic);
    if (0 <= index) {
      writePrefs(&preferences, prefName, sets[index].interval(), WateringSet::intervalName, command);
    }

  } else if (setTopic.find(selectMode.getShortName()) != std::string::npos) {
    int32_t command = getVectorIndex(options, message);
    int32_t index = getIndex(setTopic);
    if (0 <= index && 0 <= command) {
      writePrefs(&preferences, prefName, sets[index].mode(), WateringSet::modeName, command);
    }

  } else if (setTopic.find(buttonWater.getShortName()) != std::string::npos) {
    int32_t index = getIndex(setTopic);
    if (0 <= index && sets[index].mode() != 0) {
      allowSensor = sets[index].tryManualWatering();
    }
  }
}
