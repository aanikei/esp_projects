#pragma once
#include <regex>
#include <ESP32_FTPClient.h>
#include <ESP32Time.h>

bool connectToWifi() {
  log_i("Attempting Wifi connection");
  WiFi.begin(ssid, password);
  int32_t wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    log_i(".");
    wifiAttempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setAutoReconnect(true);
    log_i("ip: %s", WiFi.localIP().toString());
    return true;
  } else {
    WiFi.disconnect(true, true);
    return false;
  }
}

bool setTime(int32_t& prevHour, ESP32Time& rtc, const char* ntpServer, bool& mqttDiscovery) {
  if (prevHour != rtc.getHour(true)) {
      prevHour = rtc.getHour(true);
      configTime(0, 0, ntpServer);
      struct tm timeinfo;
      if (getLocalTime(&timeinfo)) {
        rtc.setTimeStruct(timeinfo);
      }
    mqttDiscovery = true;
	return true;
  }
  return false;
}

void sendDataToFTP(ESP32_FTPClient& ftp, unsigned char* in, int size, const char* dir, const char* filename) {
  log_i("Opening connection to FTP");
  ftp.OpenConnection(); // try open FTP
  if (ftp.isConnected()) {
    if (dir != NULL) {
      ftp.ChangeWorkDir(dir);
    }
    ftp.InitFile("Type I");
    ftp.NewFile(filename);
    ftp.WriteData(in, size);
    ftp.CloseFile();
    ftp.CloseConnection();
  }
}

void checkMQTT(PubSubClient& client) {
  log_i("checkMQTT");
  unsigned long start = millis();
  while (start + 2000 > millis()) {
    client.loop();
  }
}

void publishDiscoveryMessage(PubSubClient& client, DiscoveryBase& dmb) {
  client.publish(dmb.getConfigurationTopic(), dmb.getDiscoveryMessage(), true);
}

void publishDiscoveryMessage(PubSubClient& client, DiscoveryBase& dmb, int32_t i) {
  client.publish(dmb.getConfigurationTopic(i), dmb.getDiscoveryMessage(i), true);
}

void publishDiscoveryMessage(PubSubClient& client, DiscoveryGenericSensor& dm, int32_t i, int32_t j) {
  client.publish(dm.getConfigurationTopic(i, j), dm.getDiscoveryMessage(i, j), true);
}

int32_t getIndex(std::string setTopic) {
  std::regex r(R"((?:\D*\d+\D+)(\d+))"); //may need to be adjusted based on your device's name
  std::smatch match;
  int32_t index = 0;

  if (std::regex_search(setTopic, match, r)) {
    index = std::stoi(match[1]);
  }

  return index - 1;
}

int32_t getVectorIndex(std::vector<std::string> v, std::string value) {
  ptrdiff_t pos = find(v.begin(), v.end(), value) - v.begin();
  if (pos >= v.size()) {
    return -1;
  } else {
    return (int32_t)pos;
  }
}
