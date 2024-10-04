const std::string device_name = "D13"; //this is also FTP user

int32_t sleep_time = 120; //seconds
int32_t offset_hours = 0; //for NTP

char* ssid = "YourWiFi"; //WiFi ssid
char* password = "YourWiFiPassword"; //WiFi password

char* server = "192.168.1.2"; //mqtt & ftp broker IP

char* mqtt_user = "mqtt_user";
char* mqtt_pass = "mqtt_pass";

char* ftp_user = strdup(device_name.c_str());
char* ftp_pass = "ftp_pass";

char* ota_user = "ota_user";
char* ota_pass = "ota_pass";

int32_t wdt_timeout = 360; // Seconds for loop() to be reset just in case; should be greater than any value below
int32_t ota_time = 340;    // Seconds to wait after OTA is initiated, before going to sleep
int32_t video_time = 180;  // Seconds to stream video before going to sleep

int32_t irrigation_start_hour = 10;
int32_t irrigation_end_hour = 15;
