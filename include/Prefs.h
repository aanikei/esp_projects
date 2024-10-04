#pragma once
#include <Preferences.h>
#include <esp32-hal-log.h>

void readPrefs(Preferences* prefs, const char* name, bool& out, const char* key, bool initValue) {
  bool res = prefs->begin(name, true); //open in Read-Only mode

  if (prefs->isKey(key)) {
    out = prefs->getBool(key);
    log_i("Key [%s] exists: %d", key, out);
  } else {
    prefs->end();
    bool res = prefs->begin(name, false); //open in RW mode
    log_i("Open storage result, %d", res);
    size_t putRes = prefs->putBool(key, initValue);
    out = initValue;
    log_i("Key [%s] does not exist, out %d, putRes %d", key, out, putRes);
  }
  prefs->end();
}

void readPrefs(Preferences* prefs, const char* name, int32_t& out, const char* key, int32_t initValue) {
  bool res = prefs->begin(name, true); //open in Read-Only mode
  
  if (prefs->isKey(key)) {
    out = prefs->getInt(key);
    log_i("Key [%s] exists: %d", key, out);
  } else {
    prefs->end();
    bool res = prefs->begin(name, false); //open in RW mode
    log_i("Open storage result, %d", res);
    size_t putRes = prefs->putInt(key, initValue);
    out = initValue;
    log_i("Key [%s] does not exist, out %d, putRes %d", key, out, putRes);
  }
  prefs->end();
}

void writePrefs(Preferences* prefs, const char* name, bool& out, const char* key, bool value) {
  if (out != value) {
    bool res = prefs->begin(name, false); //open in RW mode
    log_i("Open storage result, %d", res);
    size_t putRes = prefs->putBool(key, value);
    log_i("putBool result, %d", putRes);
    prefs->end();
    out = value;
    log_i("writePrefs, %s: %d", key, out);
  }
}

void writePrefs(Preferences* prefs, const char* name, int32_t& out, const char* key, int32_t value) {
  if (out != value) {
    bool res = prefs->begin(name, false); //open in RW mode
    log_i("Open storage result, %d", res);
    size_t putRes = prefs->putInt(key, value);
    log_i("putInt result, %d", putRes);
    prefs->end();
    out = value;
    log_i("writePrefs, %s: %d", key, out);
  }
}