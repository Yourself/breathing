/*
Important: This code is only for the DIY PRO PCB Version 3.7 that has a push
button mounted.

This is the code for the AirGradient DIY PRO Air Quality Sensor with an ESP8266
Microcontroller with the SGP40 TVOC module from AirGradient.

It is a high quality sensor showing PM2.5, CO2, Temperature and Humidity on a
small display and can send data over Wifi.

Build Instructions:
https://www.airgradient.com/open-airgradient/instructions/diy-pro-v37/

Kits (including a pre-soldered version) are available:
https://www.airgradient.com/open-airgradient/kits/

The codes needs the following libraries installed:
“WifiManager by tzapu, tablatronix” tested with version 2.0.11-beta
“U8g2” by oliver tested with version 2.32.15
"Sensirion I2C SGP41" by Sensation Version 0.1.0
"Sensirion Gas Index Algorithm" by Sensation Version 3.2.1

Configuration:
Please set in the code below the configuration parameters.

If you have any questions please visit our forum at
https://forum.airgradient.com/

If you are a school or university contact us for a free trial on the AirGradient
platform. https://www.airgradient.com/

CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License

*/
#include <EEPROM.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>

#include "AirGradient.h"

// #include "SGP30.h"
#include <NOxGasIndexAlgorithm.h>
#include <SensirionI2CSgp41.h>
#include <U8g2lib.h>
#include <VOCGasIndexAlgorithm.h>

#include <cstdio>

AirGradient ag = AirGradient(false, 115200);
SensirionI2CSgp41 sgp41;
VOCGasIndexAlgorithm voc_algorithm;
NOxGasIndexAlgorithm nox_algorithm;
// time in seconds needed for NOx conditioning
uint16_t conditioning_s = 10;

// for peristent saving and loading
int addr = 0;
byte value;

// Display bottom right
// U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Replace above if you have display on top left
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// CONFIGURATION START

// set to the endpoint you would like to use
char API_ROOT[] = "http://192.168.0.16:3000/api/restricted/submit/";
char API_ENDPOINT[128];

// set to true to switch from Celcius to Fahrenheit
boolean inF = false;

// PM2.5 in US AQI (default ug/m3)
boolean inUSAQI = false;

// Display Position
boolean displayTop = true;

// set to true if you want to connect to wifi. You have 60 seconds to connect.
// Then it will go into an offline mode.
boolean connectWIFI = true;

// CONFIGURATION END

unsigned long currentMillis = 0;

const int oledInterval = 5000;
unsigned long previousOled = 0;

const int sendToServerInterval = 10000;
unsigned long previousSendToServer = 0;

const int tvocInterval = 5000;
unsigned long previousTVOC = 0;
int TVOC = 0;
int NOX = 0;

const int co2Interval = 5000;
unsigned long previousCo2 = 0;
int co2 = 0;

const int pm25Interval = 5000;
unsigned long previousPm25 = 0;
int pm25 = 0;

const int tempHumInterval = 2500;
unsigned long previousTempHum = 0;
float temp = 0;
float hum = 0;

const int minInterval = std::min({oledInterval, sendToServerInterval, tvocInterval, co2Interval, pm25Interval, tempHumInterval});

std::uint8_t buttonConfig = 0;
int lastState = LOW;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

// Calculate PM2.5 US AQI
int pmToAqi(int pm02) {
  if (pm02 <= 12.0)
    return static_cast<int>((50 - 0) / (12.0 - .0) * (pm02 - .0) + 0);
  else if (pm02 <= 35.4)
    return static_cast<int>((100 - 50) / (35.4 - 12.0) * (pm02 - 12.0) + 50);
  else if (pm02 <= 55.4)
    return static_cast<int>((150 - 100) / (55.4 - 35.4) * (pm02 - 35.4) + 100);
  else if (pm02 <= 150.4)
    return static_cast<int>((200 - 150) / (150.4 - 55.4) * (pm02 - 55.4) + 150);
  else if (pm02 <= 250.4)
    return static_cast<int>((300 - 200) / (250.4 - 150.4) * (pm02 - 150.4) +
                            200);
  else if (pm02 <= 350.4)
    return static_cast<int>((400 - 300) / (350.4 - 250.4) * (pm02 - 250.4) +
                            300);
  else if (pm02 <= 500.4)
    return static_cast<int>((500 - 400) / (500.4 - 350.4) * (pm02 - 350.4) +
                            400);
  else
    return 500;
};

int tempToF(float tempC) {
  return static_cast<int>(round(tempC * 9. / 5.)) + 32;
}

void updateTVOC() {
  uint16_t srawVoc = 0;
  uint16_t srawNox = 0;

  uint16_t compensationT = static_cast<uint16_t>((temp + 45) * 65535 / 175);
  uint16_t compensationRh = static_cast<uint16_t>(hum * 65535 / 100);

  if (conditioning_s > 0) {
    if (sgp41.executeConditioning(compensationRh, compensationT, srawVoc)) {
      TVOC = -1;
      NOX = -1;
      return;
    }
    conditioning_s--;
  } else {
    if (sgp41.measureRawSignals(compensationRh, compensationT, srawVoc,
                                srawNox)) {
      TVOC = -1;
      NOX = -1;
      return;
    }
  }

  if (currentMillis - previousTVOC >= tvocInterval) {
    previousTVOC += tvocInterval;
    TVOC = voc_algorithm.process(srawVoc);
    NOX = nox_algorithm.process(srawNox);
  }
}

void updateCo2() {
  if (currentMillis - previousCo2 >= co2Interval) {
    previousCo2 += co2Interval;
    co2 = ag.getCO2_Raw();
  }
}

void updatePm25() {
  if (currentMillis - previousPm25 >= pm25Interval) {
    previousPm25 += pm25Interval;
    pm25 = ag.getPM2_Raw();
  }
}

void updateTempHum() {
  if (currentMillis - previousTempHum >= tempHumInterval) {
    previousTempHum += tempHumInterval;
    TMP_RH result = ag.periodicFetchData();
    temp = result.t;
    hum = result.rh;
  }
}

void setOLEDLines(const char *ln1, const char *ln2, const char *ln3) {
  char buf[9];
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_t0_16_tf);
    u8g2.drawStr(1, 10, ln1);
    u8g2.drawStr(1, 30, ln2);
    u8g2.drawStr(1, 50, ln3);
  } while (u8g2.nextPage());
}

void updateOLED() {
  if (currentMillis - previousOled >= oledInterval) {
    previousOled += oledInterval;

    const std::size_t MAX_LINE_CHARS = 24;

    char line1[MAX_LINE_CHARS] = "";
    char line2[MAX_LINE_CHARS] = "";
    char line3[MAX_LINE_CHARS] = "";

    if (inUSAQI) {
      std::snprintf(line1, MAX_LINE_CHARS, "AQI:%3d CO2:%4d", pmToAqi(pm25),
                    co2);
    } else {
      std::snprintf(line1, MAX_LINE_CHARS, "PM:%3d CO2:%4d", pm25, co2);
    }

    if (TVOC < 0) {
      std::snprintf(line2, MAX_LINE_CHARS, "TVOC: -  NOX: -");
    } else {
      std::snprintf(line2, MAX_LINE_CHARS, "TVOC:%3d NOX:%3d", TVOC, NOX);
    }

    if (inF) {
      std::snprintf(line3, MAX_LINE_CHARS, "%3d\260F RH:%3d%%", tempToF(temp),
                    static_cast<int>(round(hum)));
    } else {
      std::snprintf(line3, MAX_LINE_CHARS, "%3d\260C RH:%3d%%",
                    static_cast<int>(round(temp)),
                    static_cast<int>(round(hum)));
    }

    setOLEDLines(line1, line2, line3);
  }
}

void sendToServer() {
  if (currentMillis - previousSendToServer >= sendToServerInterval) {
    previousSendToServer += sendToServerInterval;

    char payload[128];
    char *front = payload;
    const char * const back = payload + sizeof(payload);
    int ret = 0;

#define SNPRINTF_P(...) do { \
    ret = std::snprintf(front, back - front,  __VA_ARGS__ ); \
    if (ret < 0 || (front += ret) >= back) return; } while (0)

    SNPRINTF_P("{\"wifi\":%hhd", WiFi.RSSI());
    if (co2 >= 0) {
      SNPRINTF_P(",\"rco2\":%d", co2);
    }
    if (pm25 >= 0) {
      SNPRINTF_P(",\"pm02\":%d", pm25);
    }
    if (TVOC >= 0) {
      SNPRINTF_P(",\"tvoc\":%d", TVOC);
    }
    if (NOX >= 0) {
      SNPRINTF_P(",\"nox\":%d", NOX);
    }
    if (!isnan(temp)) {
      SNPRINTF_P(",\"atmp\":%f", static_cast<double>(temp));
    }
    if (!isnan(hum)) {
      SNPRINTF_P(",\"rhum\":%f", static_cast<double>(hum));
    }
    SNPRINTF_P("}");

#undef SNPRINTF_P

    if (WiFi.status() == WL_CONNECTED) {
      WiFiClient client;
      HTTPClient http;
      http.begin(client, API_ENDPOINT);
      http.addHeader("content-type", "application/json");
      int result = http.POST(payload);
      Serial.println(result);
      Serial.println(payload);
      http.end();
    } else {
      Serial.println("WiFi Disconnected");
    }
  }
}

// Wifi Manager
void connectToWifi() {
  WiFiManager wifiManager;
  // WiFi.disconnect(); //to delete previous saved hotspot
  char hotspot[16];
  std::snprintf(hotspot, sizeof(hotspot), "AG-%08x", ESP.getChipId());
  setOLEDLines("60s to connect", "to Wifi Hotspot", hotspot);
  wifiManager.setTimeout(60);

  if (!wifiManager.autoConnect(hotspot)) {
    setOLEDLines("Booting into", "offline mode", "");
    Serial.println("failed to connect and hit timeout");
    delay(6000);
  }
}

void setConfig() {
  switch (buttonConfig) {
  case 0:
    setOLEDLines("Temp. in C", "PM in ug/m3", "Display Top");
    u8g2.setDisplayRotation(U8G2_R2);
    inF = false;
    inUSAQI = false;
    break;
  case 1:
    setOLEDLines("Temp. in C", "PM in US AQI", "Display Top");
    u8g2.setDisplayRotation(U8G2_R2);
    inF = false;
    inUSAQI = true;
    break;
  case 2:
    setOLEDLines("Temp. in F", "PM in ug/m3", "Display Top");
    u8g2.setDisplayRotation(U8G2_R2);
    inF = true;
    inUSAQI = false;
    break;
  case 3:
    setOLEDLines("Temp. in F", "PM in US AQI", "Display Top");
    u8g2.setDisplayRotation(U8G2_R2);
    inF = true;
    inUSAQI = true;
    break;
  case 4:
    setOLEDLines("Temp. in C", "PM in ug/m3", "Display Top");
    u8g2.setDisplayRotation(U8G2_R0);
    inF = false;
    inUSAQI = false;
    break;
  case 5:
    setOLEDLines("Temp. in C", "PM in US AQI", "Display Top");
    u8g2.setDisplayRotation(U8G2_R0);
    inF = false;
    inUSAQI = true;
    break;
  case 6:
    setOLEDLines("Temp. in F", "PM in ug/m3", "Display Top");
    u8g2.setDisplayRotation(U8G2_R0);
    inF = true;
    inUSAQI = false;
    break;
  case 7:
    setOLEDLines("Temp. in F", "PM in US AQI", "Display Top");
    u8g2.setDisplayRotation(U8G2_R0);
    inF = true;
    inUSAQI = true;
    break;
  }
}

void inConf() {
  setConfig();
  int currentState = digitalRead(D7);

  if (lastState == LOW && currentState == HIGH) {
    pressedTime = millis();
  }

  else if (lastState == HIGH && currentState == LOW) {
    releasedTime = millis();
    long pressDuration = releasedTime - pressedTime;
    if (pressDuration < 1000) {
      buttonConfig++;
      if (buttonConfig > 7)
        buttonConfig = 0;
    }
  }

  if (lastState == HIGH && currentState == HIGH) {
    long passedDuration = millis() - pressedTime;
    if (passedDuration > 4000) {
      setOLEDLines("Saved", "Release", "Button Now");
      delay(1000);
      setOLEDLines("Rebooting", "in", "5 seconds");
      delay(5000);
      EEPROM.write(addr, buttonConfig);
      EEPROM.commit();
      delay(1000);
      ESP.restart();
    }
  }
  lastState = currentState;
  delay(100);
  inConf();
}

void setup() {
  Serial.begin(115200);
  u8g2.begin();

  EEPROM.begin(512);
  delay(500);

  buttonConfig = EEPROM.read(addr);
  setConfig();

  setOLEDLines("Press button", "now for", "config menu");
  delay(2000);

  if (digitalRead(D7) == HIGH) {
    setOLEDLines("Entering", "config menu", "");
    delay(3000);
    lastState = LOW;
    inConf();
  }

  if (connectWIFI) {
    connectToWifi();
  }

  std::snprintf(API_ENDPOINT, sizeof(API_ENDPOINT), "%s%x", &API_ROOT,
                ESP.getChipId());

  setOLEDLines("Warming up the", "sensors.", "");
  sgp41.begin(Wire);
  ag.CO2_Init();
  ag.PMS_Init();
  ag.TMP_RH_Init(0x44);
}

void loop() {
  currentMillis = millis();
  updateTVOC();
  updateOLED();
  updateCo2();
  updatePm25();
  updateTempHum();
  sendToServer();
  delay(minInterval / 2);
}
