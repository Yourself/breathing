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
#include "gas_index.h"
#include "json.h"

// #include "SGP30.h"
#include <NOxGasIndexAlgorithm.h>
#include <SensirionI2CSgp41.h>
#include <U8g2lib.h>
#include <VOCGasIndexAlgorithm.h>

#include <cstdio>

AirGradient ag = AirGradient(false, 115200);

// for peristent saving and loading
int addr = 4;

// Display bottom right
// U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Replace above if you have display on top left
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// CONFIGURATION START

// set to the endpoint you would like to use
char API_ROOT[] = "http://192.168.0.16:3000/api/restricted/submit/";
char API_ENDPOINT[128];
char API_CONTROL[] = "http://192.168.0.16:3000/api/restricted/control/";
char API_CONTROL_EP[128];

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

const int co2Interval = 5000;
unsigned long previousCo2 = 0;
int co2 = 0;

const int pm25Interval = 5000;
unsigned long previousPm25 = 0;
int pm25 = 0;

const int tempHumInterval = 2500;
unsigned long previousTempHum = 0;
float temp = 0;
int rhum = 0;

GasIndexStateMachine gasIndex;

bool displayOn = true;

const int minInterval =
    std::min({oledInterval, sendToServerInterval, gasIndex.updateInterval,
              co2Interval, pm25Interval, tempHumInterval});

std::uint8_t buttonConfig = 4;
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

int getPushButtonState() {
#if PCB_VERSION < 40
  return digitalRead(D7);
#else
  return digitalRead(D7) == LOW ? HIGH : LOW;
#endif
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
    if (result.error == SHT3XD_NO_ERROR) {
      temp = result.t;
      rhum = result.rh;
    } else {
      Serial.print("SHT3XD Error: ");
      Serial.println(result.error);
      temp = NAN;
      rhum = -1;
    }
  }
}

void setOLEDLines(const char *ln1, const char *ln2, const char *ln3) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_t0_16_tf);
    if (displayOn) {
      u8g2.drawStr(1, 10, ln1);
      u8g2.drawStr(1, 30, ln2);
      u8g2.drawStr(1, 50, ln3);
    }
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

    if (gasIndex.voc_available()) {
      if (gasIndex.nox_available()) {
        std::snprintf(line2, MAX_LINE_CHARS, "TVOC:%3d NOX:%3d", gasIndex.voc(),
                      gasIndex.nox());
      } else {
        std::snprintf(line2, MAX_LINE_CHARS, "TVOC:%3d NOX: -", gasIndex.voc());
      }
    } else {
      std::snprintf(line2, MAX_LINE_CHARS, "TVOC: -  NOX: -");
    }

    if (inF) {
      std::snprintf(line3, MAX_LINE_CHARS, "%3d\260F RH:%3d%%", tempToF(temp),
                    rhum);
    } else {
      std::snprintf(line3, MAX_LINE_CHARS, "%3d\260C RH:%3d%%",
                    static_cast<int>(round(temp)), rhum);
    }

    setOLEDLines(line1, line2, line3);
  }
}

void sendToServer() {
  if (currentMillis - previousSendToServer >= sendToServerInterval) {
    previousSendToServer += sendToServerInterval;

    char payload[128];
    JsonFormatter json(payload);
    {
      auto root = json.object();
      root.addMember("wifi", WiFi.RSSI());
      if (co2 >= 0)
        root.addMember("rco2", co2);
      if (pm25 >= 0)
        root.addMember("pm02", pm25);
      if (gasIndex.voc_available())
        root.addMember("tvoc", gasIndex.voc());
      if (gasIndex.nox_available())
        root.addMember("nox", gasIndex.nox());
      if (!isnan(temp))
        root.addMember("atmp", temp);
      if (rhum >= 0)
        root.addMember("rhum", rhum);
    }

    if (!json.formatter()) {
      Serial.println("Failed to render json");
    } else if (WiFi.status() == WL_CONNECTED) {
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

const int checkControllerInterval = 60 * 1000;
unsigned long previousCheckController = 0;

void checkController() {
  if (currentMillis - previousCheckController >= checkControllerInterval) {
    previousCheckController += checkControllerInterval;
    WiFiClient client;
    HTTPClient http;
    http.begin(client, API_CONTROL_EP);
    int result = http.GET();
    if (result == 200) {
      const auto &body = http.getString();
      Serial.println(body);
      int brightness = 255;
      std::sscanf(body.c_str(), "%d", &brightness);
      displayOn = brightness > 0;
      u8g2.setContrast(static_cast<uint8_t>(brightness));
    } else {
      Serial.print("Controller endpoint responded with: ");
      Serial.println(result);
    }
  }
}

// Wifi Manager
void connectToWifi() {
  WiFiManager wifiManager;
  // WiFi.disconnect(); //to delete previous saved hotspot
  char hotspot[16];
  std::snprintf(hotspot, sizeof(hotspot), "AG-%08x", ESP.getChipId());
  setOLEDLines("90s to connect", "to Wifi Hotspot", hotspot);
  wifiManager.setTimeout(90);

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
    setOLEDLines("Temp. in C", "PM in ug/m3", "Display Bottom");
    u8g2.setDisplayRotation(U8G2_R0);
    inF = false;
    inUSAQI = false;
    break;
  case 5:
    setOLEDLines("Temp. in C", "PM in US AQI", "Display Bottom");
    u8g2.setDisplayRotation(U8G2_R0);
    inF = false;
    inUSAQI = true;
    break;
  case 6:
    setOLEDLines("Temp. in F", "PM in ug/m3", "Display Bottom");
    u8g2.setDisplayRotation(U8G2_R0);
    inF = true;
    inUSAQI = false;
    break;
  case 7:
    setOLEDLines("Temp. in F", "PM in US AQI", "Display Bottom");
    u8g2.setDisplayRotation(U8G2_R0);
    inF = true;
    inUSAQI = true;
    break;
  default:
    buttonConfig = 0;
    break;
  }
}

void inConf() {
  bool config = true;
  while (config) {
    setConfig();
    int currentState = getPushButtonState();

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
        config = false;
        ESP.restart();
      }
    }
    lastState = currentState;
    delay(100);
  }
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

#if PCB_VERSION >= 40
  pinMode(D7, INPUT_PULLUP);
#endif

  if (getPushButtonState() == HIGH) {
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
  std::snprintf(API_CONTROL_EP, sizeof(API_CONTROL_EP), "%s%x", &API_CONTROL,
                ESP.getChipId());

  char serialBuf[24] = "";
  std::snprintf(serialBuf, sizeof(serialBuf), "%x", ESP.getChipId());

  setOLEDLines("Warming up", "Serial number:", serialBuf);
  gasIndex.begin(Wire);
  ag.CO2_Init();
  ag.PMS_Init();
  ag.TMP_RH_Init(0x44);
}

void loop() {
  currentMillis = millis();
  gasIndex.update(temp, rhum, currentMillis);
  updateCo2();
  updatePm25();
  updateTempHum();
  updateOLED();
  sendToServer();
  checkController();
  delay(minInterval / 2);
}
