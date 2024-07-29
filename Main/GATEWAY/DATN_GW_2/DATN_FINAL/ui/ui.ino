#include "FS.h"
#include <LittleFS.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <RTClib.h>
#include "LoRa_E32.h"
#include "freertos/portmacro.h"
#include <Preferences.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include "esp_task_wdt.h"
Preferences preferences_ino;
#define WIFI_SSID "BAD_Group"
#define WIFI_PASSWORD "deocodau"
#define EEPROM_SIZE 512     // Define the size of EEPROM (change based on your Arduino model)
#define AREA_SIZE 4         // Size for storing area (4 bytes)
#define NODE_BYTES 1        // Number of bytes to store each node
#define READ_INTERVAL 2000  // Read interval in milliseconds
// #define WIFI_SSID "Minh Huy"
// #define WIFI_PASSWORD "12345678"
String Web_App_URL = "https://script.google.com/macros/s/AKfycbyvQpDn7o-FetKJMsR1bsOdLIrg8oovJI52oZe67RAIkRFiMmE4AB8jP2coZR0oMbaD_Q/exec";
String Web_App_Relay = "https://script.google.com/macros/s/AKfycbwEZddoXVAG9ysNg-tygim7TN14mCsSWP7mGWMbjw4KwHJzvEaaVhU5mUXk6D66gxOb8A/exec";
#define API_KEY "AIzaSyCqJDt3vrOiJNmq7MFS8w28K9Pgz7PWFb0"
#define STORAGE_BUCKET_ID "smartfactory-f258b.appspot.com"
#define DATABASE_URL "https://smartfactory-f258b-default-rtdb.asia-southeast1.firebasedatabase.app/"  //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
#define USER_EMAIL "huy02010102@gmail.com"
#define USER_PASSWORD "minhuy03072002"
String parentPath = "/FactoryControl";
String childPath[3] = { "/Control", "/Audio", "/PassHW" };
FirebaseData fbdo;
FirebaseData stream;
FirebaseAuth authentication;
FirebaseConfig configuration;
byte relayStateStringSheet[4];
byte OnOffAll = 0;
byte sendControlInfor = 0;
byte receiveData = 0;
byte disconnectSensor = 0;
byte numAreas = 1, numNodesPerArea = 3;
int currentArea = 1;
int currentNode = 1;
uint8_t hour, minute;
bool relaystatus[4] = { 0 };
bool timer = 0;
bool status = 0;
TaskHandle_t xHandle = NULL;
#define FORMAT_LITTLEFS_IF_FAILED true
#define EEPROM_SIZE 404
bool answernodelora = false;
bool checkPresentDataFirst = false;
bool aver = false;
bool CheckSwitch = false;
bool btn_send_ctrl = false;
bool checksend = false;
byte sendcmd = 0;
byte sentnode = 0;
byte checkstreamFB = 0;
byte checkGW = 0;
byte addh_dis = 0;
byte addl_dis = 0;
bool btn_timer = false;
uint16_t deviceNumber;
bool deviceNB_timer[8] = { 0 };
byte NumberOfArea = 1;
byte NumberOfIDNUM = 2;
HardwareSerial mySerial(1);  // RX, TX
LoRa_E32 e32ttl(&mySerial, 45, 47, 48);
byte addh_node;
byte addl_node;
bool checkwifi = false;

unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
unsigned long lastDataSentTime = 0;
uint16_t IdNumChooseScr1 = 0;
uint16_t areaChooseScr1 = 0;
uint16_t IdNumChooseScr2 = 0;
uint16_t areaChooseScr2 = 0;
uint16_t IdNumChooseScr4 = 0;
uint16_t areaChooseScr4 = 0;
RTC_DS1307 rtc;
DateTime now;

SemaphoreHandle_t Mutex;
/*Change to your screen resolution*/
static const uint16_t screenWidth = 480;
static const uint16_t screenHeight = 320;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf) {
}
#endif
// The setup function is called once at startup of the sketch
typedef struct DataSensor {
  byte header1;
  byte ID_NUM;   // STT Dây chuyền (NODE)
  byte area;     // STT KHU
  byte FAC_NUM;  // STT nhà máy
  byte t[4];
  byte h[4];
  byte pm1[2];
  byte pm2_5[2];
  byte pm10[2];
  byte lux[2];
  byte CO[2];
  byte CO2[2];
  char Fire[2];
  char timestamp[15];
};
struct DataSensor datasensor;

typedef struct AverageSensor {
  float avg_temp = 0;
  float avg_humi = 0;
  float avg_pm1 = 0;
  float avg_pm2_5 = 0;
  float avg_pm10 = 0;
  float avg_lux = 0;
  float avg_CO2 = 0;
  unsigned long count = 0;
};
struct AverageSensor averSen;

typedef struct configData {
  byte header1 = 0B01000001;
  byte addh_gw = 0B00000001;
  byte addl_gw = 0B00000001;
  byte FAC_NUM = 0B10100001;  // STT nhà máy
  byte addl;
  byte addh;
  byte chan;
};
struct configData dataconf;

typedef struct controlRelayGW {
  byte header1 = 0B01000011;
  byte addh_gw = 0B00000001;
  byte addl_gw = 0B00000001;
  byte FAC_NUM = 0B10100001;  // STT nhà máy
  char OnOffStatus[5] = "2222";
};
struct controlRelayGW ctrrelayGW;

typedef struct MessCtrlGW {
  char Start_Sign[5] = "*CGC";
  controlRelayGW ctrrelayGW;
  byte CheckSum[2];
};
struct MessCtrlGW messctrlGW;

typedef struct MessConf {
  char Start_Sign[5] = "*PGN";
  configData dataconf;
  byte CheckSum[2];
};
struct MessConf messconf;

void parseAndPrintJson(const char *jsonString) {
  // Khởi tạo bộ đệm đủ lớn để chứa dữ liệu JSON
  static byte area_audio;
  static byte node_audio;
  static byte addh_fb_stream;
  static byte addl_fb_stream;
  static byte sendctrl = 0;
  static byte sendarea = 0;
  static byte confirmPW = 0;
  static bool statuscontrol[4] = { false };
  char relayStateString[5];
  byte status_fb_stream;
  byte status_audio;
  static byte count = 0;
  StaticJsonDocument<5120> doc;

  // Phân tích chuỗi JSON
  DeserializationError error = deserializeJson(doc, jsonString);

  // Kiểm tra lỗi
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  // Lặp qua tất cả các thành phần của JSON để lấy giá trị
  for (JsonPair kv : doc.as<JsonObject>()) {
    const char *key = kv.key().c_str();
    if (strcmp(key, "Area") == 0 && (kv.value().is<int>() || kv.value().is<const char *>())) {
      if (kv.value().is<int>()) {
        addh_fb_stream = kv.value();
      }
      if (kv.value().is<const char *>()) {
        const char *value = kv.value();
        for (int i = 0; i < strlen(value); i++) {
          if (i == 0) {
            addh_fb_stream = value[i] - '0';
          }
        }
      }
      sendarea = 1;
      uint16_t area_sel = addh_fb_stream;
      if (count != 0) {
        lv_dropdown_set_selected(ui_AreaScr2, area_sel);
      }
    }
    if (strcmp(key, "Node") == 0 && (kv.value().is<int>() || kv.value().is<const char *>())) {
      if (kv.value().is<int>()) {
        addl_fb_stream = kv.value();
      }
      if (kv.value().is<const char *>()) {
        const char *value = kv.value();
        for (int i = 0; i < strlen(value); i++) {
          if (i == 0) {
            addl_fb_stream = value[i] - '0';
          }
        }
      }
      sendarea = 1;
      uint16_t node_sel = addl_fb_stream;
      if (count != 0) {
        lv_dropdown_set_selected(ui_NodeScr2, node_sel);
      }
    }
    if (strcmp(key, "Dev 1") == 0 && kv.value().is<int>()) {
      status_fb_stream = kv.value();
      if (count != 0) {
        if (status_fb_stream == 1) {
          statuscontrol[0] = true;
          sendctrl = 1;
          lv_obj_add_state(ui_SwitchLIGHTScr2, LV_STATE_CHECKED);
        } else if (status_fb_stream == 0) {
          statuscontrol[0] = false;
          sendctrl = 1;
          lv_obj_clear_state(ui_SwitchLIGHTScr2, LV_STATE_CHECKED);
        }
      }
    } else if (strcmp(key, "Dev 2") == 0 && kv.value().is<int>()) {
      status_fb_stream = kv.value();
      if (count != 0) {
        if (status_fb_stream == 1) {
          statuscontrol[1] = true;
          sendctrl = 1;
          lv_obj_add_state(ui_SwitchFANScr2, LV_STATE_CHECKED);
        }
        if (status_fb_stream == 0) {
          statuscontrol[1] = false;
          sendctrl = 1;
          lv_obj_clear_state(ui_SwitchFANScr2, LV_STATE_CHECKED);
        }
      }
    } else if (strcmp(key, "Dev 3") == 0 && kv.value().is<int>()) {
      status_fb_stream = kv.value();
      if (count != 0) {
        if (status_fb_stream == 1) {
          statuscontrol[2] = true;
          sendctrl = 1;
          lv_obj_add_state(ui_SwitchDEVICE1Scr2, LV_STATE_CHECKED);
        }
        if (status_fb_stream == 0) {
          statuscontrol[2] = false;
          sendctrl = 1;
          lv_obj_clear_state(ui_SwitchDEVICE1Scr2, LV_STATE_CHECKED);
        }
      }
    } else if (strcmp(key, "Dev 4") == 0 && kv.value().is<int>()) {
      status_fb_stream = kv.value();
      if (count != 0) {
        if (status_fb_stream == 1) {
          statuscontrol[3] = true;
          sendctrl = 1;
          lv_obj_add_state(ui_SwitchDEVICE2Scr2, LV_STATE_CHECKED);
        }
        if (status_fb_stream == 0) {
          statuscontrol[3] = false;
          sendctrl = 1;
          lv_obj_clear_state(ui_SwitchDEVICE2Scr2, LV_STATE_CHECKED);
        }
      }
    }
    if (strcmp(key, "AreaAudio") == 0 && kv.value().is<int>()) {
      area_audio = kv.value().as<int>();
      Serial.println(area_audio);
    }
    if (strcmp(key, "NodeAudio") == 0 && kv.value().is<int>()) {
      node_audio = kv.value().as<int>();
      Serial.println(node_audio);
    }
    if (strcmp(key, "Audio") == 0 && kv.value().is<int>()) {
      status_audio = kv.value();
      Serial.println(status_audio);
      if (status_audio == 0) {
        struct PauseAudio {
          char Start_Sign[5] = "*PGC";
          byte audio;
        } pauseaudio;
        pauseaudio.audio = 0;
        if ((node_audio == 0) && (area_audio == 0)) {
          vTaskDelay(350);
          ResponseStatus rs = e32ttl.sendBroadcastFixedMessage(2, &pauseaudio, sizeof(pauseaudio));
          Serial.println(rs.getResponseDescription());
        }
      }
    }
    if (strcmp(key, "Confirm") == 0 && kv.value().is<int>()) {
      confirmPW = kv.value();
    }
    if (strcmp(key, "Password") == 0 && kv.value().is<const char *>()) {
      const char *value = kv.value();
      if (confirmPW == 1) {
        preferences_ino.begin("User", false);
        preferences_ino.putString("password", String(value));
        preferences_ino.end();
      }
    }
    if (strcmp(key, "Test") == 0 && kv.value().is<int>()) {
      byte test = kv.value();
      if (test == 2) {
        checkGW = 1;
      }
    }
  }
  if ((sendctrl == 1) || (sendarea == 1)) {
    for (int i = 0; i < 4; i++) {
      if (statuscontrol[i]) {
        relayStateString[i] = '1';
      } else {
        relayStateString[i] = '0';
      }
    }
    if (count != 0) {
      strcpy(messctrlGW.ctrrelayGW.OnOffStatus, relayStateString);
      sendCmd(addh_fb_stream, addl_fb_stream);
      sendctrl = 0;
      sendarea = 0;
    }
  }
  if (count == 0) {
    count = 1;
  }
}

void streamCallback(MultiPathStream stream) {
  checkstreamFB = 1;
  size_t numChild = sizeof(childPath) / sizeof(childPath[0]);
  for (size_t i = 0; i < numChild; i++) {
    if (stream.get(childPath[i])) {
      Serial.printf("path: %s, event: %s, type: %s, value: %s%s", stream.dataPath.c_str(), stream.eventType.c_str(), stream.type.c_str(), stream.value.c_str(), i < numChild - 1 ? "\n" : "");
    }
    if (strcmp(stream.type.c_str(), "json") == 0) {
      parseAndPrintJson(stream.value.c_str());
    }
    // Serial.printf("Received stream payload size: %d (Max. %d)\n\n", stream.payloadLength(), stream.maxPayloadLength());
  }
  checkstreamFB = 0;
}
void streamTimeoutCallback(bool timeout) {
  if (timeout)
    Serial.println(F("stream timed out, resuming...\n"));

  if (!stream.httpConnected())
    Serial.printf("error code: %d, reason: %s\n\n", stream.httpCode(), stream.errorReason().c_str());
}

bool createEEPROM(int numAreas, int numNodesPerArea) {
  int totalNodes = numAreas * numNodesPerArea;
  int totalNodeBytes = totalNodes * NODE_BYTES;

  if (totalNodeBytes + AREA_SIZE > EEPROM_SIZE) {
    // Not enough space in EEPROM
    return false;
  }

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Write area values to EEPROM (addresses 1 to 4)
  for (int i = 1; i <= AREA_SIZE; i++) {
    EEPROM.write(i, i);
  }

  // Write node values to EEPROM for each area
  int nodeValue = 1;
  for (int a = 1; a <= numAreas; a++) {
    for (int n = 1; n <= numNodesPerArea; n++) {
      EEPROM.write((a - 1) * numNodesPerArea + n + AREA_SIZE, nodeValue);
      nodeValue++;
    }
    nodeValue = 1;
  }

  // Commit changes to EEPROM
  EEPROM.commit();
  return true;
}

/*CheckSum XOR 16 bit*/
uint16_t XORChecksum16(const byte *data, size_t dataLength) {
  uint16_t value = 0;
  for (size_t i = 0; i < dataLength / 2; i++) {
    value ^= data[2 * i] + (data[2 * i + 1] << 8);
  }
  if (dataLength % 2)
    value ^= data[dataLength - 1];
  return ~value;
}

// Hàm tính toán checksum cho dữ liệu bất kỳ
uint16_t calculateChecksum(const void *data, size_t dataSize) {
  const byte *dataBytes = (const byte *)data;
  return XORChecksum16(dataBytes, dataSize);
}

// Hàm tính toán trung bình
void updateAveragesInt(uint16_t newValue, float *average, unsigned long count) {
  *average = ((*average) * count + newValue) / (count + 1);
}
void updateAveragesFLoat(float newValue, float *average, unsigned long count) {
  *average = ((*average) * count + newValue) / (count + 1);
}

boolean runEvery(unsigned long interval, unsigned long *previousMillis) {
  unsigned long currentMillis = millis();
  if (currentMillis - *previousMillis >= interval) {
    *previousMillis = currentMillis;
    return true;
  }
  return false;
}
void CheckWiFi(void *pvParameters) {
  (void)pvParameters;
  const unsigned long interval = 5000;
  while (1) {
    vTaskDelay(5 / portTICK_PERIOD_MS);
    if (runEvery(5000, &previousMillis2)) {
      if (WiFi.status() != WL_CONNECTED) {
        unsigned long reconnectAttemptTime = millis();
        WiFi.disconnect();
        WiFi.reconnect();
        if (WiFi.status() != WL_CONNECTED && millis() - reconnectAttemptTime < interval) {
          Serial.print(F("."));
          vTaskDelay(100);
        }
        if (WiFi.status() == WL_CONNECTED) {
          checkwifi = true;
          // Serial.println("Reconnected");
        } else {
          checkwifi = false;
          // Serial.println("Failed to reconnect");
        }
      } else {
        checkwifi = true;
        // Serial.println("WiFi connected.");
      }
      if (Firebase.ready()) {
      }
    }
    if ((runEvery(60000 * 60, &previousMillis3)) && (checkwifi == true)) {
      vTaskDelay(2 / portTICK_PERIOD_MS);
      char path[30];
      char DateFB[] = "DD_MM_YYYY";
      char HourFB[] = "hh:mm";
      now.toString(DateFB);
      now.toString(HourFB);
      strcpy(path, "/");
      strcat(path, DateFB);
      strcat(path, "/");
      strcat(path, HourFB);
      strcat(path, "/data.csv");
      Serial.println(path);
      esp_task_wdt_reset();
      if (!Firebase.Storage.upload(&fbdo, STORAGE_BUCKET_ID, "/data.csv", mem_storage_type_flash, path, "file/csv", fcsUploadCallback)) {
        Serial.println(fbdo.errorReason());
      } else {
        if (LittleFS.remove("/data.csv")) {
          Serial.println(F("- file deleted"));
        } else {
          Serial.println(F("- delete failed"));
        }
      }
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
void ReceiveDataSensor(void *pvParameters) {
  (void)pvParameters;
  static byte countsentFailed = 0;
  typedef struct AnswerLoRa {
    byte header1 = 0B00000000;
    byte addh_gw = 0B00000001;
    byte addl_gw = 0B00000001;
    byte FAC_NUM = 0B10100001;  // STT nhà máy
    char Answer[8] = "success";
  };
  struct AnswerLoRa answer;
  while (1) {
    esp_task_wdt_init(30, false);
    if (e32ttl.available() > 1) {
      answernodelora = true;
      struct MessDS {
        DataSensor datasensor;
        byte CheckSum[2];
      };
      struct data_crc {
        char Start_Sign[5];  // first part of structure
        struct MessDS messDS;
      } a1;

      typedef struct MessAns {
        char Start_Sign[5] = "*AGS";
        AnswerLoRa answer;
        byte CheckSum[2];
      };
      struct MessAns messans;
      ResponseContainer rs = e32ttl.receiveInitialMessage(sizeof(a1.Start_Sign));
      String typeStr = rs.data;
      if (typeStr == "*DSG") {
        ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessDS));
        a1.messDS = *(MessDS *)rsc.data;
        uint16_t x = calculateChecksum(&a1.messDS.datasensor, sizeof(a1.messDS.datasensor));
        datasensor = a1.messDS.datasensor;
        rsc.close();
        /*Answer*/
        *(uint16_t *)(messans.CheckSum) = calculateChecksum(&messans.answer, sizeof(messans.answer));
        vTaskDelay(350 / portTICK_PERIOD_MS);
        ResponseStatus rs_ans = e32ttl.sendFixedMessage(a1.messDS.datasensor.area, a1.messDS.datasensor.ID_NUM, 1, &messans, sizeof(MessAns));
        Serial.println(rs_ans.getResponseDescription());
        int area = EEPROM.read(currentArea);
        int address = AREA_SIZE + (currentArea - 1) * numNodesPerArea + currentNode;
        int node = EEPROM.read(address);
        if ((area == a1.messDS.datasensor.area) && (node == a1.messDS.datasensor.ID_NUM)) {
          sentnode = 0;
        }
        answernodelora = false;
        aver = true;
        receiveData = 1;
        writefile();
        CompareValueToAlert(*(float *)(datasensor.t), *(float *)(datasensor.h), *(uint16_t *)(datasensor.pm1), *(uint16_t *)(datasensor.pm2_5), *(uint16_t *)(datasensor.pm10), *(uint16_t *)(datasensor.lux), *(uint16_t *)(datasensor.CO2), datasensor.Fire, datasensor.area, datasensor.ID_NUM);
        if (checkwifi == true) {
          esp_task_wdt_reset();
          updateSheet();
          vTaskDelay(10 / portTICK_PERIOD_MS);
        }
      } else {
        answernodelora = false;
      }
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    if ((sentnode == 1) && (millis() - lastDataSentTime >= 7000)) {
      vTaskDelay(2 / portTICK_PERIOD_MS);
      countsentFailed += 1;
      if (countsentFailed != 2) {
        xTaskCreatePinnedToCore(sendRequest, "sendRequest", 5000, NULL, 4, NULL, 0);
        lastDataSentTime = millis();
      } else if (countsentFailed == 2) {
        addh_dis = EEPROM.read(currentArea);
        int address = AREA_SIZE + (currentArea - 1) * numNodesPerArea + currentNode;
        addl_dis = EEPROM.read(address);
        sentnode = 0;
        countsentFailed = 0;
        disconnectSensor = 1;
        if (checksend == true) {
          if (sentnode == 0) {
            currentNode++;
            if (currentNode > numNodesPerArea) {
              currentNode = 1;
              currentArea++;
              if (currentArea > numAreas) {
                currentArea = 1;
              }
            }
            checksend = false;
          }
          vTaskDelay(5 / portTICK_PERIOD_MS);
        }
      }
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    esp_task_wdt_reset();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
void sendrequest(void *pvParameters) {
  (void)pvParameters;
  static unsigned long previousMillis1 = 0;
  while (1) {
    vTaskDelay(5 / portTICK_PERIOD_MS);
    if ((runEvery(20000, &previousMillis1)) && (sentnode == 0)) {
      xTaskCreatePinnedToCore(sendRequest, "sendRequest", 5000, NULL, 4, NULL, 0);
      checksend = true;
      vTaskDelay(5 / portTICK_PERIOD_MS);
      sentnode = 1;
      lastDataSentTime = millis();
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
void sendRequest(void *pvParameters) {
  (void)pvParameters;
  typedef struct Request {
    byte header1 = 0B01000010;
    byte addh_gw = 0B00000001;
    byte addl_gw = 0B00000001;
    byte FAC_NUM = 0B10100001;  // STT nhà máy
    byte req = 1;
  };
  struct Request request;
  typedef struct MessRequest {
    char Start_Sign[5] = "*RGS";
    Request request;
    byte CheckSum[2];
  };
  struct MessRequest messrequest;
  *(uint16_t *)(messrequest.CheckSum) = calculateChecksum(&messrequest.request, sizeof(messrequest.request));
  int area = EEPROM.read(currentArea);
  int address = AREA_SIZE + (currentArea - 1) * numNodesPerArea + currentNode;
  int node = EEPROM.read(address);
  Serial.println(area);
  Serial.println(node);
  vTaskDelay(350 / portTICK_PERIOD_MS);
  if ((btn_send_ctrl == false) && (sendcmd == 0)) {
    ResponseStatus rs = e32ttl.sendFixedMessage(area, node, 1, &messrequest, sizeof(messrequest));
    Serial.println(rs.getResponseDescription());
  }
  vTaskDelete(NULL);
}

void SendConfigData(void *pvParameters) {
  (void)pvParameters;
  *(uint16_t *)(messconf.CheckSum) = calculateChecksum(&messconf.dataconf, sizeof(messconf.dataconf));
  vTaskDelay(350 / portTICK_PERIOD_MS);
  ResponseStatus rs_dc1 = e32ttl.sendFixedMessage(messconf.dataconf.addh, messconf.dataconf.addl, messconf.dataconf.chan, &messconf, sizeof(messconf));
  Serial.println(rs_dc1.getResponseDescription());
  vTaskDelete(NULL);
}
/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  uint16_t touchX = 0, touchY = 0;

  bool touched = tft.getTouch(&touchX, &touchY, 600);

  if (!touched) {
    data->state = LV_INDEV_STATE_REL;
  } else {
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = screenWidth - touchX;
    data->point.y = touchY + 20;
  }
}
void setupfb() {
  Serial.print(F("Connected with IP: "));
  Serial.println(WiFi.localIP());
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
  configuration.api_key = API_KEY;
  configuration.database_url = DATABASE_URL;
  authentication.user.email = USER_EMAIL;
  authentication.user.password = USER_PASSWORD;
  configuration.token_status_callback = tokenStatusCallback;
  fbdo.setBSSLBufferSize(4096, 1024);
  Firebase.begin(&configuration, &authentication);
  configuration.fcs.upload_buffer_size = 512;
  Firebase.reconnectWiFi(true);
  // Firebase.setwriteSizeLimit(fbdo, "tiny");
  stream.keepAlive(5, 5, 1);
  if (!Firebase.RTDB.beginMultiPathStream(&stream, parentPath)) {
    Serial.printf("sream begin error, %s\n\n", stream.errorReason().c_str());
  }
  Firebase.RTDB.setMultiPathStreamCallback(&stream, streamCallback, streamTimeoutCallback);
}
void printNodeAddressAndValue(int numAreas, int numNodesPerArea) {
  for (int a = 1; a <= numAreas; a++) {
    for (int n = 1; n <= numNodesPerArea; n++) {
      int nodeAddress = (a - 1) * numNodesPerArea + n + AREA_SIZE;
      int nodeValue = EEPROM.read(nodeAddress);
      Serial.print(F("Node at address "));
      Serial.print(nodeAddress);
      Serial.print(F(" has value: "));
      Serial.println(nodeValue);
    }
  }
}
void setup() {
  Serial.begin(115200); /* prepare for possible serial debug */
  // esp_task_wdt_deinit();
  Wire.begin(8, 9);
  mySerial.begin(9600, SERIAL_8N1, 19, 20);
  EEPROM.begin(EEPROM_SIZE);
  e32ttl.begin();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Serial.print("Connecting to Wi-Fi");
  const unsigned long interval = 20000;
  unsigned long reconnectAttemptTime = millis();
  while ((WiFi.status() != WL_CONNECTED) && (millis() - reconnectAttemptTime < interval)) {
    Serial.print(F("."));
    vTaskDelay(100);
  }
  if (WiFi.status() == WL_CONNECTED) {
    checkwifi = true;
  } else {
    checkwifi = false;
  }
  if (checkwifi == true) {
    setupfb();
  }
  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
    Serial.println(F("LittleFS Mount Failed"));
    return;
  } else {
    Serial.println(F("Little FS Mounted Successfully"));
  }
  // Mở tệp CSV để ghi
  File file = LittleFS.open("/data.csv", "a");
  if (!file) {
    Serial.println(F("Failed to open file for writing"));
    return;
  }

  // Ghi dữ liệu cột tiêu đề nếu cần
  if (file.size() == 0) {
    file.println("header1,ID_NUM,area,FAC_NUM,t,h,pm1,pm2_5,pm10,lux,CO,CO2,Fire,timestamp");
  }
  file.close();
  if (createEEPROM(numAreas, numNodesPerArea)) {
    Serial.println(F("EEPROM created successfully."));
    // Serial.println("Node addresses and values:");
    printNodeAddressAndValue(numAreas, numNodesPerArea);
  } else {
    Serial.println(F("Failed to create EEPROM. Not enough space."));
    return;
  }
  Mutex = xSemaphoreCreateMutex();
  if (!rtc.begin()) {
    while (1)
      vTaskDelay(10);
  }
  if (!rtc.isrunning()) {
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

  tft.begin();        /* TFT init */
  tft.setRotation(3); /* Landscape orientation, flipped */

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
  ui_init();
  xTaskCreatePinnedToCore(MainTaskLVGL, "MainTaskLVGL", 5000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task_HomePage, "Task_HomePage", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskControl, "TaskControl", 15000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task_Timer, "Task_Timer", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(ReceiveDataSensor, "ReceiveDataSensor", 15000, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(TaskSendToFB, "TaskSendToFB", 15000, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(CheckWiFi, "CheckWiFi", 10000, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(sendrequest, "sendrequest", 5000, NULL, 3, NULL, 0);
}

void fcsUploadCallback(FCS_UploadStatusInfo info) {
  if (info.status == firebase_fcs_upload_status_init) {
    Serial.printf("Uploading file %s (%d) to %s\n", info.localFileName.c_str(), info.fileSize, info.remoteFileName.c_str());
  } else if (info.status == firebase_fcs_upload_status_upload) {
    Serial.printf("Uploaded %d%s, Elapsed time %d ms\n", (int)info.progress, "%", info.elapsedTime);
  } else if (info.status == firebase_fcs_upload_status_complete) {
    Serial.println(F("Upload completed\n"));
    FileMetaInfo meta = fbdo.metaData();
    // Serial.printf("Name: %s\n", meta.name.c_str());
    // Serial.printf("Bucket: %s\n", meta.bucket.c_str());
    // Serial.printf("contentType: %s\n", meta.contentType.c_str());
    // Serial.printf("Size: %d\n", meta.size);
    // Serial.printf("Generation: %lu\n", meta.generation);
    // Serial.printf("Metageneration: %lu\n", meta.metageneration);
    // Serial.printf("ETag: %s\n", meta.etag.c_str());
    // Serial.printf("CRC32: %s\n", meta.crc32.c_str());
    // Serial.printf("Tokens: %s\n", meta.downloadTokens.c_str());
    // Serial.printf("Download URL: %s\n\n", fbdo.downloadURL().c_str());
  } else if (info.status == firebase_fcs_upload_status_error) {
    Serial.printf("Upload failed, %s\n", info.errorMsg.c_str());
  }
}
void writefile() {
  File file = LittleFS.open("/data.csv", "a");
  if (!file) {
    Serial.println(F("Failed to open file for writing"));
  } else {
    if (file.size() == 0) {
      file.println("header1,ID_NUM,area,FAC_NUM,t,h,pm1,pm2_5,pm10,lux,CO,CO2,Fire,timestamp");
    }
    // Ghi dữ liệu từ struct vào tệp CSV
    file.print(datasensor.header1);
    file.print(",");
    file.print(datasensor.ID_NUM);
    file.print(",");
    file.print(datasensor.area);
    file.print(",");
    file.print(datasensor.FAC_NUM);
    file.print(",");
    file.print(*(float *)(datasensor.t));
    file.print(",");
    file.print(*(float *)(datasensor.h));
    file.print(",");
    file.print(*(uint16_t *)(datasensor.pm1));
    file.print(",");
    file.print(*(uint16_t *)(datasensor.pm2_5));
    file.print(",");
    file.print(*(uint16_t *)(datasensor.pm10));
    file.print(",");
    file.print(*(uint16_t *)(datasensor.lux));
    file.print(",");
    file.print(*(uint16_t *)(datasensor.CO));
    file.print(",");
    file.print(*(uint16_t *)(datasensor.CO2));
    file.print(",");
    file.print(datasensor.Fire);
    file.print(",");
    file.print(datasensor.timestamp);
    file.print("\r\n");
    file.close();
  }
}
void updateFB() {
  vTaskDelay(5 / portTICK_PERIOD_MS);
  Firebase.RTDB.setInt(&fbdo, F("/Factory/Sensor/Area"), datasensor.area);
  Firebase.RTDB.setInt(&fbdo, F("/Factory/Sensor/Node"), datasensor.ID_NUM);
  Firebase.RTDB.setFloat(&fbdo, F("/Factory/Sensor/Temp"), *(float *)(datasensor.t));
  Firebase.RTDB.setFloat(&fbdo, F("/Factory/Sensor/Humi"), *(float *)(datasensor.h));
  Firebase.RTDB.setInt(&fbdo, F("/Factory/Sensor/CO2"), *(uint16_t *)(datasensor.CO2));
  Firebase.RTDB.setInt(&fbdo, F("/Factory/Sensor/PM1"), *(uint16_t *)(datasensor.pm1));
  Firebase.RTDB.setInt(&fbdo, F("/Factory/Sensor/PM25"), *(uint16_t *)(datasensor.pm2_5));
  Firebase.RTDB.setInt(&fbdo, F("/Factory/Sensor/PM10"), *(uint16_t *)(datasensor.pm10));
  Firebase.RTDB.setInt(&fbdo, F("/Factory/Sensor/Light"), *(uint16_t *)(datasensor.lux));
  Firebase.RTDB.setString(&fbdo, F("/Factory/Sensor/Fire"), String(datasensor.Fire));
  Firebase.RTDB.setString(&fbdo, F("/Factory/Sensor/Timestamp"), String(datasensor.timestamp));
  vTaskDelay(5 / portTICK_PERIOD_MS);
}
void updateSheet() {
  if (WiFi.status() == WL_CONNECTED) {
    String Send_Data_URL = Web_App_URL + "?sts=write";
    Send_Data_URL += "&temp=" + String((int)(*(float *)(datasensor.t)));
    Send_Data_URL += "&hum=" + String((int)(*(float *)(datasensor.h)));
    Send_Data_URL += "&pm1=" + String(*(uint16_t *)(datasensor.pm1));
    Send_Data_URL += "&pm25=" + String(*(uint16_t *)(datasensor.pm2_5));
    Send_Data_URL += "&pm10=" + String(*(uint16_t *)(datasensor.pm10));
    Send_Data_URL += "&co2=" + String(*(uint16_t *)(datasensor.CO2));
    Send_Data_URL += "&lux=" + String(*(uint16_t *)(datasensor.lux));
    Send_Data_URL += "&fire=" + String(datasensor.Fire);
    HTTPClient http;

    // HTTP GET Request.
    http.begin(Send_Data_URL.c_str());
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    // Gets the HTTP status code.
    int httpCode = http.GET();
    // Serial.print("HTTP Status Code : ");
    vTaskDelay(5 / portTICK_PERIOD_MS);
    Serial.println(httpCode);
    esp_task_wdt_reset();
    // Getting response from google sheets.
    String payload;
    if (httpCode > 0) {
      payload = http.getString();
      Serial.println("Payload : " + payload);
    }
    esp_task_wdt_reset();
    http.end();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
void findLatestData(fs::FS &fs, const char *path, byte targetID_NUM, byte targetArea) {
  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println(F("- failed to open file for reading"));
    return;
  }

  // Serial.println("- Finding latest data:");

  DataSensor latestData;
  bool foundData = false;

  while (file.available()) {
    // Đọc dữ liệu từ mỗi dòng và phân tích nó
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      // Tách các trường dữ liệu từ dòng
      char *data = strdup(line.c_str());
      char *token = strtok(data, ",");
      byte header1 = atoi(token);  // header1 là trường đầu tiên
      token = strtok(NULL, ",");
      byte ID_NUM = atoi(token);  // ID_NUM là trường thứ hai
      token = strtok(NULL, ",");
      byte area = atoi(token);  // area là trường thứ ba

      // Kiểm tra xem dữ liệu có phù hợp với điều kiện không
      if (ID_NUM == targetID_NUM && area == targetArea) {
        // Lưu trữ dữ liệu nếu đây là dữ liệu mới nhất phù hợp
        latestData.header1 = header1;
        latestData.ID_NUM = ID_NUM;
        latestData.area = area;
        token = strtok(NULL, ",");
        latestData.FAC_NUM = atoi(token);
        token = strtok(NULL, ",");
        float tValue = atof(token);
        *(float *)(latestData.t) = tValue;
        token = strtok(NULL, ",");
        float hValue = atof(token);
        *(float *)(latestData.h) = hValue;
        token = strtok(NULL, ",");
        *(uint16_t *)(latestData.pm1) = atoi(token);
        token = strtok(NULL, ",");
        *(uint16_t *)(latestData.pm2_5) = atoi(token);
        token = strtok(NULL, ",");
        *(uint16_t *)(latestData.pm10) = atoi(token);
        token = strtok(NULL, ",");
        *(uint16_t *)(latestData.lux) = atoi(token);
        token = strtok(NULL, ",");
        *(uint16_t *)(latestData.CO) = atoi(token);
        token = strtok(NULL, ",");
        *(uint16_t *)(latestData.CO2) = atoi(token);
        token = strtok(NULL, ",");
        strcpy(latestData.Fire, token);
        token = strtok(NULL, ",");
        strcpy(latestData.timestamp, token);

        foundData = true;
      }
      free(data);
    }
  }

  // Kiểm tra xem có dữ liệu phù hợp không
  if (foundData) {
    // In ra dữ liệu mới nhất phù hợp
    // Serial.println("Latest data found:");
    // Serial.print("header1: ");
    // Serial.println(latestData.header1);
    // Serial.print("ID_NUM: ");
    // Serial.println(latestData.ID_NUM);
    // Serial.print("area: ");
    // Serial.println(latestData.area);
    // Serial.print("FAC_NUM: ");
    // Serial.println(latestData.FAC_NUM);
    // // In ra các trường dữ liệu khác của latestData
    // Serial.print("t: ");
    // Serial.println(*(float *)(latestData.t));
    // Serial.print("h: ");
    // Serial.println(*(float *)(latestData.h));
    // Serial.print("pm10: ");
    // Serial.println(*(uint16_t *)(latestData.pm10));
    // Serial.print("lux: ");
    // Serial.println(*(uint16_t *)(latestData.lux));
    // Serial.print("CO: ");
    // Serial.println(*(uint16_t *)(latestData.CO));
    // Serial.print("CO2: ");
    // Serial.println(*(uint16_t *)(latestData.CO2));
    // Serial.print("Fire: ");
    // Serial.println(latestData.Fire);
    // Serial.print("timestamp: ");
    // Serial.println(latestData.timestamp);
    lv_label_set_text_fmt(ui_NBTScr1, "%d", (int)*(float *)(latestData.t));
    lv_label_set_text_fmt(ui_NBHScr1, "%d", (int)*(float *)(latestData.h));
    lv_label_set_text_fmt(ui_NBCO2Scr1, "%d", *(uint16_t *)(latestData.CO2));
    lv_label_set_text_fmt(ui_NBLScr1, "%d", *(uint16_t *)(latestData.lux));
    lv_label_set_text_fmt(ui_NBPM1Scr1, "%d", *(uint16_t *)(latestData.pm1));
    lv_label_set_text_fmt(ui_NBPM25Scr1, "%d", *(uint16_t *)(latestData.pm2_5));
    lv_label_set_text_fmt(ui_NBPM10Scr1, "%d", *(uint16_t *)(latestData.pm10));
    CompareValueToPresent((uint16_t) * (float *)(latestData.t), (uint16_t) * (float *)(latestData.h), *(uint16_t *)(latestData.pm1), *(uint16_t *)(latestData.pm2_5), *(uint16_t *)(latestData.pm10), *(uint16_t *)(latestData.lux), *(uint16_t *)(latestData.CO2));
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  // else {
  //   Serial.println("No matching data found.");
  // }
  file.close();
}
void TakeMutex(void) {
  while (xSemaphoreTake(Mutex, portMAX_DELAY) != pdTRUE)
    ;
}
void GiveMutex(void) {
  xSemaphoreGive(Mutex);
}
void MainTaskLVGL(void *pvParameters) {
  while (1) {
    vTaskDelay(5 / portTICK_PERIOD_MS);
    char Date[] = "DD/MM/YYYY";
    char Hour[] = "hh:mm";
    now = rtc.now();
    hour = now.hour();
    minute = now.minute();
    now.toString(Date);
    now.toString(Hour);
    modifyTime(Date, Hour);
    TakeMutex();
    lv_timer_handler(); /* let the GUI do its work */
    GiveMutex();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
void modifyTime(const char *Date, const char *Hour) {
  lv_label_set_text(ui_DateScr1, Date);
  lv_label_set_text(ui_HourScr1, Hour);
  vTaskDelay(5 / portTICK_PERIOD_MS);
}
void presentaver() {
  updateAveragesFLoat(*(float *)(datasensor.t), &averSen.avg_temp, averSen.count);
  updateAveragesFLoat(*(float *)(datasensor.h), &averSen.avg_humi, averSen.count);
  updateAveragesInt(*(uint16_t *)(datasensor.pm1), &averSen.avg_pm1, averSen.count);
  updateAveragesInt(*(uint16_t *)(datasensor.pm2_5), &averSen.avg_pm2_5, averSen.count);
  updateAveragesInt(*(uint16_t *)(datasensor.pm10), &averSen.avg_pm10, averSen.count);
  updateAveragesInt(*(uint16_t *)(datasensor.lux), &averSen.avg_lux, averSen.count);
  updateAveragesInt(*(uint16_t *)(datasensor.CO2), &averSen.avg_CO2, averSen.count);
  lv_label_set_text_fmt(ui_NBTScr1, "%d", (int)averSen.avg_temp);
  lv_label_set_text_fmt(ui_NBHScr1, "%d", (int)averSen.avg_humi);
  lv_label_set_text_fmt(ui_NBCO2Scr1, "%d", (int)averSen.avg_CO2);
  lv_label_set_text_fmt(ui_NBLScr1, "%d", (int)averSen.avg_lux);
  lv_label_set_text_fmt(ui_NBPM1Scr1, "%d", (int)averSen.avg_pm1);
  lv_label_set_text_fmt(ui_NBPM25Scr1, "%d", (int)averSen.avg_pm2_5);
  lv_label_set_text_fmt(ui_NBPM10Scr1, "%d", (int)averSen.avg_pm10);
  CompareValueToPresent((uint16_t)(averSen.avg_temp), (uint16_t)(averSen.avg_humi), (uint16_t)(averSen.avg_pm1), (uint16_t)(averSen.avg_pm2_5), (uint16_t)(averSen.avg_pm10), (uint16_t)(averSen.avg_lux), (uint16_t)(averSen.avg_CO2));
  averSen.count++;
  vTaskDelay(10 / portTICK_PERIOD_MS);
}
void presentData() {
  vTaskDelay(5 / portTICK_PERIOD_MS);
  lv_label_set_text_fmt(ui_NBTScr1, "%d", (int)*(float *)(datasensor.t));
  lv_label_set_text_fmt(ui_NBHScr1, "%d", (int)*(float *)(datasensor.h));
  lv_label_set_text_fmt(ui_NBCO2Scr1, "%d", *(uint16_t *)(datasensor.CO2));
  lv_label_set_text_fmt(ui_NBLScr1, "%d", *(uint16_t *)(datasensor.lux));
  lv_label_set_text_fmt(ui_NBPM1Scr1, "%d", *(uint16_t *)(datasensor.pm1));
  lv_label_set_text_fmt(ui_NBPM25Scr1, "%d", *(uint16_t *)(datasensor.pm2_5));
  lv_label_set_text_fmt(ui_NBPM10Scr1, "%d", *(uint16_t *)(datasensor.pm10));
  CompareValueToPresent((uint16_t) * (float *)(datasensor.t), (uint16_t) * (float *)(datasensor.h), *(uint16_t *)(datasensor.pm1), *(uint16_t *)(datasensor.pm2_5), *(uint16_t *)(datasensor.pm10), *(uint16_t *)(datasensor.lux), *(uint16_t *)(datasensor.CO2));
  vTaskDelay(5 / portTICK_PERIOD_MS);
}
void CompareValueToPresent(uint16_t t, uint16_t h, uint16_t pm1, uint16_t pm25, uint16_t pm10, uint16_t lux, uint16_t CO2) {
  vTaskDelay(5 / portTICK_PERIOD_MS);
  if (t < 30) {
    lv_obj_set_style_bg_color(ui_NameTempScr1, lv_color_hex(0x25CB3B), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else if ((t >= 30) && (t < 50)) {
    lv_obj_set_style_bg_color(ui_NameTempScr1, lv_color_hex(0xBDC601), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else if (t >= 50) {
    lv_obj_set_style_bg_color(ui_NameTempScr1, lv_color_hex(0xF53E2F), LV_PART_MAIN | LV_STATE_DEFAULT);
  }

  if ((h >= 75) && (h <= 85)) {
    lv_obj_set_style_bg_color(ui_NameHumiScr1, lv_color_hex(0x25CB3B), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else {
    lv_obj_set_style_bg_color(ui_NameHumiScr1, lv_color_hex(0xBDC601), LV_PART_MAIN | LV_STATE_DEFAULT);
  }

  if (pm1 <= 12) {
    lv_obj_set_style_bg_color(ui_NamePM1Scr1, lv_color_hex(0x25CB3B), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else if ((pm1 > 12) && (pm1 <= 60)) {
    lv_obj_set_style_bg_color(ui_NamePM1Scr1, lv_color_hex(0xBDC601), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else if (pm1 > 60) {
    lv_obj_set_style_bg_color(ui_NamePM1Scr1, lv_color_hex(0xF53E2F), LV_PART_MAIN | LV_STATE_DEFAULT);
  }

  if (pm25 <= 12) {
    lv_obj_set_style_bg_color(ui_NamePM25Scr1, lv_color_hex(0x25CB3B), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else if ((pm25 > 12) && (pm25 <= 80)) {
    lv_obj_set_style_bg_color(ui_NamePM25Scr1, lv_color_hex(0xBDC601), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else if (pm25 > 80) {
    lv_obj_set_style_bg_color(ui_NamePM25Scr1, lv_color_hex(0xF53E2F), LV_PART_MAIN | LV_STATE_DEFAULT);
  }

  if (pm10 <= 54) {
    lv_obj_set_style_bg_color(ui_NamePM10Scr1, lv_color_hex(0x25CB3B), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else if ((pm10 > 54) && (pm10 <= 154)) {
    lv_obj_set_style_bg_color(ui_NamePM10Scr1, lv_color_hex(0xBDC601), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else if (pm10 > 154) {
    lv_obj_set_style_bg_color(ui_NamePM10Scr1, lv_color_hex(0xF53E2F), LV_PART_MAIN | LV_STATE_DEFAULT);
  }

  if ((lux >= 350) && (lux <= 650)) {
    lv_obj_set_style_bg_color(ui_NameLightScr1, lv_color_hex(0x25CB3B), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else {
    lv_obj_set_style_bg_color(ui_NameLightScr1, lv_color_hex(0xBDC601), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  if (CO2 <= 1000) {
    lv_obj_set_style_bg_color(ui_NameCO2Scr1, lv_color_hex(0x25CB3B), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else if ((CO2 >= 1000) && (CO2 <= 2000)) {
    lv_obj_set_style_bg_color(ui_NameCO2Scr1, lv_color_hex(0xBDC601), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else if (CO2 > 2000) {
    lv_obj_set_style_bg_color(ui_NameCO2Scr1, lv_color_hex(0xF53E2F), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
  vTaskDelay(5 / portTICK_PERIOD_MS);
}
void CompareValueToAlert(float &t, float &h, uint16_t &pm1, uint16_t &pm25, uint16_t &pm10, uint16_t &lux, uint16_t &CO2, char (&Fire)[2], byte &addh, byte &addl) {
  typedef struct AlertGW {
    byte header1 = 0B01000011;
    byte addh_gw = 0B00000001;
    byte addl_gw = 0B00000001;
    byte FAC_NUM = 0B10100001;  // STT nhà máy
    byte alert_temp = 2;
    byte alert_pm = 2;
    byte alert_CO2 = 2;
    byte alert_Fire = 2;
  };
  struct AlertGW alertGW;
  typedef struct MessAlertGW {
    char Start_Sign[5] = "*AGC";
    AlertGW alertGW;
    byte CheckSum[2];
  };
  struct MessAlertGW messalertGW;

  int temp_thres = 60;
  float humi_thres = 85;
  int pm10_thres = 155;
  int pm25_thres = 80;
  int pm1_thres = 70;
  int lux_thres_low = 350;  // Thấp hơn 300 (tối thiểu 300)
  int lux_thres_high = 650;
  int CO2_thres = 2000;
  byte chan = 2;
  if (t >= temp_thres) {
    messalertGW.alertGW.alert_temp = 1;
  } else if (t < temp_thres - 20) {
    messalertGW.alertGW.alert_temp = 0;
  }
  if (h >= humi_thres) {
  } else if (h < humi_thres - 20) {
  }
  if ((pm1 >= pm1_thres) || (pm25 >= pm25_thres) || (pm10 >= pm10_thres)) {
    messalertGW.alertGW.alert_pm = 1;
  } else if ((pm1 < pm1_thres - 10) && (pm25 < pm25_thres - 15) && (pm10 < pm10_thres - 25)) {
    messalertGW.alertGW.alert_pm = 0;
  }
  if ((lux <= lux_thres_low) || (lux >= lux_thres_high)) {
  } else if ((lux > lux_thres_low + 50) && (lux < lux_thres_high - 50)) {
  }
  if (CO2 >= CO2_thres) {
    messalertGW.alertGW.alert_CO2 = 1;
  } else if (CO2 < CO2_thres - 600) {
    messalertGW.alertGW.alert_CO2 = 0;
  }
  if (strcmp(Fire, "Y") == 0) {
    messalertGW.alertGW.alert_Fire = 1;
  } else if (strcmp(Fire, "N") == 0) {
    messalertGW.alertGW.alert_Fire = 0;
  }
  vTaskDelay(350);
  ResponseStatus rs = e32ttl.sendFixedMessage(addh, addl, chan, &messalertGW, sizeof(messalertGW));
  Serial.println(rs.getResponseDescription());
}
void Task_HomePage(void *pvParameters) {
  while (1) {
    vTaskDelay(5 / portTICK_PERIOD_MS);
    TakeMutex();
    if (aver == true) {
      if ((IdNumChooseScr1 == 0) && (areaChooseScr1 == 0)) {
        presentaver();
      }
      if ((IdNumChooseScr1 != 0) && (areaChooseScr1 != 0) && (checkPresentDataFirst == false)) {
        // Serial.print(areaChooseScr1);
        // Serial.println(IdNumChooseScr1);
        findLatestData(LittleFS, "/data.csv", datasensor.ID_NUM, datasensor.area);
        checkPresentDataFirst = true;
      }
      if ((IdNumChooseScr1 != 0) && (areaChooseScr1 != 0) && (checkPresentDataFirst == true) && (datasensor.ID_NUM == IdNumChooseScr1) && (datasensor.area == areaChooseScr1)) {
        presentData();
      }
      aver = false;
    }
    GiveMutex();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
void sendCmd(byte &addh, byte &addl) {
  byte chan = 2;
  *(uint16_t *)(messctrlGW.CheckSum) = calculateChecksum(&messctrlGW.ctrrelayGW, sizeof(messctrlGW.ctrrelayGW));
  if (answernodelora == false) {
    sendcmd = 1;
    vTaskDelay(5 / portTICK_PERIOD_MS);
    if ((addh == 0) && (addl == 0)) {
      // Serial.println(String(messctrlGW.ctrrelayGW.OnOffStatus));
      vTaskDelay(350);
      ResponseStatus rs = e32ttl.sendBroadcastFixedMessage(chan, &messctrlGW, sizeof(messctrlGW));
      Serial.println(rs.getResponseDescription());
    } else {
      vTaskDelay(350);
      ResponseStatus rs = e32ttl.sendFixedMessage(addh, addl, chan, &messctrlGW, sizeof(messctrlGW));
      Serial.println(rs.getResponseDescription());
    }
    sendcmd = 0;
  } else {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    esp_task_wdt_reset();
    return;
  }
}
void TaskControl(void *pvParameters) {
  char On[5] = "1111";
  char Off[5] = "0000";
  char relayStateString[5] = "2222";
  while (1) {
    vTaskDelay(5 / portTICK_PERIOD_MS);
    TakeMutex();
    if (btn_send_ctrl == true) {
      for (int i = 0; i < 4; i++) {
        if (relaystatus[i]) {
          relayStateString[i] = '1';
          relayStateStringSheet[i] = 1;
        } else {
          relayStateString[i] = '0';
          relayStateStringSheet[i] = 0;
        }
      }
      strcpy(messctrlGW.ctrrelayGW.OnOffStatus, relayStateString);
      if (strcmp(relayStateString, On) == 0) {
        OnOffAll = 1;
      }
      if (strcmp(relayStateString, Off) == 0) {
        OnOffAll = 0;
      }
      addh_node = areaChooseScr2;
      addl_node = IdNumChooseScr2;
      byte chan_node = 2;
      *(uint16_t *)(messctrlGW.CheckSum) = calculateChecksum(&messctrlGW.ctrrelayGW, sizeof(messctrlGW.ctrrelayGW));
      vTaskDelay(350 / portTICK_PERIOD_MS);
      if ((addh_node == 0) && (addl_node == 0)) {
        // Serial.println(String(messctrlGW.ctrrelayGW.OnOffStatus));
        ResponseStatus rs = e32ttl.sendBroadcastFixedMessage(chan_node, &messctrlGW, sizeof(messctrlGW));
        Serial.println(rs.getResponseDescription());
      } else {
        ResponseStatus rs = e32ttl.sendFixedMessage(addh_node, addl_node, chan_node, &messctrlGW, sizeof(messctrlGW));
        Serial.println(rs.getResponseDescription());
      }
      btn_send_ctrl = false;
      sendControlInfor = 1;
    }
    GiveMutex();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
void TaskSendToFB(void *pvParameters) {
  static unsigned long startTime = 0;
  static bool started = false;
  while (1) {
    vTaskDelay(5 / portTICK_PERIOD_MS);
    if (sendControlInfor == 1) {
      Serial.println("sendControlInfor");
      if ((checkwifi == true) && (addh_node == 0) && (addl_node == 0) && (Firebase.ready())) {
        if (OnOffAll == 1) {
          Firebase.RTDB.setInt(&fbdo, F("/AllDevices/Switch"), 1);
        }
        if (OnOffAll == 0) {
          Firebase.RTDB.setInt(&fbdo, F("/AllDevices/Switch"), 0);
        }
      }
      if ((checkwifi == true) && (Firebase.ready())) {
        Firebase.RTDB.setInt(&fbdo, F("/FactoryControl/Control/Area"), addh_node);
        Firebase.RTDB.setInt(&fbdo, F("/FactoryControl/Control/Node"), addl_node);
        if (relayStateStringSheet[0] == 1) {
          Firebase.RTDB.setInt(&fbdo, F("/FactoryControl/Control/Dev 1"), 1);
        }
        if (relayStateStringSheet[0] == 0) {
          Firebase.RTDB.setInt(&fbdo, F("/FactoryControl/Control/Dev 1"), 0);
        }
        if (relayStateStringSheet[1] == 1) {
          Firebase.RTDB.setInt(&fbdo, F("/FactoryControl/Control/Dev 2"), 1);
        }
        if (relayStateStringSheet[1] == 0) {
          Firebase.RTDB.setInt(&fbdo, F("/FactoryControl/Control/Dev 2"), 0);
        }
        if (relayStateStringSheet[2] == 1) {
          Firebase.RTDB.setInt(&fbdo, F("/FactoryControl/Control/Dev 3"), 1);
        }
        if (relayStateStringSheet[2] == 0) {
          Firebase.RTDB.setInt(&fbdo, F("/FactoryControl/Control/Dev 3"), 0);
        }
        if (relayStateStringSheet[3] == 1) {
          Firebase.RTDB.setInt(&fbdo, F("/FactoryControl/Control/Dev 4"), 1);
        }
        if (relayStateStringSheet[3] == 0) {
          Firebase.RTDB.setInt(&fbdo, F("/FactoryControl/Control/Dev 4"), 0);
        }
      }
      sendControlInfor = 0;
    }
    if ((receiveData == 1) && (checkwifi == true) && (Firebase.ready())) {
      Firebase.RTDB.setInt(&fbdo, F("/Disconnect/Area"), datasensor.area);
      Firebase.RTDB.setInt(&fbdo, F("/Disconnect/Node"), datasensor.ID_NUM);
      Firebase.RTDB.setInt(&fbdo, F("/Disconnect/Disconnect"), 0);
      updateFB();
      receiveData = 0;
    }
    if ((disconnectSensor == 1) && (checkwifi == true) && (Firebase.ready())) {
      Firebase.RTDB.setInt(&fbdo, F("/Disconnect/Area"), addh_dis);
      Firebase.RTDB.setInt(&fbdo, F("/Disconnect/Node"), addl_dis);
      Firebase.RTDB.setInt(&fbdo, F("/Disconnect/Disconnect"), 1);
      Firebase.RTDB.setInt(&fbdo, F("/Disconnect/Confirm"), 0);
      disconnectSensor = 0;
    }
    if ((checkstreamFB == 0) && (checkGW == 1) && (checkwifi == true) && (Firebase.ready())) {
      Firebase.RTDB.setInt(&fbdo, F("/FactoryControl/PassHW/Test"), 1);
      checkGW = 0;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
void getInforTimer() {
  static bool statuscontrol[4] = { false };
  char relayStateString[5];
  preferences_ino.begin("DevTM", false);
  uint32_t timerEn1_Timer = preferences_ino.getInt("TmEn1", 2);
  uint32_t timerEn2_Timer = preferences_ino.getInt("TmEn2", 2);
  uint32_t timerEn3_Timer = preferences_ino.getInt("TmEn3", 2);
  uint32_t timerEn4_Timer = preferences_ino.getInt("TmEn4", 2);
  uint32_t timerEn1_Period = preferences_ino.getInt("TmEn1Period", 2);
  uint32_t timerEn2_Period = preferences_ino.getInt("TmEn2Period", 2);
  uint32_t timerEn3_Period = preferences_ino.getInt("TmEn3Period", 2);
  uint32_t timerEn4_Period = preferences_ino.getInt("TmEn4Period", 2);
  static bool timerOnExecuted[4] = { false, false, false, false };
  static bool timerOffExecuted[4] = { false, false, false, false };
  static bool PeriodOnExecuted[4] = { false, false, false, false };
  static bool PeriodOffExecuted[4] = { false, false, false, false };
  if (timerEn1_Timer != 2) {
    switch (timerEn1_Timer) {
      case 0:
        break;
      case 1:
        // Serial.println(F("Timer1"));
        uint32_t statusTimer1On = preferences_ino.getInt("statusTM1On", 2);
        uint32_t statusTimer1Off = preferences_ino.getInt("statusTM1Off", 2);
        if (statusTimer1On == 1) {
          // Serial.println(F("ON"));
          uint32_t timer_hour1_Timer_On = preferences_ino.getInt("TmHour1On", 0);
          uint32_t timer_minute1_Timer_On = preferences_ino.getInt("TmMin1On", 0);
          uint32_t IdNumNode_TM_On_1 = preferences_ino.getInt("Id1TMOn", 0);
          uint32_t areaNode_TM_On_1 = preferences_ino.getInt("area1TMOn", 0);
          if ((hour == timer_hour1_Timer_On) && (minute == timer_minute1_Timer_On) && (timerOnExecuted[0] == false) && (answernodelora == false)) {
            Serial.println(F("Timer 1 ON"));
            strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "1222");
            byte addh = areaNode_TM_On_1;
            byte addl = IdNumNode_TM_On_1;
            sendCmd(addh, addl);
            timerOnExecuted[0] = true;
            lv_obj_add_state(ui_SwitchLIGHTScr2, LV_STATE_CHECKED);
          }
          if ((hour != timer_hour1_Timer_On) || (minute != timer_minute1_Timer_On)) {
            timerOnExecuted[0] = false;
          }
        }
        if (statusTimer1Off == 0) {
          // Serial.println(F("OFF"));
          uint32_t timer_hour1_Timer_Off = preferences_ino.getInt("TmHour1Off", 0);
          uint32_t timer_minute1_Timer_Off = preferences_ino.getInt("TmMin1Off", 0);
          uint32_t IdNumNode_TM_Off_1 = preferences_ino.getInt("Id1TMOff", 0);
          uint32_t areaNode_TM_Off_1 = preferences_ino.getInt("area1TMOff", 0);
          if ((hour == timer_hour1_Timer_Off) && (minute == timer_minute1_Timer_Off) && (timerOffExecuted[0] == false) && (answernodelora == false)) {
            Serial.println(F("Timer 1 OFF"));
            byte addh = areaNode_TM_Off_1;
            byte addl = IdNumNode_TM_Off_1;
            strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "0222");
            sendCmd(addh, addl);
            timerOffExecuted[0] = true;
            lv_obj_clear_state(ui_SwitchLIGHTScr2, LV_STATE_CHECKED);
          }
          if ((hour != timer_hour1_Timer_Off) || (minute != timer_minute1_Timer_Off)) {
            timerOffExecuted[0] = false;
          }
        }
        break;
    }
  }
  if (timerEn2_Timer != 2) {
    switch (timerEn2_Timer) {
      case 0:
        break;
      case 1:
        uint32_t statusTimer2On = preferences_ino.getInt("statusTM2On", 2);
        uint32_t statusTimer2Off = preferences_ino.getInt("statusTM2Off", 2);
        if (statusTimer2On == 1) {
          uint32_t timer_hour2_Timer_On = preferences_ino.getInt("TmHour2On", 0);
          uint32_t timer_minute2_Timer_On = preferences_ino.getInt("TmMin2On", 0);
          uint32_t IdNumNode_TM_On_2 = preferences_ino.getInt("Id2TMOn", 0);
          uint32_t areaNode_TM_On_2 = preferences_ino.getInt("area2TMOn", 0);
          if ((hour == timer_hour2_Timer_On) && (minute == timer_minute2_Timer_On) && (timerOnExecuted[1] == false) && (answernodelora == false)) {
            Serial.println(F("Timer 2 ON"));
            strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "2122");
            byte addh = areaNode_TM_On_2;
            byte addl = IdNumNode_TM_On_2;
            sendCmd(addh, addl);
            timerOnExecuted[1] = true;
            lv_obj_add_state(ui_SwitchFANScr2, LV_STATE_CHECKED);
          }
          if ((hour != timer_hour2_Timer_On) || (minute != timer_minute2_Timer_On)) {
            timerOnExecuted[1] = false;
          }
        }
        if (statusTimer2Off == 0) {
          uint32_t timer_hour2_Timer_Off = preferences_ino.getInt("TmHour2Off", 0);
          uint32_t timer_minute2_Timer_Off = preferences_ino.getInt("TmMin2Off", 0);
          uint32_t IdNumNode_TM_Off_2 = preferences_ino.getInt("Id2TMOff", 0);
          uint32_t areaNode_TM_Off_2 = preferences_ino.getInt("area2TMOff", 0);
          if ((hour == timer_hour2_Timer_Off) && (minute == timer_minute2_Timer_Off) && (timerOffExecuted[1] == false) && (answernodelora == false)) {
            Serial.println(F("Timer 2 OFF"));
            strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "2022");
            byte addh = areaNode_TM_Off_2;
            byte addl = IdNumNode_TM_Off_2;
            sendCmd(addh, addl);
            timerOffExecuted[1] = true;
            lv_obj_clear_state(ui_SwitchFANScr2, LV_STATE_CHECKED);
          }
          if ((hour != timer_hour2_Timer_Off) || (minute != timer_minute2_Timer_Off)) {
            timerOffExecuted[1] = false;
          }
        }
        break;
    }
  }
  if (timerEn3_Timer != 2) {
    switch (timerEn3_Timer) {
      case 0:
        break;
      case 1:
        uint32_t statusTimer3On = preferences_ino.getInt("statusTM3On", 2);
        uint32_t statusTimer3Off = preferences_ino.getInt("statusTM3Off", 2);
        if (statusTimer3On == 1) {
          uint32_t timer_hour3_Timer_On = preferences_ino.getInt("TmHour3On", 0);
          uint32_t timer_minute3_Timer_On = preferences_ino.getInt("TmMin3On", 0);
          uint32_t IdNumNode_TM_On_3 = preferences_ino.getInt("Id3TMOn", 0);
          uint32_t areaNode_TM_On_3 = preferences_ino.getInt("area3TMOn", 0);
          if ((hour == timer_hour3_Timer_On) && (minute == timer_minute3_Timer_On) && (timerOnExecuted[2] == false) && (answernodelora == false)) {
            Serial.println(F("Timer 3 ON"));
            strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "2212");
            byte addh = areaNode_TM_On_3;
            byte addl = IdNumNode_TM_On_3;
            sendCmd(addh, addl);
            timerOnExecuted[2] = true;
            lv_obj_add_state(ui_SwitchDEVICE1Scr2, LV_STATE_CHECKED);
          }
          if ((hour != timer_hour3_Timer_On) || (minute != timer_minute3_Timer_On)) {
            timerOnExecuted[2] = false;
          }
        }
        if (statusTimer3Off == 0) {
          uint32_t timer_hour3_Timer_Off = preferences_ino.getInt("TmHour3Off", 0);
          uint32_t timer_minute3_Timer_Off = preferences_ino.getInt("TmMin3Off", 0);
          uint32_t IdNumNode_TM_Off_3 = preferences_ino.getInt("Id3TMOff", 0);
          uint32_t areaNode_TM_Off_3 = preferences_ino.getInt("area3TMOff", 0);
          if ((hour == timer_hour3_Timer_Off) && (minute == timer_minute3_Timer_Off) && (timerOffExecuted[2] == false) && (answernodelora == false)) {
            Serial.println(F("Timer 3 OFF"));
            strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "2202");
            byte addh = areaNode_TM_Off_3;
            byte addl = IdNumNode_TM_Off_3;
            sendCmd(addh, addl);
            timerOffExecuted[2] = true;
            lv_obj_clear_state(ui_SwitchDEVICE1Scr2, LV_STATE_CHECKED);
          }
          if ((hour != timer_hour3_Timer_Off) || (minute != timer_minute3_Timer_Off)) {
            timerOffExecuted[2] = false;
          }
        }
        break;
    }
  }
  if (timerEn4_Timer != 2) {
    switch (timerEn4_Timer) {
      case 0:
        break;
      case 1:
        uint32_t statusTimer4On = preferences_ino.getInt("statusTM4On", 2);
        uint32_t statusTimer4Off = preferences_ino.getInt("statusTM4Off", 2);
        if (statusTimer4On == 1) {
          uint32_t timer_hour4_Timer_On = preferences_ino.getInt("TmHour4On", 0);
          uint32_t timer_minute4_Timer_On = preferences_ino.getInt("TmMin4On", 0);
          uint32_t IdNumNode_TM_On_4 = preferences_ino.getInt("Id4TMOn", 0);
          uint32_t areaNode_TM_On_4 = preferences_ino.getInt("area4TMOn", 0);
          if ((hour == timer_hour4_Timer_On) && (minute == timer_minute4_Timer_On) && (timerOnExecuted[3] == false) && (answernodelora == false)) {
            Serial.println(F("Timer 4 ON"));
            strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "2221");
            byte addh = areaNode_TM_On_4;
            byte addl = IdNumNode_TM_On_4;
            sendCmd(addh, addl);
            timerOnExecuted[3] = true;
            lv_obj_add_state(ui_SwitchDEVICE2Scr2, LV_STATE_CHECKED);
          }
          if ((hour != timer_hour4_Timer_On) || (minute != timer_minute4_Timer_On)) {
            timerOnExecuted[3] = false;
          }
        }
        if (statusTimer4Off == 0) {
          uint32_t timer_hour4_Timer_Off = preferences_ino.getInt("TmHour4Off", 0);
          uint32_t timer_minute4_Timer_Off = preferences_ino.getInt("TmMin4Off", 0);
          uint32_t IdNumNode_TM_Off_4 = preferences_ino.getInt("Id4TMOff", 0);
          uint32_t areaNode_TM_Off_4 = preferences_ino.getInt("area4TMOff", 0);
          if ((hour == timer_hour4_Timer_Off) && (minute == timer_minute4_Timer_Off) && (timerOffExecuted[3] == false) && (answernodelora == false)) {
            Serial.println(F("Timer 4 OFF"));
            strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "2220");
            byte addh = areaNode_TM_Off_4;
            byte addl = IdNumNode_TM_Off_4;
            sendCmd(addh, addl);
            timerOffExecuted[3] = true;
            lv_obj_clear_state(ui_SwitchDEVICE2Scr2, LV_STATE_CHECKED);
          }
          if ((hour != timer_hour4_Timer_Off) || (minute != timer_minute4_Timer_Off)) {
            timerOffExecuted[3] = false;
          }
        }
        break;
    }
  }
  if (timerEn1_Period != 2) {
    if (timerEn1_Period == 0) {
      uint32_t statusTimer1On_Period = preferences_ino.getInt("statusPeriod1On", 2);
      uint32_t statusTimer1Off_Period = preferences_ino.getInt("statusPr1Off", 2);
      static uint8_t hour_add_1_On, minute_add_1_On;
      static uint8_t hour_add_1_Off, minute_add_1_Off;
      static bool PeriodOn1 = false;
      static bool PeriodOff1 = false;
      if (statusTimer1On_Period == 1) {
        uint32_t IdNumPr_On_1 = preferences_ino.getInt("Id1PrOn", 0);
        uint32_t areaPr_On_1 = preferences_ino.getInt("area1PrOn", 0);
        if (PeriodOn1 == false) {
          uint32_t timer_hour1_Timer_On = preferences_ino.getInt("TmHour1OnPr", 0);
          uint32_t timer_minute1_Timer_On = preferences_ino.getInt("TmMin1OnPr", 0);
          hour_add_1_On = hour + timer_hour1_Timer_On;
          minute_add_1_On = minute + timer_minute1_Timer_On;
          if (minute_add_1_On >= 60) {
            minute_add_1_On = minute_add_1_On - 60;
            hour_add_1_On = hour_add_1_On + 1;
          }
          if (hour_add_1_On >= 24) {
            hour_add_1_On = hour_add_1_On % 24;
          }
          Serial.print(F("Time Period On 1: "));
          Serial.print(hour_add_1_On);
          Serial.print(F(":"));
          Serial.println(minute_add_1_On);
          PeriodOn1 = true;
        }
        if ((hour == hour_add_1_On) && (minute == minute_add_1_On) && (PeriodOnExecuted[0] == false) && (answernodelora == false)) {
          Serial.println(F("Period 1 ON"));
          strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "1222");
          byte addh = areaPr_On_1;
          byte addl = IdNumPr_On_1;
          sendCmd(addh, addl);
          lv_obj_add_state(ui_SwitchLIGHTScr2, LV_STATE_CHECKED);
          statusTimer1On_Period = 0;
          preferences_ino.putInt("statusPeriod1On", statusTimer1On_Period);
          PeriodOnExecuted[0] = true;
        }
        if (PeriodOnExecuted[0] == true) {
          PeriodOnExecuted[0] = false;
          PeriodOn1 = false;
        }
      }
      if (statusTimer1Off_Period == 0) {
        uint32_t IdNumPr_Off_1 = preferences_ino.getInt("Id1PrOff", 0);
        uint32_t areaPr_Off_1 = preferences_ino.getInt("area1PrOff", 0);
        if (PeriodOff1 == false) {
          uint32_t timer_hour1_Timer_Off = preferences_ino.getInt("TmHour1OffPr", 0);
          uint32_t timer_minute1_Timer_Off = preferences_ino.getInt("TmMin1OffPr", 0);
          hour_add_1_Off = hour + timer_hour1_Timer_Off;
          minute_add_1_Off = minute + timer_minute1_Timer_Off;
          if (minute_add_1_Off >= 60) {
            minute_add_1_Off = minute_add_1_Off - 60;
            hour_add_1_Off = hour_add_1_Off + 1;
          }
          if (hour_add_1_Off >= 24) {
            hour_add_1_Off = hour_add_1_Off % 24;
          }
          Serial.print(F("Time Period OFF 1: "));
          Serial.print(hour_add_1_Off);
          Serial.print(F(":"));
          Serial.println(minute_add_1_Off);
          PeriodOff1 = true;
        }
        if ((hour == hour_add_1_Off) && (minute == minute_add_1_Off) && (PeriodOffExecuted[0] == false) && (answernodelora == false)) {
          Serial.println(F("Period 1 OFF"));
          strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "0222");
          byte addh = areaPr_Off_1;
          byte addl = IdNumPr_Off_1;
          sendCmd(addh, addl);
          lv_obj_clear_state(ui_SwitchLIGHTScr2, LV_STATE_CHECKED);
          statusTimer1Off_Period = 1;
          preferences_ino.putInt("statusPr1Off", statusTimer1Off_Period);
          PeriodOffExecuted[0] = true;
        }
        if (PeriodOffExecuted[0] == true) {
          PeriodOffExecuted[0] = false;
          PeriodOff1 = false;
        }
      }
    }
  }
  if (timerEn2_Period != 2) {
    if (timerEn2_Period == 0) {
      uint32_t statusTimer2On_Period = preferences_ino.getInt("statusPeriod2On", 2);
      uint32_t statusTimer2Off_Period = preferences_ino.getInt("statusPr2Off", 2);
      static uint8_t hour_add_2_On, minute_add_2_On;
      static uint8_t hour_add_2_Off, minute_add_2_Off;
      static bool PeriodOn2 = false;
      static bool PeriodOff2 = false;
      if (statusTimer2On_Period == 1) {
        uint32_t IdNumPr_On_2 = preferences_ino.getInt("Id2PrOn", 0);
        uint32_t areaPr_On_2 = preferences_ino.getInt("area2PrOn", 0);
        if (PeriodOn2 == false) {
          uint32_t timer_hour2_Timer_On = preferences_ino.getInt("TmHour2OnPr", 0);
          uint32_t timer_minute2_Timer_On = preferences_ino.getInt("TmMin2OnPr", 0);
          hour_add_2_On = hour + timer_hour2_Timer_On;
          minute_add_2_On = minute + timer_minute2_Timer_On;
          if (minute_add_2_On >= 60) {
            minute_add_2_On = minute_add_2_On - 60;
            hour_add_2_On = hour_add_2_On + 1;
          }
          if (hour_add_2_On >= 24) {
            hour_add_2_On = hour_add_2_On % 24;
          }
          Serial.print(F("Time Period On 2: "));
          Serial.print(hour_add_2_On);
          Serial.print(F(":"));
          Serial.println(minute_add_2_On);
          PeriodOn2 = true;
        }
        if ((hour == hour_add_2_On) && (minute == minute_add_2_On) && (PeriodOnExecuted[1] == false) && (answernodelora == false)) {
          Serial.println("Period 2 ON");
          strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "2122");
          byte addh = areaPr_On_2;
          byte addl = IdNumPr_On_2;
          sendCmd(addh, addl);
          lv_obj_add_state(ui_SwitchFANScr2, LV_STATE_CHECKED);
          statusTimer2On_Period = 0;
          preferences_ino.putInt("statusPeriod2On", statusTimer2On_Period);
          PeriodOnExecuted[1] = true;
        }
        if (PeriodOnExecuted[1] == true) {
          PeriodOnExecuted[1] = false;
          PeriodOn2 = false;
        }
      }
      if (statusTimer2Off_Period == 0) {
        uint32_t IdNumPr_Off_2 = preferences_ino.getInt("Id2PrOff", 0);
        uint32_t areaPr_Off_2 = preferences_ino.getInt("area2PrOff", 0);
        if (PeriodOff2 == false) {
          uint32_t timer_hour2_Timer_Off = preferences_ino.getInt("TmHour2OffPr", 0);
          uint32_t timer_minute2_Timer_Off = preferences_ino.getInt("TmMin2OffPr", 0);
          hour_add_2_Off = hour + timer_hour2_Timer_Off;
          minute_add_2_Off = minute + timer_minute2_Timer_Off;
          if (minute_add_2_Off >= 60) {
            minute_add_2_Off = minute_add_2_Off - 60;
            hour_add_2_Off = hour_add_2_Off + 1;
          }
          if (hour_add_2_Off >= 24) {
            hour_add_2_Off = hour_add_2_Off % 24;
          }
          Serial.print(F("Time Period OFF 2: "));
          Serial.print(hour_add_2_Off);
          Serial.print(F(":"));
          Serial.println(minute_add_2_Off);
          PeriodOff2 = true;
        }
        if ((hour == hour_add_2_Off) && (minute == minute_add_2_Off) && (PeriodOffExecuted[1] == false) && (answernodelora == false)) {
          Serial.println(F("Period 2 OFF"));
          strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "2022");
          byte addh = areaPr_Off_2;
          byte addl = IdNumPr_Off_2;
          sendCmd(addh, addl);
          lv_obj_clear_state(ui_SwitchFANScr2, LV_STATE_CHECKED);
          statusTimer2Off_Period = 1;
          preferences_ino.putInt("statusPr2Off", statusTimer2Off_Period);
          PeriodOffExecuted[1] = true;
        }
        if (PeriodOffExecuted[1] == true) {
          PeriodOffExecuted[1] = false;
          PeriodOff2 = false;
        }
      }
    }
  }
  if (timerEn3_Period != 2) {
    if (timerEn3_Period == 0) {
      uint32_t statusTimer3On_Period = preferences_ino.getInt("statusPeriod3On", 2);
      uint32_t statusTimer3Off_Period = preferences_ino.getInt("statusPr3Off", 2);
      static uint8_t hour_add_3_On, minute_add_3_On;
      static uint8_t hour_add_3_Off, minute_add_3_Off;
      static bool PeriodOn3 = false;
      static bool PeriodOff3 = false;
      if (statusTimer3On_Period == 1) {
        uint32_t IdNumPr_On_3 = preferences_ino.getInt("Id3PrOn", 0);
        uint32_t areaPr_On_3 = preferences_ino.getInt("area3PrOn", 0);
        if (PeriodOn3 == false) {
          uint32_t timer_hour3_Timer_On = preferences_ino.getInt("TmHour3OnPr", 0);
          uint32_t timer_minute3_Timer_On = preferences_ino.getInt("TmMin3OnPr", 0);
          hour_add_3_On = hour + timer_hour3_Timer_On;
          minute_add_3_On = minute + timer_minute3_Timer_On;
          if (minute_add_3_On >= 60) {
            minute_add_3_On = minute_add_3_On - 60;
            hour_add_3_On = hour_add_3_On + 1;
          }
          if (hour_add_3_On >= 24) {
            hour_add_3_On = hour_add_3_On % 24;
          }
          Serial.print(F("Time Period On 3: "));
          Serial.print(hour_add_3_On);
          Serial.print(F(":"));
          Serial.println(minute_add_3_On);
          PeriodOn3 = true;
        }
        if ((hour == hour_add_3_On) && (minute == minute_add_3_On) && (PeriodOnExecuted[2] == false) && (answernodelora == false)) {
          Serial.println(F("Period 3 ON"));
          strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "2212");
          byte addh = areaPr_On_3;
          byte addl = IdNumPr_On_3;
          sendCmd(addh, addl);
          lv_obj_add_state(ui_SwitchDEVICE1Scr2, LV_STATE_CHECKED);
          statusTimer3On_Period = 0;
          preferences_ino.putInt("statusPeriod3On", statusTimer3On_Period);
          PeriodOnExecuted[2] = true;
        }
        if (PeriodOnExecuted[2] == true) {
          PeriodOnExecuted[2] = false;
          PeriodOn3 = false;
        }
      }
      if (statusTimer3Off_Period == 0) {
        uint32_t IdNumPr_Off_3 = preferences_ino.getInt("Id3PrOff", 0);
        uint32_t areaPr_Off_3 = preferences_ino.getInt("area3PrOff", 0);
        if (PeriodOff3 == false) {
          uint32_t timer_hour3_Timer_Off = preferences_ino.getInt("TmHour3OffPr", 0);
          uint32_t timer_minute3_Timer_Off = preferences_ino.getInt("TmMin3OffPr", 0);
          hour_add_3_Off = hour + timer_hour3_Timer_Off;
          minute_add_3_Off = minute + timer_minute3_Timer_Off;
          if (minute_add_3_Off >= 60) {
            minute_add_3_Off = minute_add_3_Off - 60;
            hour_add_3_Off = hour_add_3_Off + 1;
          }
          if (hour_add_3_Off >= 24) {
            hour_add_3_Off = hour_add_3_Off % 24;
          }
          Serial.print(F("Time Period OFF 3: "));
          Serial.print(hour_add_3_Off);
          Serial.print(F(":"));
          Serial.println(minute_add_3_Off);
          PeriodOff3 = true;
        }
        if ((hour == hour_add_3_Off) && (minute == minute_add_3_Off) && (PeriodOffExecuted[2] == false) && (answernodelora == false)) {
          Serial.println(F("Period 3 OFF"));
          strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "2202");
          byte addh = areaPr_Off_3;
          byte addl = IdNumPr_Off_3;
          sendCmd(addh, addl);
          lv_obj_clear_state(ui_SwitchDEVICE1Scr2, LV_STATE_CHECKED);
          statusTimer3Off_Period = 1;
          preferences_ino.putInt("statusPr3Off", statusTimer3Off_Period);
          PeriodOffExecuted[2] = true;
        }
        if (PeriodOffExecuted[2] == true) {
          PeriodOffExecuted[2] = false;
          PeriodOff3 = false;
        }
      }
    }
  }
  if (timerEn4_Period != 2) {
    if (timerEn4_Period == 0) {
      uint32_t statusTimer4On_Period = preferences_ino.getInt("statusPeriod4On", 2);
      uint32_t statusTimer4Off_Period = preferences_ino.getInt("statusPr4Off", 2);
      static uint8_t hour_add_4_On, minute_add_4_On;
      static uint8_t hour_add_4_Off, minute_add_4_Off;
      static bool PeriodOn4 = false;
      static bool PeriodOff4 = false;
      if (statusTimer4On_Period == 1) {
        uint32_t IdNumPr_On_4 = preferences_ino.getInt("Id4PrOn", 0);
        uint32_t areaPr_On_4 = preferences_ino.getInt("area4PrOn", 0);
        if (PeriodOn4 == false) {
          uint32_t timer_hour4_Timer_On = preferences_ino.getInt("TmHour4OnPr", 0);
          uint32_t timer_minute4_Timer_On = preferences_ino.getInt("TmMin4OnPr", 0);
          hour_add_4_On = hour + timer_hour4_Timer_On;
          minute_add_4_On = minute + timer_minute4_Timer_On;
          if (minute_add_4_On >= 60) {
            minute_add_4_On = minute_add_4_On - 60;
            hour_add_4_On = hour_add_4_On + 1;
          }
          if (hour_add_4_On >= 24) {
            hour_add_4_On = hour_add_4_On % 24;
          }
          Serial.print(F("Time Period On 4: "));
          Serial.print(hour_add_4_On);
          Serial.print(F(":"));
          Serial.println(minute_add_4_On);
          PeriodOn4 = true;
        }
        if ((hour == hour_add_4_On) && (minute == minute_add_4_On) && (PeriodOnExecuted[3] == false) && (answernodelora == false)) {
          Serial.println(F("Period 4 ON"));
          strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "2221");
          byte addh = areaPr_On_4;
          byte addl = IdNumPr_On_4;
          sendCmd(addh, addl);
          lv_obj_add_state(ui_SwitchDEVICE2Scr2, LV_STATE_CHECKED);
          statusTimer4On_Period = 0;
          preferences_ino.putInt("statusPeriod4On", statusTimer4On_Period);
          PeriodOnExecuted[3] = true;
        }
        if (PeriodOnExecuted[3] == true) {
          PeriodOnExecuted[3] = false;
          PeriodOn4 = false;
        }
      }
      if (statusTimer4Off_Period == 0) {
        uint32_t IdNumPr_Off_4 = preferences_ino.getInt("Id4PrOff", 0);
        uint32_t areaPr_Off_4 = preferences_ino.getInt("area4PrOff", 0);
        if (PeriodOff4 == false) {
          uint32_t timer_hour4_Timer_Off = preferences_ino.getInt("TmHour4OffPr", 0);
          uint32_t timer_minute4_Timer_Off = preferences_ino.getInt("TmMin4OffPr", 0);
          hour_add_4_Off = hour + timer_hour4_Timer_Off;
          minute_add_4_Off = minute + timer_minute4_Timer_Off;
          if (minute_add_4_Off >= 60) {
            minute_add_4_Off = minute_add_4_Off - 60;
            hour_add_4_Off = hour_add_4_Off + 1;
          }
          if (hour_add_4_Off >= 24) {
            hour_add_4_Off = hour_add_4_Off % 24;
          }
          Serial.print(F("Time Period OFF 4: "));
          Serial.print(hour_add_4_Off);
          Serial.print(F(":"));
          Serial.println(minute_add_4_Off);
          PeriodOff4 = true;
        }
        if ((hour == hour_add_4_Off) && (minute == minute_add_4_Off) && (PeriodOffExecuted[3] == false) && (answernodelora == false)) {
          Serial.println(F("Period 4 OFF"));
          strcpy(messctrlGW.ctrrelayGW.OnOffStatus, "2220");
          byte addh = areaPr_Off_4;
          byte addl = IdNumPr_Off_4;
          sendCmd(addh, addl);
          lv_obj_clear_state(ui_SwitchDEVICE2Scr2, LV_STATE_CHECKED);
          statusTimer4Off_Period = 1;
          preferences_ino.putInt("statusPr4Off", statusTimer4Off_Period);
          PeriodOffExecuted[3] = true;
        }
        if (PeriodOffExecuted[3] == true) {
          PeriodOffExecuted[3] = false;
          PeriodOff4 = false;
        }
      }
    }
  }
  preferences_ino.end();
}
void Task_Timer(void *pvParameters) {
  while (1) {
    vTaskDelay(5 / portTICK_PERIOD_MS);
    TakeMutex();
    getInforTimer();
    GiveMutex();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
void loop() {
  vTaskDelete(NULL);
}
