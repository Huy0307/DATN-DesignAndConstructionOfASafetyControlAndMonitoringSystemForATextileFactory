#include "freertos/portmacro.h"
#include "FreeRTOSConfig.h"
#include "HardwareSerial.h"
#include "freertos/projdefs.h"
#include "freertos/portmacro.h"
#include "Adafruit_SHT31.h"
#include <Wire.h>
#include <RTClib.h>
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 10          /* Time ESP32 will go to sleep (in seconds) */
#define FAC A1
#define addh_GW_FAC 1
#define addl_GW_FAC 1
#define chan_GW_FAC 0
#define addh_Node 1
#define addl_Node 2
#define chan_Node 1
Adafruit_SHT31 sht31 = Adafruit_SHT31();
RTC_DS1307 rtc;
DateTime now;
/*Semaphore*/
SemaphoreHandle_t xBinarySemaphore;
SemaphoreHandle_t xBinarySemaphore2;
SemaphoreHandle_t xMutex;
portMUX_TYPE timerMUX = portMUX_INITIALIZER_UNLOCKED;

/*Queue*/
QueueHandle_t xQueueTemp, xQueueHumi, xQueuePM1, xQueuePM25, xQueuePM10, xQueueLight, xQueueCO, xQueueCO2, xQueueSmoke;

/*Sensor*/
RTC_DATA_ATTR int bootCount = 0;
uint8_t loopCnt = 0;

void Send_Data(void *pvParameters);
void TimeSend(void *pvParameters);
void Receive_Data(void *pvParameters);
void AutoDetectAndControl(void *pvParameters);
void CheckBlockControl(void *pvParameters);

TickType_t xLastWakeTime;
const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
bool receivedAnswer = true;
TaskHandle_t Task_Send_Data = NULL;
TaskHandle_t Task_Receive_Data = NULL;
TaskHandle_t Task_AutoDetectControl = NULL;
TaskHandle_t Task_CheckControlBlock = NULL;
bool check = false;
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis = 0;
unsigned long interval = 5000;
bool dn = false;
bool cn = false;
byte retrycountdn = 0;
byte retrycountcn = 0;
byte GW = 0;
byte CB = 0;
bool br = false;
bool br_c = false;
bool datasent = false;
unsigned long lastDataSentTime = 0;
byte count = 0;
typedef struct DataSensor {
  byte header1 = 0B11011111;
  byte ID_NUM = 0B00000001;   //STT Dây chuyền (NODE)
  byte area = 0B00000010;     //STT KHU
  byte FAC_NUM = 0B10100001;  //STT nhà máy
  byte t[4];
  byte h[4];
  byte pm1[2];
  byte pm2_5[2];
  byte pm10[2];
  byte lux[2];
  byte CO[2];
  byte CO2[2];
  char Fire[2] = "Y";
  char timestamp[15] = "YY/MM/DD-hh:mm";
};

typedef struct Check_Control_Block {
  byte header1 = 0B11000010;
  byte ID_NUM = 0B00000001;   //STT KHU
  byte area = 0B00000010;     //STT Dây chuyền
  byte FAC_NUM = 0B10100001;  //STT nhà máy
  char Check[5] = "How?";
};

typedef struct controlRelay {
  byte header1 = 0B01000011;
  byte ID_NUM = 0B00000001;   //STT KHU
  byte area = 0B00000011;     //STT Dây chuyền
  byte FAC_NUM = 0B10100001;  //STT nhà máy
  byte NameDevice;
  byte region;  //vùng chẵn lẻ của đèn, quạt
  byte status;  //0: OFF, 1: ON
  char timestamp[15] = "18:28-17/03/24";
};

struct controlRelay ctrrelay;
struct DataSensor datasensor;
struct Check_Control_Block check_cb;

typedef struct MessDS {
  char Start_Sign[5] = "*DNG";
  DataSensor datasensor;
  byte CheckSum[2];
};
struct MessDS messDS;

typedef struct MessCB {
  char Start_Sign[5] = "*TNN";
  Check_Control_Block check_cb;
  byte CheckSum[2];
};
struct MessCB messCB;

typedef struct MessCtrl {
  char Start_Sign[5] = "*CNN";
  controlRelay ctrrelay;
  byte CheckSum[2];
};
struct MessCtrl messctrl;

bool fanOn = false;
bool lightOn = false;
byte danger_temp = 0;
byte danger_humi = 0;
byte danger_pm = 0;
byte danger_lux = 0;
byte danger_CO_CO2 = 0;
byte Fire = 0;
bool alert1;