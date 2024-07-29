/*ESP NORMAL*/
#include "Arduino.h"
#include "LoRa_E32.h"
#include "DFRobotDFPlayerMini.h"
#define addh_Node 1
#define addl_Node 2
#define chan_Node 1
const int relayPins[] = { 19, 32, 33, 14 };
bool relayStates[4] = { 0 };
HardwareSerial mySerial(1);
LoRa_E32 e32ttl(&mySerial, 15, 13, 12);
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);
void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);
enum AlertStatus {
  NONE,
  Alert_Temp,
  Alert_PM,
  Alert_Fire,
  Alert_CO2
};
AlertStatus AlertCurrent = NONE;
unsigned long audioPeriod = 0;
unsigned long audioStart = 0;
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
unsigned long previousMillis4 = 0;
unsigned long previousMillis5 = 0;
typedef struct AlertGW {
  byte header1;
  byte addh_gw;
  byte addl_gw;
  byte FAC_NUM;  // STT nhà máy
  byte alert_temp;
  byte alert_pm;
  byte alert_CO2;
  byte alert_Fire;
};
struct AlertGW alertGW;

typedef struct MessAlertGW {
  AlertGW alertGW;
  byte CheckSum[2];
};
struct MessAlertGW messalertGW;

//The setup function is called once at startup of the sketch
typedef struct Check_Control_Block {
  byte header1;
  byte ID_NUM;   //STT KHU
  byte area;     //STT Dây chuyền
  byte FAC_NUM;  //STT nhà máy
  char Check[5];
};
struct Check_Control_Block check_cb;

typedef struct AnswerLoRaCB {
  byte header1 = 0B10000000;
  byte ID_NUM = 0B00000001;   //STT KHU
  byte area = 0B00000010;     //STT Dây chuyền
  byte FAC_NUM = 0B10100001;  //STT nhà máy
  char Answer[8] = "success";
};
struct AnswerLoRaCB answer;

boolean runEvery(unsigned long interval, unsigned long* previousMillis) {
  unsigned long currentMillis = millis();
  if (currentMillis - *previousMillis >= interval) {
    *previousMillis = currentMillis;
    return true;
  }
  return false;
}

/*CheckSum XOR 16 bit*/
uint16_t XORChecksum16(const byte* data, size_t dataLength) {
  uint16_t value = 0;
  for (size_t i = 0; i < dataLength / 2; i++) {
    value ^= data[2 * i] + (data[2 * i + 1] << 8);
  }
  if (dataLength % 2) value ^= data[dataLength - 1];
  return ~value;
}

// Hàm tính toán checksum cho dữ liệu bất kỳ
uint16_t calculateChecksum(const void* data, size_t dataSize) {
  const byte* dataBytes = (const byte*)data;
  return XORChecksum16(dataBytes, dataSize);
}

/*Set config E32*/
void setup_lora(byte& addl, byte& addh, byte& chan) {
  ResponseStructContainer c;
  c = e32ttl.getConfiguration();
  Configuration configuration = *(Configuration*)c.data;
  configuration.ADDL = addl;
  configuration.ADDH = addh;
  configuration.CHAN = chan;
  configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
  configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
  configuration.OPTION.transmissionPower = POWER_20;
  configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;
  configuration.SPED.airDataRate = AIR_DATA_RATE_100_96;
  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.uartParity = MODE_00_8N1;
  ResponseStatus rs = e32ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  Serial.println(rs.getResponseDescription());
  Serial.println(rs.code);
  printParameters(configuration);
  c.close();
}

void Receive(void* pvParameters) {
  (void)pvParameters;
  typedef struct configData {
    byte header1;
    byte addh_gw;
    byte addl_gw;
    byte FAC_NUM;  //STT nhà máy
    byte addl;
    byte addh;
    byte chan;
  };
  struct configData dataconf;

  /*Struct Config*/
  struct MessConf {
    configData dataconf;
    byte CheckSum[2];
  } messconf;

  typedef struct MessAnsCB {
    char Start_Sign[5] = "*ACS";
    AnswerLoRaCB answer;
    byte CheckSum[2];
  };
  struct MessAnsCB messanscb;

  typedef struct MessCheck {
    Check_Control_Block check_cb;
    byte CheckSum[2];
  };
  struct MessCheck messcheck;

  typedef struct controlRelayGW {
    byte header1 = 0B01000011;
    byte addh_gw = 0B00000001;
    byte addl_gw = 0B00000001;
    byte FAC_NUM = 0B10100001;  //STT nhà máy
    char OnOffStatus[5] = "0000";
  };
  struct controlRelayGW ctrrelayGW;
  typedef struct MessCtrlGW {
    controlRelayGW ctrrelayGW;
    byte CheckSum[2];
  };
  struct MessCtrlGW messctrlGW;

  struct PauseAudio {
    byte audio;
  } pauseaudio;

  for (;;) {
    if (e32ttl.available() > 1) {
      char Start_Sign[5];  // first part of structure
      ResponseContainer rs = e32ttl.receiveInitialMessage(sizeof(Start_Sign));
      String typeStr = rs.data;
      Serial.println(typeStr);
      if (typeStr == "*TSC") {
        ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessCheck));
        messcheck = *(MessCheck*)rsc.data;
        // Serial.println(messcheck.check_cb.Check);
        // Serial.println(*(uint16_t*)(messcheck.CheckSum), HEX);
        rsc.close();
        // Serial.println("CRC Answer: ");
        *(uint16_t*)(messanscb.CheckSum) = calculateChecksum(&messanscb.answer, sizeof(messanscb.answer));
        // Serial.println(*(uint16_t*)(messanscb.CheckSum), HEX);
        // Serial.println(F("--------------------------------------"));
        // vTaskDelay(750);
        ResponseStatus rs_ans = e32ttl.sendFixedMessage(addh_Node, addl_Node, chan_Node, &messanscb, sizeof(MessAnsCB));
        Serial.println(rs_ans.getResponseDescription());
      }
      if (typeStr == "*PGN") {
        ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessConf));
        messconf = *(MessConf*)rsc.data;
        // Serial.println(messconf.dataconf.timestamp);
        // Serial.print("addl: ");
        // Serial.println(messconf.dataconf.addl);
        // Serial.print("addh: ");
        // Serial.println(messconf.dataconf.addh);
        // Serial.print("chan: ");
        // Serial.println(messconf.dataconf.chan);
        // Serial.println(*(uint16_t*)(messconf.CheckSum), HEX);
        // Serial.println(F("--------------------------------------"));
        rsc.close();
        setup_lora(messconf.dataconf.addl, messconf.dataconf.addh, messconf.dataconf.chan);
      }
      if (typeStr == "*CGC") {
        ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessCtrlGW));
        messctrlGW = *(MessCtrlGW*)rsc.data;
        rsc.close();
        Serial.print("OnOffStatus: ");
        // Serial.println(messctrlGW.ctrrelayGW.OnOffStatus);
        // Serial.println(*(uint16_t*)(messctrlGW.CheckSum), HEX);
        // Serial.println("CRC calculate: ");
        uint16_t x = calculateChecksum(&messctrlGW.ctrrelayGW, sizeof(messctrlGW.ctrrelayGW));
        // Serial.println(x, HEX);
        // Serial.println(F("--------------------------------------"));
        if (x == (*(uint16_t*)(messctrlGW.CheckSum))) {
          for (int i = 0; i < 4; i++) {
            if (messctrlGW.ctrrelayGW.OnOffStatus[i] == '0') {
              relayStates[i] = 0;                // Relay tắt
              digitalWrite(relayPins[i], HIGH);  // Tắt relay
              Serial.print("Relay OFF ");
              Serial.println(i);
            } else if (messctrlGW.ctrrelayGW.OnOffStatus[i] == '1') {
              relayStates[i] = 1;               // Relay bật
              digitalWrite(relayPins[i], LOW);  // Bật relay
              Serial.print("Relay ON ");
              Serial.println(i);
            } else {
              // Xử lý ngoại lệ nếu cần
            }
          }
        }
      }
      if (typeStr == "*AGC") {
        ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessAlertGW));
        messalertGW = *(MessAlertGW*)rsc.data;
        rsc.close();
        Serial.println(messalertGW.alertGW.alert_temp);
        Serial.println(messalertGW.alertGW.alert_pm);
        Serial.println(messalertGW.alertGW.alert_CO2);
        Serial.println(messalertGW.alertGW.alert_Fire);
      }
      if (typeStr == "*ASC") {
        ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessAlertGW));
        messalertGW = *(MessAlertGW*)rsc.data;
        rsc.close();
        Serial.println(messalertGW.alertGW.alert_Fire);
        *(uint16_t*)(messanscb.CheckSum) = calculateChecksum(&messanscb.answer, sizeof(messanscb.answer));
        ResponseStatus rs_ans = e32ttl.sendFixedMessage(addh_Node, addl_Node, chan_Node, &messanscb, sizeof(MessAnsCB));
        Serial.println(rs_ans.getResponseDescription());
      }
      if (typeStr == "*PGC") {
        ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(PauseAudio));
        pauseaudio = *(PauseAudio*)rsc.data;
        rsc.close();
        Serial.println(pauseaudio.audio);
        myDFPlayer.pause();
      }
    }
  }
  vTaskDelay(10);
}
void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, 26, 27);
  Serial2.begin(9600);
  e32ttl.begin();
  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);  // Đặt chân GPIO là OUTPUT
    digitalWrite(relayPins[i], HIGH);
  }
  unsigned long connectionStartTime = millis();
  if (!myDFPlayer.begin(Serial2, /*isACK = */ true, /*doReset = */ true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
  }
  myDFPlayer.volume(30);
  Serial.println(F("DFPlayer Mini online."));
  Serial.println();
  Serial.println("Start listening!");
  xTaskCreatePinnedToCore(Alert, "Alert", 15000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(Receive, "Receive", 15000, NULL, 2, NULL, 1);
}
void Alert(void* pvParameters) {
  (void)pvParameters;
  static bool statustemp = false;
  static bool statuspm = false;
  static bool statusco2 = false;
  static bool statusfire = false;
  // messalertGW.alertGW.alert_temp = 1;
  // messalertGW.alertGW.alert_pm = 1;
  // messalertGW.alertGW.alert_CO2 = 1;
  // messalertGW.alertGW.alert_Fire = 1;
  while (1) {
    if (AlertCurrent == NONE && runEvery(5000, &previousMillis5)) {
      if ((messalertGW.alertGW.alert_temp == 1) && (messalertGW.alertGW.alert_Fire != 1) && (statustemp == false)) {
        Serial.println("temp");
        myDFPlayer.playMp3Folder(1);
        AlertCurrent = Alert_Temp;
        audioPeriod = 6500;
        audioStart = millis();
        statustemp = true;
      } else if ((messalertGW.alertGW.alert_pm == 1) && (messalertGW.alertGW.alert_Fire != 1) && (statuspm == false)) {
        Serial.println("pm");
        myDFPlayer.playMp3Folder(2);
        AlertCurrent = Alert_PM;
        audioPeriod = 6500;
        audioStart = millis();
        statuspm = true;
      } else if ((messalertGW.alertGW.alert_CO2 == 1) && (messalertGW.alertGW.alert_Fire != 1) && (statusco2 == false)) {
        Serial.println("co2");
        myDFPlayer.playMp3Folder(4);
        AlertCurrent = Alert_CO2;
        audioPeriod = 7000;
        audioStart = millis();
        statusco2 = true;
      } else if ((messalertGW.alertGW.alert_Fire == 1) && (statusfire == false)) {
        Serial.println("fire");
        myDFPlayer.playMp3Folder(3);
        AlertCurrent = Alert_Fire;
        audioPeriod = 22000;
        audioStart = millis();
        statusfire = true;
      }
    }
    if (AlertCurrent != NONE) {
      if (millis() - audioStart >= audioPeriod) {
        AlertCurrent = NONE;
      }
    }
    if ((statustemp == true) && runEvery(23000, &previousMillis1)) {
      statustemp = false;
    }
    if ((statuspm == true) && runEvery(22000, &previousMillis2)) {
      statuspm = false;
    }
    if ((statusco2 == true) && runEvery(22000, &previousMillis3)) {
      statusco2 = false;
    }
    if ((statusfire == true) && runEvery(1000, &previousMillis4)) {
      statusfire = false;
    }
    vTaskDelay(10);
  }
}

// The loop function is called in an endless loop
void loop() {
  vTaskDelete(NULL);
  // vTaskDelay(100);
}

void printParameters(struct Configuration configuration) {
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD : "));
  Serial.print(configuration.HEAD, BIN);
  Serial.print(" ");
  Serial.print(configuration.HEAD, DEC);
  Serial.print(" ");
  Serial.println(configuration.HEAD, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));
  Serial.println(configuration.ADDH, DEC);
  Serial.print(F("AddL : "));
  Serial.println(configuration.ADDL, DEC);
  Serial.print(F("Chan : "));
  Serial.print(configuration.CHAN, DEC);
  Serial.print(" -> ");
  Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));
  Serial.print(configuration.SPED.uartParity, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte  : "));
  Serial.print(configuration.SPED.uartBaudRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTBaudRate());
  Serial.print(F("SpeedAirDataRate   : "));
  Serial.print(configuration.SPED.airDataRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getAirDataRate());

  Serial.print(F("OptionTrans        : "));
  Serial.print(configuration.OPTION.fixedTransmission, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getFixedTransmissionDescription());
  Serial.print(F("OptionPullup       : "));
  Serial.print(configuration.OPTION.ioDriveMode, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getIODroveModeDescription());
  Serial.print(F("OptionWakeup       : "));
  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
  Serial.print(F("OptionFEC          : "));
  Serial.print(configuration.OPTION.fec, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getFECDescription());
  Serial.print(F("OptionPower        : "));
  Serial.print(configuration.OPTION.transmissionPower, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getTransmissionPowerDescription());

  Serial.println("----------------------------------------");
}
void printModuleInformation(struct ModuleInformation moduleInformation) {
  Serial.println("----------------------------------------");
  Serial.print(F("HEAD BIN: "));
  Serial.print(moduleInformation.HEAD, BIN);
  Serial.print(" ");
  Serial.print(moduleInformation.HEAD, DEC);
  Serial.print(" ");
  Serial.println(moduleInformation.HEAD, HEX);

  Serial.print(F("Freq.: "));
  Serial.println(moduleInformation.frequency, HEX);
  Serial.print(F("Version  : "));
  Serial.println(moduleInformation.version, HEX);
  Serial.print(F("Features : "));
  Serial.println(moduleInformation.features, HEX);
  Serial.println("----------------------------------------");
}
void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}