#include "LoRa_E32.h"

/*Define LoRa*/
LoRa_E32 e32ttl(&Serial2, 15, 12, 13);  //[16(RX), 17(TX)]

#include "declare_nodesensor.h"
#include "function.h"
#include "alert.h"
#include "SaveData.h"

#define FORMAT_LITTLEFS_IF_FAILED true

void printParameters(struct Configuration configuration);

byte setBits(byte value, byte* bitPositions, byte numBits) {
  for (byte i = 0; i < numBits; i++) {
    byte pos = bitPositions[i];
    value |= (1 << pos);
  }
  return value;
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

boolean runEvery(unsigned long interval, unsigned long* previousMillis) {
  unsigned long currentMillis = millis();
  if (currentMillis - *previousMillis >= interval) {
    *previousMillis = currentMillis;
    return true;
  }
  return false;
}

/*Receive Config*/
void Receive_Data(void* pvParameters) {
  (void)pvParameters;
  typedef struct configData {
    byte header1;
    byte addh_gw;
    byte addl_gw;
    byte FAC_NUM;  //STT nhà máy
    byte addl;
    byte addh;
    byte chan;
    char timestamp[15];
  };
  struct configData dataconf;

  typedef struct AnswerLoRa {
    byte header1;
    byte addh_gw;
    byte addl_gw;
    byte FAC_NUM;  //STT nhà máy
    char Answer[8];
  };
  struct AnswerLoRa answer;

  typedef struct AnswerLoRaCB {
    byte header1;
    byte ID_NUM;   //STT KHU
    byte area;     //STT Dây chuyền
    byte FAC_NUM;  //STT nhà máy
    char Answer[8];
  };
  struct AnswerLoRaCB answercb;

  typedef struct Request {
    byte header1;
    byte addh_gw;
    byte addl_gw;
    byte FAC_NUM;  //STT nhà máy
    byte req;
  };
  struct Request request;

  /*Struct Ans*/
  struct MessAns {
    AnswerLoRa answer;
    byte CheckSum[2];
  } messans;

  /*Struct Config*/
  struct MessConf {
    configData dataconf;
    byte CheckSum[2];
  } messconf;

  /*Struct Ans CB*/
  struct MessAnsCB {
    AnswerLoRaCB answercb;
    byte CheckSum[2];
  } messanscb;

  struct MessRequest {
    Request request;
    byte CheckSum[2];
  } messrequest;
  String typeStr = "";
  static uint16_t old_CheckSumAnswer;
  for (;;) {
    if (e32ttl.available() > 1) {
      char Start_Sign[5];
      ResponseContainer rs = e32ttl.receiveInitialMessage(sizeof(Start_Sign));
      String typeStr = rs.data;
      if (typeStr == "*PGN") {
        ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessConf));
        messconf = *(MessConf*)rsc.data;
        Serial.println(*(uint16_t*)(messconf.CheckSum), HEX);
        Serial.println(F("--------------------------------------"));
        rsc.close();
        if ((messconf.dataconf.addh_gw == addh_GW_FAC) && (messconf.dataconf.addl_gw == addl_GW_FAC)) {
          setup_lora(messconf.dataconf.addl, messconf.dataconf.addh, messconf.dataconf.chan);
          retrycountdn = 0;
          GW = 0;
        }
      }
      if (typeStr == "*AGS") {
        ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessAns));
        messans = *(MessAns*)rsc.data;
        Serial.println(*(uint16_t*)(messans.CheckSum), HEX);
        uint16_t x = calculateChecksum(&messans.answer, sizeof(messans.answer));
        Serial.print("CRC after Receive: ");
        Serial.println(x, HEX);
        rsc.close();
        if ((messans.answer.addh_gw == addh_GW_FAC) && (messans.answer.addl_gw == addl_GW_FAC)) {
          dn = false;
          retrycountdn = 0;
          GW = 0;
        }
        Serial.println(F("--------------------------------------"));
      }
      if (typeStr == "*ACS") {
        ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(messanscb));
        Serial.println("CB Answer");
        messanscb = *(MessAnsCB*)rsc.data;
        Serial.println(*(uint16_t*)(messanscb.CheckSum), HEX);
        uint16_t x1 = calculateChecksum(&messanscb.answercb, sizeof(messanscb.answercb));
        Serial.print("CRC after receive CB: ");
        Serial.println(x1, HEX);
        rsc.close();
        retrycountcn = 0;
        cn = false;
        CB = 0;
        Serial.println(F("--------------------------------------"));
      }
      if (typeStr == "*RGS") {
        ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(messrequest));
        messrequest = *(MessRequest*)rsc.data;
        Serial.println(*(uint16_t*)(messrequest.CheckSum), HEX);
        uint16_t x1 = calculateChecksum(&messrequest.request, sizeof(messrequest.request));
        Serial.print("CRC after receive CB: ");
        Serial.println(x1, HEX);
        rsc.close();
        if ((messrequest.request.addh_gw == addh_GW_FAC) && (messrequest.request.addl_gw == addl_GW_FAC)) {
          xTaskCreatePinnedToCore(Send_Data, "Send_Data", 15000, NULL, 4, &Task_Send_Data, 0);
        }
      }
    }
    if (datasent == true) {
      if ((dn || cn) && (millis() - lastDataSentTime >= 5000)) {
        if (alert1 && dn && (cn == false)) {
          Serial.println("ALERT!!!");
          retrycountdn = 3;
          dn = false;
        }
        if ((retrycountdn < 3) && (dn == true)) {
          retrycountdn++;
          Serial.print("RetryCountDN: ");
          Serial.println(retrycountdn);
          dn = false;
        }
        if ((retrycountcn < 3) && (cn == true)) {
          retrycountcn++;
          Serial.print("RetryCountCN: ");
          Serial.println(retrycountcn);
          cn = false;
        }
        if ((retrycountdn >= 3) && (br == true)) {
          GW = 1;
          xTaskCreatePinnedToCore(AutoDetectAndControl, "AutoDetectAndControl", 10000, NULL, 4, &Task_AutoDetectControl, 1);
        }
        if ((retrycountcn >= 3) && (br_c == true)) {
          CB = 1;
          Serial.println("CB lost connect");
          cn = false;
          br_c = false;
        }
        datasent = false;
        count++;
      }
    }
  }
}

void ReadData(void* pvParameters) {
  (void)pvParameters;
  while (1) {
    /*Data to GW Parameters*/
    // now = rtc.now();
    // char buf[] = "YY/MM/DD-hh:mm";
    // now.toString(buf);
    // strcpy(messDS.datasensor.timestamp, buf);
    float t = 0.0;
    float h = 0.0;
    uint16_t lux = 0;
    uint16_t pm1 = 0;
    uint16_t pm2_5 = 0;
    uint16_t pm10 = 0;
    uint16_t co = 0;
    uint16_t co2 = 0;
    uint16_t smoke = 0;
    /*SHT31*/
    // t = sht31.readTemperature();
    // h = sht31.readHumidity();
    if (count == 0) {
      t = 20;
      h = 65;
    } else if (count == 1) {
      t = 40;
      h = 50;
    } else if (count == 2) {
      t = 20;
      h = 80;
    } else if (count == 3) {
      t = 20;
      h = 40;
    } else if (count == 4) {
      t = 20;
      h = 40;
    } else if (count > 4) {
      count = 0;
    }
    /*PMS7003*/
    pm1 = 25;
    pm2_5 = 24;
    pm10 = 98;
    co = 42;
    co2 = 500;

    /*BH1750*/
    lux = 500;

    xQueueOverwrite(xQueueTemp, &t);
    xQueueOverwrite(xQueueHumi, &h);
    xQueueOverwrite(xQueuePM1, &pm1);
    xQueueOverwrite(xQueuePM25, &pm2_5);
    xQueueOverwrite(xQueuePM10, &pm10);
    xQueueOverwrite(xQueueLight, &lux);
    xQueueOverwrite(xQueueCO, &co);
    xQueueOverwrite(xQueueCO2, &co2);
    xQueueOverwrite(xQueueSmoke, &smoke);
    alert1 = alert(t, h, pm1, pm2_5, pm10, lux, co, co2);
    if (alert1) {
      Serial.println("ALERT1");
      // xTaskCreatePinnedToCore(Send_Data, "Send_Data", 15000, NULL, 4, &Task_Send_Data, 0);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

/*ReadSensor*/
void Send_Data(void* pvParameters) {
  (void)pvParameters;
  /*Data to GW Parameters*/
  float t = 0.0;
  float h = 0.0;
  uint16_t lux = 0;
  uint16_t pm1 = 0;
  uint16_t pm2_5 = 0;
  uint16_t pm10 = 0;
  uint16_t co = 0;
  uint16_t co2 = 0;
  uint16_t smoke = 0;
  if ((xQueueReceive(xQueueHumi, &h, portMAX_DELAY) == pdPASS) && (xQueueReceive(xQueueTemp, &t, portMAX_DELAY) == pdPASS)
      && (xQueueReceive(xQueuePM1, &pm1, portMAX_DELAY) == pdPASS) && (xQueueReceive(xQueueLight, &lux, portMAX_DELAY) == pdPASS)
      && (xQueueReceive(xQueuePM10, &pm10, portMAX_DELAY) == pdPASS) && (xQueueReceive(xQueuePM25, &pm2_5, portMAX_DELAY) == pdPASS)
      && (xQueueReceive(xQueueCO, &co, portMAX_DELAY) == pdPASS) && (xQueueReceive(xQueueCO2, &co2, portMAX_DELAY) == pdPASS)
      && (xQueueReceive(xQueueSmoke, &smoke, portMAX_DELAY) == pdPASS)) {
    *(float*)(messDS.datasensor.t) = t;
    *(float*)(messDS.datasensor.h) = h;
    *(uint16_t*)(messDS.datasensor.lux) = lux;
    *(uint16_t*)(messDS.datasensor.pm1) = pm1;
    *(uint16_t*)(messDS.datasensor.pm2_5) = pm2_5;
    *(uint16_t*)(messDS.datasensor.pm10) = pm10;
    *(uint16_t*)(messDS.datasensor.CO) = co;
    *(uint16_t*)(messDS.datasensor.CO2) = co2;
    if ((smoke > 650) && (t > 60)) {
      strcpy(messDS.datasensor.Fire,"Y");
    } else {
      strcpy(messDS.datasensor.Fire,"N");
    }
    Serial.print("temp trans:");
    Serial.println(t);
    Serial.print("humi trans:");
    Serial.println(h);
    *(uint16_t*)(messDS.CheckSum) = calculateChecksum(&messDS.datasensor, sizeof(messDS.datasensor));
    Serial.println(*(uint16_t*)(messDS.CheckSum), HEX);

    /*Block Control Parameters*/
    *(uint16_t*)(messCB.CheckSum) = calculateChecksum(&messCB.check_cb, sizeof(messCB.check_cb));
    Serial.println(*(uint16_t*)(messCB.CheckSum), HEX);

    /*Save Data to Flash*/
    appendFile(LittleFS, "/data.csv", (String(*(float*)(messDS.datasensor.t)) + ",").c_str());
    appendFile(LittleFS, "/data.csv", (String(*(float*)(messDS.datasensor.h)) + ",").c_str());
    appendFile(LittleFS, "/data.csv", (String(*(uint16_t*)(messDS.datasensor.pm1)) + ",").c_str());
    appendFile(LittleFS, "/data.csv", (String(*(uint16_t*)(messDS.datasensor.pm2_5)) + ",").c_str());
    appendFile(LittleFS, "/data.csv", (String(*(uint16_t*)(messDS.datasensor.pm10)) + ",").c_str());
    appendFile(LittleFS, "/data.csv", (String(*(uint16_t*)(messDS.datasensor.lux)) + ",").c_str());
    appendFile(LittleFS, "/data.csv", (String(*(uint16_t*)(messDS.datasensor.CO)) + ",").c_str());
    appendFile(LittleFS, "/data.csv", (String(*(uint16_t*)(messDS.datasensor.CO2)) + "\r\n").c_str());
  }
  if ((CB == 1) && (GW == 1)) {
    Serial.println("ALERT");
  } else {
    /*Send Data*/
    vTaskDelay(500);
    ResponseStatus rs = e32ttl.sendFixedMessage(addh_GW_FAC, addl_GW_FAC, chan_GW_FAC, &messDS, sizeof(MessDS));
    Serial.println(rs.getResponseDescription());
    /*End Send Data*/

    /*Check Block Control*/
    vTaskDelay(500);
    ResponseStatus rs_block = e32ttl.sendFixedMessage(addh_Node, addl_Node, chan_Node, &messCB, sizeof(MessCB));
    Serial.println(rs_block.getResponseDescription());
    /*End Check Block Control*/

    dn = true;
    cn = true;
    br = true;
    br_c = true;
    datasent = true;
    lastDataSentTime = millis();
  }
  vTaskDelete(NULL);
}

void sendControlCmd(void* pvParameters) {
  (void)pvParameters;
  *(uint16_t*)(messctrl.CheckSum) = calculateChecksum(&messctrl.ctrrelay, sizeof(messctrl.ctrrelay));
  Serial.println(*(uint16_t*)(messctrl.CheckSum), HEX);
  byte addh_node = addh_Node;
  byte addl_node = addl_Node;
  byte chan_node = chan_Node;
  vTaskDelay(500);
  ResponseStatus rs = e32ttl.sendFixedMessage(addh_node, addl_node, chan_node, &messctrl, sizeof(messctrl));
  Serial.println(rs.getResponseDescription());
  Serial.println(F("--------------------------------------"));
  vTaskDelete(NULL);
}

/*Auto Detect Dangerous and Control*/
void AutoDetectAndControl(void* pvParameters) {
  (void)pvParameters;
  Serial.println("AUTO");
  alert_temp(*(float*)(messDS.datasensor.t), danger_temp);
  alert_humi(*(float*)(messDS.datasensor.h), danger_humi);
  alert_pm(*(uint16_t*)(messDS.datasensor.pm1), *(uint16_t*)(messDS.datasensor.pm2_5), *(uint16_t*)(messDS.datasensor.pm10), danger_pm);
  alert_light(*(uint16_t*)(messDS.datasensor.lux), danger_lux, lightOn);
  alert_CO_CO2(*(uint16_t*)(messDS.datasensor.CO), *(uint16_t*)(messDS.datasensor.CO2), danger_CO_CO2);
  if (((danger_temp == 1) || (danger_humi == 1) || (danger_pm == 1)) && (fanOn == false)) {
    //Send command turn on fan
    messctrl.ctrrelay.NameDevice = 1;
    messctrl.ctrrelay.region = 1;
    messctrl.ctrrelay.status = 1;
    vTaskDelay(500);
    xTaskCreatePinnedToCore(sendControlCmd, "sendControlCmd", 15000, NULL, 4, NULL, 0);
    fanOn = true;
  }
  if (((danger_temp == 2) && (danger_humi == 2) && (danger_pm == 2)) && (fanOn == true)) {
    //Send command turn off fan
    messctrl.ctrrelay.NameDevice = 1;
    messctrl.ctrrelay.region = 1;
    messctrl.ctrrelay.status = 0;
    vTaskDelay(500);
    xTaskCreatePinnedToCore(sendControlCmd, "sendControlCmd", 15000, NULL, 4, NULL, 0);
    fanOn = false;
  }
  if (danger_lux == 1) {
    //Send command turn on light
    messctrl.ctrrelay.NameDevice = 1;
    messctrl.ctrrelay.region = 2;
    messctrl.ctrrelay.status = 1;
    xTaskCreatePinnedToCore(sendControlCmd, "sendControlCmd", 15000, NULL, 4, NULL, 0);
    danger_lux = 0;
  }
  if (danger_lux == 2) {
    //Send command turn off light
    messctrl.ctrrelay.NameDevice = 1;
    messctrl.ctrrelay.region = 2;
    messctrl.ctrrelay.status = 0;
    xTaskCreatePinnedToCore(sendControlCmd, "sendControlCmd", 15000, NULL, 4, NULL, 0);
    danger_lux = 0;
  }
  br = false;
  dn = false;
  vTaskDelete(Task_AutoDetectControl);
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void deep_sleep() {
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
  Serial.flush();
  esp_deep_sleep_start();
}

void setup_node() {
  Serial.begin(115200);
  e32ttl.begin();
  if (!sht31.begin(0x44)) {  // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
    Serial.println("LittleFS Mount Failed");
    return;
  } else {
    Serial.println("Little FS Mounted Successfully");
  }
  // if (!rtc.begin()) {
  //   Serial.println("Couldn't find RTC");
  //   Serial.flush();
  //   while (1) delay(10);
  // }
  // if (!rtc.isrunning()) {
  //   Serial.println("RTC is NOT running, let's set the time!");
  //   // When time needs to be set on a new device, or after a power loss, the
  //   // following line sets the RTC to the date & time this sketch was compiled
  //   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //   // This line sets the RTC with an explicit date & time, for example to set
  //   // January 21, 2014 at 3am you would call:
  //   // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  // }
  writeFile(LittleFS, "/data.csv", "Temp, Humi, PM1, PM25, PM10, Light, CO, CO2\r\n");
  xQueueTemp = xQueueCreate(1, sizeof(float));
  xQueueHumi = xQueueCreate(1, sizeof(float));
  xQueuePM1 = xQueueCreate(1, sizeof(unsigned int));
  xQueuePM25 = xQueueCreate(1, sizeof(unsigned int));
  xQueuePM10 = xQueueCreate(1, sizeof(unsigned int));
  xQueueLight = xQueueCreate(1, sizeof(float));
  xQueueCO = xQueueCreate(1, sizeof(unsigned int));
  xQueueCO2 = xQueueCreate(1, sizeof(unsigned int));
  xQueueSmoke = xQueueCreate(1, sizeof(unsigned int));
  xBinarySemaphore = xSemaphoreCreateBinary();
  xMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(ReadData, "ReadData", 15000, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(Receive_Data, "Receive_Data", 15000, NULL, 3, &Task_Receive_Data, 1);
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
  Serial.println(configuration.ADDH, BIN);
  Serial.print(F("AddL : "));
  Serial.println(configuration.ADDL, BIN);
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
