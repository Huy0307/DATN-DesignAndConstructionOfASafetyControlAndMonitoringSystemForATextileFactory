#include "alert.h"
void alert_temp(float &t, byte &danger_temp) {
  int temp_thres = 50;
  if (t >= temp_thres) {
    Serial.println("ALERT TEMP: ");
    Serial.println(t);
    Serial.println("Sent to On FAN/AC Temp");
    danger_temp = 1;
  }
  if (t <= 25) {
    Serial.println(t);
    Serial.println("Sent to Off FAN/AC Temp");
    danger_temp = 2;
  }
}
void alert_humi(float &h, byte &danger_humi) {
  float humi_thres = 85;
  if (h >= humi_thres) {
    Serial.println("ALERT Humi: ");
    Serial.println(h);
    Serial.println("Sent to On FAN Humi");
    danger_humi = 1;
  }
  if (h <= 75) {
    Serial.println(h);
    Serial.println("Sent to Off FAN Humi");
    danger_humi = 2;
  }
}
void alert_pm(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10, byte &danger_pm) {
  int pm10_thres = 155;
  int pm25_thres = 36;
  int pm1_thres = 36;
  if ((pm1 >= pm1_thres) || (pm25 >= pm25_thres) || (pm10 >= pm10_thres)) {
    Serial.println("ALERT PM: ");
    Serial.println(pm1);
    Serial.println(pm25);
    Serial.println(pm10);
    Serial.println("Sent to On FAN");
    danger_pm = 1;
  }
  if ((pm10 <= 100) && (pm25 <= 25) && (pm1 <= 25)) {
    Serial.println(pm1);
    Serial.println(pm25);
    Serial.println(pm10);
    Serial.println("Sent to Off FAN");
    danger_pm = 2;
  }
}
void alert_light(uint16_t &lux, byte &danger_lux, bool &lightOn) {
  int lux_thres_low = 350;  //Thấp hơn 300 (tối thiểu 300)
  int lux_thres_high = 650;
  if ((lux <= lux_thres_low) && (lightOn == false)) {
    Serial.print("Alert Light: ");
    Serial.println(lux);
    Serial.println("Turn on Light");
    danger_lux = 1;
    lightOn = true;
  }
  if ((lux >= lux_thres_high) && (lightOn == true)) {
    Serial.println("Turn off some Light");
    danger_lux = 2;
    lightOn = false;
  }
}
void alert_CO_CO2(uint16_t &CO, uint16_t &CO2, byte &danger_CO_CO2) {
  int CO_thres = 50;
  int CO2_thres = 1000;
  if ((CO >= CO_thres) || (CO2 >= CO2_thres)) {
    if (CO >= CO_thres) {
      Serial.print("Alert CO: ");
      Serial.println(CO);
    }
    if (CO2 >= CO2_thres) {
      Serial.print("Alert CO2: ");
      Serial.println(CO2);
    }
    danger_CO_CO2 = 1;
  }
  if (((CO <= (CO_thres - 30)) || (CO2 <= (CO2_thres - 500))) && (danger_CO_CO2 == 1)) {
    if (CO <= (CO_thres - 30)) {
      Serial.println("CO Safe");
    }
    if (CO2 <= (CO2_thres - 500)) {
      Serial.print("CO2 Safe");
    }
    danger_CO_CO2 = 2;
  }
}
bool alert(float &t, float &h, uint16_t &pm1, uint16_t &pm25, uint16_t &pm10, uint16_t &lux, uint16_t &CO, uint16_t &CO2) {
  int temp_thres = 50;
  float humi_thres = 85;
  int pm10_thres = 155;
  int pm25_thres = 36;
  int pm1_thres = 36;
  int lux_thres_low = 350;  //Thấp hơn 300 (tối thiểu 300)
  int lux_thres_high = 650;
  int CO_thres = 50;
  int CO2_thres = 1000;
  bool result;
  return ((t >= temp_thres) || (h >= humi_thres) || (pm1 >= pm1_thres) || (pm25 >= pm25_thres) || (pm10 >= pm10_thres) || (lux <= lux_thres_low) || (lux >= lux_thres_high) || (CO >= CO_thres) || (CO2 >= CO2_thres));
}