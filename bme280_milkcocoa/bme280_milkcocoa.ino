#include <ESP8266WiFi.h>
#include <Milkcocoa.h>

/************************* BM280 hensu *********************************/
const uint8_t sclk = 14;
const uint8_t mosi =13; //Master Output Slave Input ESP8266=Master,BME280=slave 
const uint8_t miso =12; //Master Input Slave Output
const uint8_t cs_bme = 16; //CS pin
 
uint32_t hum_raw, temp_raw, pres_raw;
 int32_t t_fine;
 
uint16_t dig_T1;
 int16_t dig_T2;
 int16_t dig_T3;
  
uint16_t dig_P1;
 int16_t dig_P2;
 int16_t dig_P3;
 int16_t dig_P4;
 int16_t dig_P5;
 int16_t dig_P6;
 int16_t dig_P7;
 int16_t dig_P8;
 int16_t dig_P9;
  
uint8_t  dig_H1;
 int16_t dig_H2;
uint8_t  dig_H3;
 int16_t dig_H4;
 int16_t dig_H5;
 int8_t  dig_H6;
 
double SeaLevelPressure_hPa = 1009.4; //標準は1013.25


/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "ssclabo-wifi-G"
#define WLAN_PASS       "ThinApp12345"


/************************* Your Milkcocoa Setup *********************************/

#define MILKCOCOA_APP_ID      "dogj1yekfma"
#define MILKCOCOA_DATASTORE   "bme280esp8266"

/************* Milkcocoa Setup (you don't need to change this!) ******************/

#define MILKCOCOA_SERVERPORT  1883

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

const char MQTT_SERVER[] PROGMEM    = MILKCOCOA_APP_ID ".mlkcca.com";
const char MQTT_CLIENTID[] PROGMEM  = __TIME__ MILKCOCOA_APP_ID;

/************ Milkcocoa MQTT Set ******************/
Milkcocoa milkcocoa = Milkcocoa(&client, MQTT_SERVER, MILKCOCOA_SERVERPORT, MILKCOCOA_APP_ID, MQTT_CLIENTID);

void onpush(DataElement *elem) {
  Serial.println("onpush");
  Serial.println(elem->getInt("v"));
};

void setupWiFi() {
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


void setup() {
  /************ BME280 ******************/
  uint8_t t_sb = 5; //stanby 1000ms
  uint8_t filter = 0; //filter O = off
  uint8_t spi3or4 = 0; //SPI 3wire or 4wire, 0=4wire, 1=3wire
  uint8_t osrs_t = 4; //OverSampling Temperature x4
  uint8_t osrs_p = 4; //OverSampling Pressure x4
  uint8_t osrs_h = 4; //OverSampling Humidity x4
  uint8_t Mode = 3; //Normal mode
 
  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | Mode;
  uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3or4;
  uint8_t ctrl_hum_reg  = osrs_h;


  /************ Milkcocoa ******************/
  Serial.begin(115200);
  delay(10);
  Serial.println(F("BME280 to Milkcocoa Start"));

  setupWiFi();

  Serial.println( milkcocoa.on(MILKCOCOA_DATASTORE, "push", onpush) );


  /************ BME280 ******************/
  pinMode(sclk, OUTPUT);
  pinMode(miso, INPUT);
  pinMode(mosi, OUTPUT);
  pinMode(cs_bme, OUTPUT);
 
  digitalWrite(cs_bme, HIGH);
  
  writeReg(0xF2,ctrl_hum_reg);
  writeReg(0xF4,ctrl_meas_reg);
  writeReg(0xF5,config_reg);
  delay(1000);
   
  readTrim();
  Serial.println();

};

void loop() {

  /************ BME280 ******************/
  double temp_act = 0.0, press_act = 0.0, hum_act = 0.0, altitude_act = 0.0;
  int32_t temp_cal;
  uint32_t press_cal,hum_cal;


  // 以下をloopの中で必ず実行
  milkcocoa.loop();

  /************ BME280 ******************/  
  readData();
 
  temp_cal = calibration_T(temp_raw);
  press_cal = calibration_P(pres_raw);
  hum_cal = calibration_H(hum_raw);
  temp_act = (double)temp_cal / 100.0;
  press_act = (double)press_cal / 100.0;
  hum_act = (double)hum_cal / 1024.0;
  altitude_act = ReadAltitude(SeaLevelPressure_hPa, press_act);
   
  Serial.println("-----------------------");
  Serial.print("Temperature = "); Serial.print(temp_act); Serial.println(" *C");
  Serial.print("Humidity = "); Serial.print(hum_act); Serial.println(" %");
  Serial.print("Pressure = "); Serial.print(press_act); Serial.println(" hPa");
  Serial.print("Altitude = "); Serial.print(altitude_act); Serial.println(" m");


  // データ格納用のオブジェクト
  DataElement elem = DataElement();
  elem.setValue("TEMP", temp_act);
  elem.setValue("HUM", hum_act);  
  elem.setValue("PRESS", press_act);

  milkcocoa.push(MILKCOCOA_DATASTORE, &elem);
  // 10秒間隔
  delay(7000);

 
  // delay(2000);
  
};

//*****************初期値読み込み**************************************
void readTrim(void) {
    dig_T1 = read16bit(0x88);
    dig_T2 = (int16_t)read16bit(0x8A);
    dig_T3 = (int16_t)read16bit(0x8C);
 
    dig_P1 = read16bit(0x8E);
    dig_P2 = (int16_t)read16bit(0x90);
    dig_P3 = (int16_t)read16bit(0x92);
    dig_P4 = (int16_t)read16bit(0x94);
    dig_P5 = (int16_t)read16bit(0x96);
    dig_P6 = (int16_t)read16bit(0x98);
    dig_P7 = (int16_t)read16bit(0x9A);
    dig_P8 = (int16_t)read16bit(0x9C);
    dig_P9 = (int16_t)read16bit(0x9E);
 
    dig_H1 = read8bit(0xA1);
    dig_H2 = (int16_t)read16bit(0xE1);
    dig_H3 = read8bit(0xE3);
    dig_H4 = (int16_t)((read8bit(0xE4) << 4) | (read8bit(0xE5) & 0x0F));
    dig_H5 = (int16_t)((read8bit(0xE6) << 4) | (read8bit(0xE5) >> 4));
    dig_H6 = (int8_t)read8bit(0xE7);
}
//***************BME280へ初期レジスタ書き込み関数****************************
void writeReg(uint8_t reg_address, uint8_t data) {
  digitalWrite(cs_bme, LOW);
  SpiWrite(reg_address & B01111111); // write, bit 7 low
  SpiWrite(data);
  digitalWrite(cs_bme, HIGH);
}
//***************BME280からの温度、湿度、気圧データ読み込み関数****************************
void readData() {
 
  uint32_t data[8];
  uint8_t i;
 
  digitalWrite(cs_bme, LOW);
  SpiWrite(0xF7 | B10000000); //0xF7 pressure msb read, bit 7 high
  for(i=0; i<8; i++){
    data[i] = SpiRead();
  }
  digitalWrite(cs_bme, HIGH);
  pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4); //0xF7, msb+lsb+xlsb=19bit
  temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4); //0xFA, msb+lsb+xlsb=19bit
  hum_raw  = (data[6] << 8) | data[7];  //0xFD, msb+lsb=19bit(16:0)
}
//***************温度キャリブレーション関数****************************
int32_t calibration_T(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
     
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}
//***************気圧キャリブレーション関数****************************
uint32_t calibration_P(int32_t adc_P) {
    int32_t var1, var2;
    uint32_t P;
    var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((int32_t)dig_P6);
    var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
    var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((int32_t)dig_P1))>>15);
    if (var1 == 0) {
        return 0;
    }    
    P = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000) {
       P = (P << 1) / ((uint32_t) var1);   
    }else{
        P = (P / (uint32_t)var1) * 2;    
    }
    var1 = (((int32_t)dig_P9) * ((int32_t)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((int32_t)(P>>2)) * ((int32_t)dig_P8))>>13;
    P = (uint32_t)((int32_t)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}
//***************湿度キャリブレーション関数****************************
uint32_t calibration_H(int32_t adc_H) {
    int32_t v_x1;
     
    v_x1 = (t_fine - ((int32_t)76800));
    v_x1 = (((((adc_H << 14) -(((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1)) + 
              ((int32_t)16384)) >> 15) * (((((((v_x1 * ((int32_t)dig_H6)) >> 10) * 
              (((v_x1 * ((int32_t)dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t)2097152)) * 
              ((int32_t) dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (uint32_t)(v_x1 >> 12);   
}
//***************標高計算関数****************************************************
double ReadAltitude(double SeaLevel_Pres, double pressure) {
  double altitude = 44330.0 * (1.0 - pow(pressure / SeaLevel_Pres, (1.0/5.255)));  
  return altitude;
}
//***************BME280から16bitデータ読み込み関数****************************
uint16_t read16bit(uint8_t reg) {
  uint16_t d1, d2;
  uint16_t data;
  digitalWrite(cs_bme, LOW);
  SpiWrite(reg | B10000000); // read, bit 7 high
  d1 = SpiRead();
  d2 = SpiRead();
  data = (d2 << 8) | d1;
  digitalWrite(cs_bme, HIGH);
  return data;
}
//***************BME280から8bitデータ読み込み関数****************************
uint8_t read8bit(uint8_t reg) {
  uint8_t data;
  digitalWrite(cs_bme, LOW);
  SpiWrite(reg | B10000000); // read, bit 7 high
  data = SpiRead();
  digitalWrite(cs_bme, HIGH);
  return data;
}
//***************BME280へSPI信号データ送信関数****************************
void SpiWrite(uint8_t data) {
  for (int i=7; i>=0; i--) {
    digitalWrite(sclk, LOW);
    digitalWrite(mosi, data & (1<<i));
    digitalWrite(sclk, HIGH);
  }
}
//***************BME280からのSPI信号データ読み込み関数****************************
uint8_t SpiRead() {
  uint8_t r_data = 0;
  for (int i=7; i>=0; i--) {
    r_data <<= 1;
    digitalWrite(sclk, LOW);
    digitalWrite(mosi, LOW);
    digitalWrite(sclk, HIGH);
    if(digitalRead(miso)){
      r_data |= 1;
    }
  }
  return r_data;
} 


