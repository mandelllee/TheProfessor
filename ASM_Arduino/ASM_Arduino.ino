#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <WiFi.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SimpleDHT.h>
#include <HTTPClient.h>
//#include <ArduinoJson.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <driver/gpio.h>
#include <SSD1306.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <bt.h>

#include "FS.h"
#include "SPI.h"
//#include "SD.h"

/* Modified libraries */
#include "src/ESP32_SD/src/SD.h"
#include "src/TSL2561_Arduino_Library/TSL2561.h"
#include "src/SparkFun_ISL29125_Breakout/src/SparkFunISL29125.h"
#include "src/Adafruit_MCP23008_library/Adafruit_MCP23008.h"
#include "src/HX711/HX711.h"

/* User includes */
#include "VarDef.h"
#include "images.h"


/* Global variables ********************************************************/
esp_adc_cal_characteristics_t characteristics;
SFE_ISL29125 RGB_sensor;
Sensors_t Sensors;
SimpleDHT22 dht22;
HTTPClient http;
Adafruit_MCP23008 mcp;
TSL2561 tsl(TSL2561_ADDR_HIGH);
HX711 lc_1(pinDOUT_LC1, pinPD_SCK_LC1);
HX711 lc_2(pinDOUT_LC2, pinPD_SCK_LC2);
SSD1306  display((SSD1306_add>>1), LCD_SDA_PIN, LCD_SCL_PIN);
OneWire oneWire(ONE_WIRE_BUS);
BLECharacteristic *pCharacteristic;
BLEService *pService;
BLEServer *pServer;
Preferences preferences;
settings_t SysSettings;
hw_timer_t * timer = NULL;
bool deviceConnected = false;
bool BLE_DecodeFlag = false;
bool Sensors_ReadFlag = false;
volatile uint8_t BLE_Status = 0;
volatile uint32_t BLE_SleepCounter = 0;
volatile uint32_t BLE_TimeOutCounter = 0;
volatile uint32_t Sensors_ReadCounter = 0;

uint8_t cardType;
uint64_t cardSize;
SPIClass &spi = SPI;
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer, outsideThermometer;


/* Functions prototype *****************************************************/
void GPIO_Config(void);
void ADC_Config(void);
void MCP23008_Config(void);
void Sensors_Init(void);
bool ISL29125_Config(void);
bool TLS2561_Config(void);
void DHT22_Config(void);
void DS18B20_Config(void);
void HX711_Config(void);
bool HX711_GetStatus(HX711 lcStruct);
void SSD1306_Init(void);
void Send_json(void);
void Sensors_Read(void);
void Read_DHT22(pDHT22 SensStruct);
void Read_ISL29125(pISL29125 SensStruct);
void Read_TLS2561(pTLS2561 SensStruct);
void Read_DS18B20(void);
void Read_LoadCell(HX711 lcStruct, pLoadCell SensStruct);
void Read_VBat(void);
void BLE_init(void);
void BLE_enable(void);
void BLE_disable(void);
void DecodeMSG(std::string msg);
void Load_SysSettings(void);
void SD_read_settings(void);
void SD_connect(void);
void DecodeTextLine(char* line, uint8_t QLen);

void testFileIO(fs::FS &fs, const char * path);
void deleteFile(fs::FS &fs, const char * path);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void appendFile(fs::FS &fs, const char * path, const char * message);
void writeFile(fs::FS &fs, const char * path, const char * message);
void readFile(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void createDir(fs::FS &fs, const char * path);
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);


/*******************************************************************************
 * BLE Callbacks
 ******************************************************************************/
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.print("\r\n*** BLE connected");
      BLE_TimeOutCounter = 0;
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.print("\r\n*** BLE disconnected");
      deviceConnected = false;
    }
};
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      Serial.print("\r\n*** BLE received data");
      BLE_TimeOutCounter = 0;
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) 
        DecodeMSG(rxValue);//BLE_DecodeFlag = true;
    }
};


/*******************************************************************************
 * Time interrupt every second
 ******************************************************************************/
void IRAM_ATTR onTimer()
{
  Sensors_ReadCounter++;
    
  // Read Sensors every xx minutes
  if (Sensors_ReadCounter >= SysSettings.Meas_RepeatTime_s)
  {
    Sensors_ReadFlag = true;
    Sensors_ReadCounter = 0;
  }
  
/*  // Turn OFF BLE after xx seconds
  if (BLE_Status)
  {
    BLE_TimeOutCounter++;
    if (BLE_TimeOutCounter == SysSettings.BLE_ActiveTime_sec)
      BLE_disable();
  }
  
  // Turn ON BLE after xx minutes
  if (!BLE_Status)
  {
    BLE_SleepCounter++;
    if (BLE_SleepCounter ==(60*SysSettings.BLE_SleepTime_min))
      BLE_enable();
  }*/
}




/*******************************************************************************
 * Setup function
 * 
 ******************************************************************************/
void setup() 
{
  // Initialize UART
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin(Sens_SDA_PIN, Sens_SCL_PIN);

  // Configure spi
  spi.begin(SD_CLK, SD_DATA, SD_CMD); 

  // Load system parameters
  Load_SysSettings();

  delay(500);
  
  // connect to SD card and read systings.txt
  SD_connect();

  // Read SD for settings
  SD_read_settings();

  Serial.print("\r\n\r\n*** Peripherals state *******************************************"); 
  GPIO_Config();
  ADC_Config();

  // Initialize Port Expander
  MCP23008_Config();

  // Read Battery Voltage before Sensors Measurements
  Read_VBat();

  // Enable Sens_EN and LCD_EN
  LCD_ON;
  Sens_ON;
  delay(500);

  SSD1306_Init();         // Attention this function is changing I2C speed to 700kHz
  Wire.setClock(100000);  // Set I2C speed to 100KHz
  
  // Configure sensors
  Sensors_Init();
  
  // Initialize BLE and start
  //BLE_init();
  //BLE_enable();
  
  // Configure and start time
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);  // value in microseconds
  timerAlarmEnable(timer);

  // Draw WiFi demo
  drawImageDemo();
  display.display();
}



/*******************************************************************************
 * Main function
 * 
 ******************************************************************************/
void loop() 
{
  
/*  //-----------------------------------------------------------------------
  // Decode BLE messages
  if (BLE_DecodeFlag == true)
  {
    //BLE_decode(rxValue);
    BLE_DecodeFlag = false;
  }*/


  //-----------------------------------------------------------------------
  // Read Sensors and send data in json via WiFi
  if (Sensors_ReadFlag == true)
  {
    Sensors_Read();

    // Connect to WiFi
    uint8_t len1 = SysSettings.ssid.length()+1;
    uint8_t len2 = SysSettings.password.length()+1;
    char ssid[len1]={0};
    char password[len2]={0};
    SysSettings.ssid.toCharArray(ssid, len1);
    SysSettings.password.toCharArray(password, len2);
    WiFi.begin((const char*)ssid, (const char*)password); 
    Serial.print("\r\n\r\n*** Connecting to WiFi");
    for (uint8_t ConnectTimeOut=0; ConnectTimeOut<10 ;ConnectTimeOut++)
    {
      if(WiFi.status() == WL_CONNECTED) {
        break;}
      Serial.print(".");
      delay(500);
    }

    // Send json if connected
    if(WiFi.status() == WL_CONNECTED)
    {
      Serial.print("connected");
      // Send Sensors data in json message, after sending save in SD card
      Send_json();
    }
   else {
      Serial.printf("error %d", WiFi.status());}
  
  // Disconnect WiFi
  WiFi.disconnect(true);  // Erases SSID/password

  // Reset Sensors_ReadFlag
  Sensors_ReadFlag = false;
  }
}




/*******************************************************************************
 * Load_SysSettings - Load system parameters
 ******************************************************************************/
void Load_SysSettings(void)
{
  preferences.begin("my-app", false);
  SysSettings.ssid = preferences.getString("ssid", DefaultVal_ssid);
  SysSettings.password = preferences.getString("password", DefaultVal_password);
  SysSettings.hostname = preferences.getString("hostname", DefaultVal_hostname);
  SysSettings.url = preferences.getString("url");
  SysSettings.Meas_RepeatTime_s = preferences.getUInt("MeasRepeat", DefaultVal_Meas_RepeatTime_s);
  SysSettings.BLE_Timout_s = preferences.getUInt("BLEtimeout", DefaultVal_BLE_Timout_s);
  preferences.end();
  SysSettings.ver = Version;
  SysSettings.UnixTimeStamp = 1502467659;
  
  // Print system parameters
  Serial.print("\r\n*** System parameters *******************************************");
  Serial.print("\r\nWiFi SSID: "); Serial.print(SysSettings.ssid);
  Serial.print("\r\nWiFi password: "); Serial.print(SysSettings.password);
  Serial.print("\r\nHostname: "); Serial.print(SysSettings.hostname);
  Serial.print("\r\nURL: "); Serial.print(SysSettings.url);
  Serial.printf("\r\nMeas_RepeatTime_s: %d", SysSettings.Meas_RepeatTime_s);
  Serial.printf("\r\nBLE_Timout_s: %d", SysSettings.BLE_Timout_s);
  Serial.printf("\r\nVersion: %.2f", SysSettings.ver);
}


/*******************************************************************************
 * SD_connect -
 ******************************************************************************/
void SD_connect(void)
{
  for (uint8_t ConnectTimeOut=0; ConnectTimeOut<5 ;ConnectTimeOut++)
  {
    if (SD.begin(SD_CD, spi))
    {
      cardType = SD.cardType();
      if(cardType = !CARD_NONE)
      {
        Serial.print("\r\n\r\n*** SD Card Type: ");
        if(cardType == CARD_MMC)        Serial.print("MMC");
        else if(cardType == CARD_SD)    Serial.print("SDSC");
        else if(cardType == CARD_SDHC)  Serial.print("SDHC");
        else                            Serial.print("UNKNOWN");
        cardSize = SD.cardSize() / (1024 * 1024);  
        Serial.printf(" Size: %lluMB", cardSize);
        return;
      }
    }
    delay(100);
  }
  Serial.print("\r\n\r\n*** No SD card attached");
}


/*******************************************************************************
 * SD_read_settings -
 ******************************************************************************/
void SD_read_settings(void)
{
  if(cardType != CARD_NONE)
  {
    // open file
    File file = SD.open("/settings.txt");
    if(!file)
    {
      Serial.print("\r\n*** Failed to open settings.txt file");
      return;
    }

    // read file
    char text[200];   // max symbols in one line
    uint8_t cnt=0;  
    while(file.available())
    {
      if (cnt >= 198)
      {
        file.close();
        Serial.print("\r\n*** Failed to read settings.txt file to big!!");
        return;
      }
      
      char symbol = file.read();
      text[cnt] = symbol;  
      if (symbol == '\n')
      {
          DecodeTextLine(text, cnt);
          cnt = 0;
      }
      else
        cnt++;
    }
    file.close();
  }
}


/*******************************************************************************
 * DecodeTextLine -
 ******************************************************************************/
void DecodeTextLine(char* line, uint8_t QLen)
{
  char * pch;
  if (pch = strstr(line,"ssid:"))
  {
    char TempChar[QLen]={0};
    sscanf(pch,"ssid:%s\n",&TempChar);
    String TempStr = TempChar;
    if (TempStr != SysSettings.ssid)
    {
      SysSettings.ssid = TempStr;
      preferences.begin("my-app", false);
      preferences.putString("ssid", SysSettings.ssid);
      preferences.end();
      Serial.print("\r\n*** New WiFi SSID: ");Serial.print(SysSettings.ssid);
    }
  }

  if (pch = strstr(line,"password:"))
  {
    char TempChar[QLen]={0};
    sscanf(pch,"password:%s\n",&TempChar);
    String TempStr = TempChar;
    if (TempStr != SysSettings.password)
    {
      SysSettings.password = TempStr;
      preferences.begin("my-app", false);
      preferences.putString("password", SysSettings.password);
      preferences.end();
      Serial.print("\r\n*** New WiFi password: ");Serial.print(SysSettings.password);
    }
  }

  if (pch = strstr(line,"hostname:"))
  {
    char TempChar[QLen]={0};
    sscanf(pch,"hostname:%s\n",&TempChar);
    String TempStr = TempChar;
    if (TempStr != SysSettings.hostname)
    {
      SysSettings.hostname = TempStr;
      preferences.begin("my-app", false);
      preferences.putString("hostname", SysSettings.hostname);
      preferences.end();
      Serial.print("\r\n*** New Hostname: ");Serial.print(SysSettings.hostname);
    }
  }
  
  if (pch = strstr(line,"url:"))
  {
    char TempChar[QLen]={0};
    sscanf(pch,"url:%s\n",&TempChar);
    String TempStr = TempChar;
    if (TempStr != SysSettings.url)
    {
      SysSettings.url = TempStr;
      preferences.begin("my-app", false);
      preferences.putString("url", SysSettings.url);
      preferences.end();
      Serial.print("\r\n*** New URL: ");Serial.print(SysSettings.url);
    }
  }
  
  if (pch = strstr(line,"Meas_Repeat_s:"))
  {
    uint16_t TempVal=0;
    sscanf(pch,"Meas_Repeat_s:%d\n",&TempVal);
    if ( (TempVal >0) &&  (TempVal!= SysSettings.Meas_RepeatTime_s))
    {
      SysSettings.Meas_RepeatTime_s = TempVal;
      preferences.begin("my-app", false);
      preferences.putUInt("MeasRepeat", SysSettings.Meas_RepeatTime_s);
      preferences.end();
      Serial.print("\r\n*** New Meas_RepeatTime_s: ");Serial.print(SysSettings.Meas_RepeatTime_s);
    }
  }

  if (pch = strstr(line,"BLE_Timout_s:"))
  {
    uint16_t TempVal=0;
    sscanf(pch,"BLE_Timout_s:%d\n",&TempVal);
    if ( (TempVal >0) &&  (TempVal!= SysSettings.BLE_Timout_s))
    {
      SysSettings.BLE_Timout_s = TempVal;
      preferences.begin("my-app", false);
      preferences.putUInt("BLEtimeout", SysSettings.BLE_Timout_s);
      preferences.end();
      Serial.print("\r\n*** New BLE_Timout_s: ");Serial.print(SysSettings.BLE_Timout_s);
    }
  }
}


/*******************************************************************************
 * BLE_enable and BLE_disable
 ******************************************************************************/
void BLE_enable(void)
{
  Serial.print("\r\n*** BLE enable"); 
  esp_bt_controller_enable(ESP_BT_MODE_BTDM);
  delay(100);
  pServer->getAdvertising()->start(); 
  BLE_Status = 1;
}
void BLE_disable(void)
{
  Serial.print("\r\n*** BLE disable");  
  pServer->getAdvertising()->stop();  
  esp_bt_controller_disable();
  BLE_Status = 0;
}

/*******************************************************************************
 * DecodeMSG - Decode received message
 ******************************************************************************/
void DecodeMSG(std::string msg)
{
//  char const* p = msg.data();
//  const uint8_t QLen = 20;    // max length of SSID, password or hostname
//
//  /* Get (SG) and set (SS) SSID -----------------------------------------*/
//  if (strstr(p,"SG"))
//  {
//    if (deviceConnected) 
//    {
//      pCharacteristic->setValue(SysSettings.ssid.c_str());
//      pCharacteristic->notify();
//    }
//    Serial.print("\r\nBLE get SSID");
//  }
//  else if (strstr(p,"SS:"))
//  {
//    char TempStr[QLen];
//    sscanf(p,"SS:%s",&TempStr);
//    if (TempStr[1] != NULL)
//    {
//      SysSettings.ssid = TempStr;
//      preferences.begin("my-app", false);
//      preferences.putString("ssid", SysSettings.ssid);
//      preferences.end();
//      Serial.print("\r\nBLE set SSID: "); Serial.print(TempStr);
//    } 
//  }
//
//
//  /* Get (PG) and set (PS) WiFi password --------------------------------*/
//  else if (strstr(p,"PG"))
//  {
//    if (deviceConnected) 
//    {
//      pCharacteristic->setValue(SysSettings.password.c_str());
//      pCharacteristic->notify();
//    }
//    Serial.print("\r\nBLE get password");
//  }
//  else if (strstr(p,"PS:"))
//  {
//    char TempStr[QLen];
//    sscanf(p,"PS:%s",&TempStr);
//    if (TempStr[1] != NULL)
//    {
//      SysSettings.password = TempStr;
//      preferences.begin("my-app", false);
//      preferences.putString("password", SysSettings.password);
//      preferences.end();
//      Serial.print("\r\nBLE set password: "); Serial.print(TempStr);
//    } 
//  }
//
//  
//  /* Get (HG) and set (HS) Hostname -------------------------------------*/
//  else if (strstr(p,"HG"))
//  {
//    if (deviceConnected) 
//    {
//      pCharacteristic->setValue(SysSettings.hostname.c_str());
//      pCharacteristic->notify();
//    }
//    Serial.print("\r\nBLE get hostname");
//  }
//  else if (strstr(p,"HS:"))
//  {
//    char TempStr[QLen];
//    sscanf(p,"HS:%s",&TempStr);
//    if (TempStr[1] != NULL)
//    {
//      SysSettings.hostname = TempStr;
//      preferences.begin("my-app", false);
//      preferences.putString("hostname", SysSettings.hostname);
//      preferences.end();
//      Serial.print("\r\nBLE set hostname: "); Serial.print(TempStr);
//    } 
//  }
//
//
//  /* Get (MRTG) and set (MRTS) measurments repeatition time -------------*/
//  else if (strstr(p,"MRTG"))
//  {
//    if (deviceConnected) 
//    {
//      String TempStr = String(SysSettings.Meas_RepeatTime_min);
//      pCharacteristic->setValue(TempStr.c_str());
//      pCharacteristic->notify();
//    }
//    Serial.print("\r\nBLE get Meas_RepeatTime_min");
//  }
//  else if (strstr(p,"MRTS:"))
//  {
//    uint16_t TempVal=0;
//    sscanf(p,"MRTS:%d",&TempVal);
//    if (TempVal > 0)
//    {
//      SysSettings.Meas_RepeatTime_min = TempVal;
//      preferences.begin("my-app", false);
//      preferences.putUInt("MeasRepeat", SysSettings.Meas_RepeatTime_min);
//      preferences.end();
//      Serial.printf("\r\nBLE set Meas_RepeatTime_min: %d", SysSettings.Meas_RepeatTime_min);
//    } 
//  }
//
//
//  /* Get (BSTG) and set (BSTS) Bluetooth sleep time ---------------------*/
//  else if (strstr(p,"BSTG"))
//  {
//    if (deviceConnected) 
//    {
//      String TempStr = String(SysSettings.BLE_SleepTime_min);
//      pCharacteristic->setValue(TempStr.c_str());
//      pCharacteristic->notify();
//    }
//    Serial.print("\r\nBLE get BLE_SleepTime_min");
//  }
//  else if (strstr(p,"BSTS:"))
//  {
//    uint16_t TempVal=0;
//    sscanf(p,"BSTS:%d",&TempVal);
//    if (TempVal > 0)
//    {
//      SysSettings.BLE_SleepTime_min = TempVal;
//      preferences.begin("my-app", false);
//      preferences.putUInt("BLEsleep", SysSettings.BLE_SleepTime_min);
//      preferences.end();
//      Serial.printf("\r\nBLE set BLE_SleepTime_min: %d", SysSettings.BLE_SleepTime_min);
//    } 
//  }
//
//  
//  /* Get (BATG) and set (BATS) Bluetooth active time --------------------*/
//  else if (strstr(p,"BATG"))
//  {
//    if (deviceConnected) 
//    {
//      String TempStr = String(SysSettings.BLE_ActiveTime_sec);
//      pCharacteristic->setValue(TempStr.c_str());
//      pCharacteristic->notify();
//    }
//    Serial.print("\r\nBLE get BLE_ActiveTime_sec");
//  }
//  else if (strstr(p,"BATS:"))
//  {
//    uint16_t TempVal=0;
//    sscanf(p,"BATS:%d",&TempVal);
//    if (TempVal > 10)
//    {
//      SysSettings.BLE_ActiveTime_sec = TempVal;
//      preferences.begin("my-app", false);
//      preferences.putUInt("BLEactive", SysSettings.BLE_ActiveTime_sec);
//      preferences.end();
//      Serial.printf("\r\nBLE set BLE_ActiveTime_sec: %d", SysSettings.BLE_ActiveTime_sec);
//    } 
//  }
//
//
//  /* Get (CF1G) and set (CF1S) LoadCell_1 calibrationfactor factor ------*/
//  else if (strstr(p,"CF1G"))
//  {
//    if (deviceConnected) 
//    {
//      String TempStr = String( (SysSettings.LoadCell_calibration_factor_1), 3);
//      pCharacteristic->setValue(TempStr.c_str());
//      pCharacteristic->notify();
//    }
//  }
//  else if (strstr(p,"CF1S:"))
//  {
//    float TempVal=0.0;
//    sscanf(p,"CF1S:%f",&TempVal);
//    if (TempVal > 0)
//    {
//      SysSettings.LoadCell_calibration_factor_1 = TempVal;
//      Sensors.LoadCell_1.calibration_factor = SysSettings.LoadCell_calibration_factor_1;  
//      lc_1.set_scale(Sensors.LoadCell_1.calibration_factor);
//      preferences.begin("my-app", false);
//      preferences.putFloat("Calib1", SysSettings.LoadCell_calibration_factor_1);
//      preferences.end();
//      Serial.printf("\r\nLoadCell_1 calibration factor: %f", SysSettings.LoadCell_calibration_factor_1);
//    } 
//  }
//
//
//  /* Get (CF2G) and set (CF2S) LoadCell_2 calibrationfactor factor ------*/
//  else if (strstr(p,"CF2G"))
//  {
//    if (deviceConnected) 
//    {
//      String TempStr = String( (SysSettings.LoadCell_calibration_factor_2), 3);
//      pCharacteristic->setValue(TempStr.c_str());
//      pCharacteristic->notify();
//    }
//  }
//  else if (strstr(p,"CF2S:"))
//  {
//    float TempVal=0.0;
//    sscanf(p,"CF2S:%f",&TempVal);
//    if (TempVal > 0)
//    {
//      SysSettings.LoadCell_calibration_factor_2 = TempVal;
//      Sensors.LoadCell_2.calibration_factor = SysSettings.LoadCell_calibration_factor_2;  
//      lc_2.set_scale(Sensors.LoadCell_2.calibration_factor);
//      preferences.begin("my-app", false);
//      preferences.putFloat("Calib2", SysSettings.LoadCell_calibration_factor_2);
//      preferences.end();
//      Serial.printf("\r\nLoadCell_2 calibration factor: %f", SysSettings.LoadCell_calibration_factor_2);
//    } 
//  }
//
//
//  /* Tare LoadCell_1 (TLC1) and LoadCell_2 (TLC2) to zero factor ---------*/
//  else if (strstr(p,"TLC1"))
//  {
//    LoadCell_tare(lc_1, &(Sensors.LoadCell_1));
//    if (deviceConnected) 
//    {
//      String TempStr = "Tared LC1";
//      pCharacteristic->setValue(TempStr.c_str());
//      pCharacteristic->notify();
//    }
//  }
//  else if (strstr(p,"TLC2"))
//  {
//    LoadCell_tare(lc_2, &(Sensors.LoadCell_2));
//    if (deviceConnected) 
//    {
//      String TempStr = "Tared LC2";
//      pCharacteristic->setValue(TempStr.c_str());
//      pCharacteristic->notify();
//    }
//  } 
//
//  /* Reset LoadCell_1 (ZF1R) and LoadCell_2 (ZF2R) zero factor -----------*/
//  else if (strstr(p,"ZF1R"))
//  {
//    Sensors.LoadCell_1.zero_factor = 0;
//    SysSettings.LoadCell_zero_factor_1 = 0;
//    lc_1.set_offset(Sensors.LoadCell_1.zero_factor);
//    preferences.begin("my-app", false);
//    preferences.putLong("Offset1", SysSettings.LoadCell_zero_factor_1);
//    preferences.end();
//    if (deviceConnected) 
//    {
//      String TempStr = "LC1 Zero factor 0";
//      pCharacteristic->setValue(TempStr.c_str());
//      pCharacteristic->notify();
//    }
//    Serial.printf("\r\nLoadCell_1 zero factor: "); Serial.print(SysSettings.LoadCell_zero_factor_1);
//  }
//    else if (strstr(p,"ZF2R"))
//    {
//      Sensors.LoadCell_2.zero_factor = 0;
//      SysSettings.LoadCell_zero_factor_2 = 0;
//      lc_2.set_offset(Sensors.LoadCell_2.zero_factor);
//      preferences.begin("my-app", false);
//      preferences.putLong("Offset2", SysSettings.LoadCell_zero_factor_2);
//      preferences.end();
//      if (deviceConnected) 
//      {
//        String TempStr = "LC2 Zero factor 0";
//        pCharacteristic->setValue(TempStr.c_str());
//        pCharacteristic->notify();
//      }
//      Serial.printf("\r\nLoadCell_2 zero factor: "); Serial.print(SysSettings.LoadCell_zero_factor_2);
//    }
}



void BLE_init(void)
{
  // Create the BLE Device
  BLEDevice::init("ESP32DEV");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  pService = pServer->createService(SERVICE_UUID);
  
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY);                
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service 
  pService->start();                  
}


/*******************************************************************************
 * GPIO_Config - Initialize digital pins
 * https://esp-idf.readthedocs.io/en/v2.0/api/peripherals/gpio.html#_CPPv211gpio_mode_t
 ******************************************************************************/
void GPIO_Config(void)
{
//  gpio_config_t io_conf;
//
//  // Configure GPIO_NUM_0
//  io_conf.intr_type = GPIO_INTR_DISABLE ;
//  io_conf.mode = GPIO_MODE_INPUT;
//  io_conf.pin_bit_mask = GPIO_SEL_0;
//  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
//  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
//  gpio_config(&io_conf); 
}


/*******************************************************************************
 * ADC_Config - Initialize analog inputs
 * https://esp-idf.readthedocs.io/en/v2.0/api/peripherals/adc.html
 ******************************************************************************/
void ADC_Config(void)
{
  adc_atten_t ADC_ATTEN = ADC_ATTEN_6db;  // max 2.2V
  adc1_config_width(ADC_WIDTH_12Bit);
  
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN);
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN);
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN);

  // Calibrate ADC
  esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, &characteristics);
  Serial.print("\r\nADC offset: "); 
  Serial.print(characteristics.offset); Serial.print("mV, ideal_offset: "); 
  Serial.print(characteristics.ideal_offset); Serial.print("mV");
  //adc2_vref_to_gpio(GPIO_NUM_25); // To measure Vref 1.1V on 25,26,27 pins
}


/*******************************************************************************
 * MCP23008_Config - Initialize MCP23008
 * http://ww1.microchip.com/downloads/en/DeviceDoc/21919e.pdf
 ******************************************************************************/
void MCP23008_Config(void)
{
  mcp.begin(MCP23008_add);
 
  mcp.pullUp(0, HIGH);
  mcp.pullUp(1, HIGH);
  mcp.pullUp(2, HIGH);
  mcp.pullUp(3, HIGH);
  mcp.pullUp(4, HIGH);
  mcp.pullUp(5, HIGH);
  mcp.pullUp(6, HIGH);
  
  mcp.pinMode(0, OUTPUT);
  mcp.pinMode(1, OUTPUT);
  mcp.pinMode(2, OUTPUT);
  mcp.pinMode(3, OUTPUT);
  mcp.pinMode(4, OUTPUT);
  mcp.pinMode(5, OUTPUT);
  mcp.pinMode(6, OUTPUT); 
}


/*******************************************************************************
 * ISL29125_Config - Initialize ISL29125
 * https://www.intersil.com/content/dam/Intersil/documents/isl2/isl29125.pdf
 ******************************************************************************/
bool ISL29125_Config(void)
{
  if (RGB_sensor.init() == true)
  {
    RGB_sensor.config(CFG1_MODE_RGB|CFG1_10KLUX|CFG1_16BIT, CFG2_IR_ADJUST_HIGH, CFG3_NO_INT);
    Sensors.ISL29125.state = 1;
    Serial.print("\r\nISL29125 Initialization Successful");
    return true;
  }
  else
  {
    Sensors.ISL29125.state = 0;
    Serial.print("\r\nISL29125 Initialization Failed");
    return false;
  }
}


/*******************************************************************************
 * TLS2561_Config - Initialize TLS2561
 * https://cdn-shop.adafruit.com/datasheets/TSL2561.pdf
 ******************************************************************************/
bool TLS2561_Config(void)
{
  if (tsl.begin() == true)
  {
    tsl.setGain(TSL2561_GAIN_16X);                  // 16x gain ... use in low light to boost sensitivity
    tsl.setTiming(TSL2561_INTEGRATIONTIME_402MS);   // 16-bit data but slowest conversions
    Sensors.TLS2561.state = 1;
    Serial.print("\r\nTSL2561 Initialization Successful");
    return true;
  } 
  else
  {
    Sensors.TLS2561.state = 0;
    Serial.print("\r\nTSL2561 Initialization Failed");
    return false;
  }
}

/*******************************************************************************
 * DHT22_Config - Initialize DHT22
 * https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf
 ******************************************************************************/
void DHT22_Config(void)
{
  Sensors.DHT22_1.pin = pinDHT22_1;
  Sensors.DHT22_2.pin = pinDHT22_2;

  // Perform Read to get Status
  Read_DHT22(&(Sensors.DHT22_1));
  Read_DHT22(&(Sensors.DHT22_2));
}

/*******************************************************************************
 * DS18B20_Config - Initialize DS18B20
 * https://cdn.sparkfun.com/datasheets/Sensors/Temp/DS18B20.pdf
 ******************************************************************************/
void DS18B20_Config(void)
{
  // Start up the library
  sensors.begin();

  Serial.print("\r\nDS18B20 Found ");
  Serial.print(sensors.getDeviceCount(), DEC); Serial.print(" devices, ");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.print("ON");
  else Serial.print("OFF");

  // method 1: by index
  if (!sensors.getAddress(insideThermometer, 0)) Serial.print("Unable to find address for Device 0"); 
  if (!sensors.getAddress(outsideThermometer, 1)) Serial.print("Unable to find address for Device 1"); 

  // show the addresses
  Serial.print("\r\nDevice 0 Address: ");
  printAddress(insideThermometer);
  sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
  Serial.print(" Resolution: "); Serial.print(sensors.getResolution(insideThermometer), DEC); 
  
  Serial.print("\r\nDevice 1 Address: ");
  printAddress(outsideThermometer);
  sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);
  Serial.print(" Resolution: "); Serial.print(sensors.getResolution(outsideThermometer), DEC); 
}


/*******************************************************************************
 * HX711_Config - Initialize HX711
 * https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/hx711_english.pdf
 ******************************************************************************/
void HX711_Config(void)
{
  Sensors.LoadCell_1.DOUT_pin = pinDOUT_LC1;
  Sensors.LoadCell_1.PD_SCK_pin = pinPD_SCK_LC1;
  Sensors.LoadCell_2.DOUT_pin = pinDOUT_LC2;
  Sensors.LoadCell_2.PD_SCK_pin = pinPD_SCK_LC2;

  Sensors.LoadCell_1.Gain = 128;
  Sensors.LoadCell_2.Gain = 128;

  // Set gain
  lc_1.set_gain(Sensors.LoadCell_1.Gain);
  lc_2.set_gain(Sensors.LoadCell_2.Gain);
}


bool HX711_GetStatus(HX711 lcStruct)
{
  float val = lcStruct.read_average(1);
  if (val == 0) return false;
  return true;
}


/*******************************************************************************
 * Sensors_Init - Initialize all Sensors
 ******************************************************************************/
void Sensors_Init(void)
{
  Sensors.counter = 0;
  ISL29125_Config();
  TLS2561_Config();
  DHT22_Config();
  HX711_Config();
  DS18B20_Config();
}

void Read_ISL29125(pISL29125 SensStruct)
{
  // Restart configuration if needed
  if (SensStruct->state == 0)
  {
    if (ISL29125_Config() == false)
    {
      SensStruct->Red = 0;
      SensStruct->Green = 0;
      SensStruct->Blue = 0;
      return;
    }
  }
  
  SensStruct->Red = RGB_sensor.readRed();
  SensStruct->Green = RGB_sensor.readGreen();
  SensStruct->Blue = RGB_sensor.readBlue();
  
  if (SensStruct->Red == 0xFFFF)
  {
    SensStruct->state = 0;
    Serial.print("\r\nISL29125: error");
  }
  else
  {
    Serial.print("\r\nISL29125: ");
    Serial.print("Red: "); Serial.print(SensStruct->Red); Serial.print(", ");
    Serial.print("Green: "); Serial.print(SensStruct->Green); Serial.print(", ");
    Serial.print("Blue: "); Serial.print(SensStruct->Blue);
  }
}

void Read_TLS2561(pTLS2561 SensStruct)
{ 
  uint32_t lum=0;
  uint16_t ir, full, vis;

  // Restart configuration if needed
  if (SensStruct->state == 0)
  {
    if (TLS2561_Config() == false)
    {
      SensStruct->IR = 0;
      SensStruct->Full = 0;
      SensStruct->Visible = 0;
      SensStruct->Lux = 0;
      return;
    }
  }
      
  lum = tsl.getFullLuminosity();
  ir = lum >> 16;
  full = lum & 0xFFFF;
  vis = full - ir;

  SensStruct->IR = ir;
  SensStruct->Full = full;
  SensStruct->Visible = vis;
  SensStruct->Lux = tsl.calculateLux(full, ir);
  
  if (lum == 0)
  {
    SensStruct->state = 0;
    Serial.print("\r\nTLS2561: error");
  }
  else
  {
    Serial.printf("\r\nTLS2561: IR: %d, Full: %d, Visible: %d, Lux: %d", SensStruct->IR, SensStruct->Full, SensStruct->Visible, SensStruct->Lux);
  } 
}

void Read_DHT22(pDHT22 SensStruct)
{
  int err;
  float temperature = 0;
  float humidity = 0;
  
  err = dht22.read2(SensStruct->pin, &temperature, &humidity, NULL);
  if (err != SimpleDHTErrSuccess) 
  {
    SensStruct->Temperature_C = 0;
    SensStruct->Temperature_F = 0;
    SensStruct->Humidity = 0;
    SensStruct->state = 0;
    Serial.printf("\r\nDHT22 (pin %d): error %d", SensStruct->pin, err);
  }
  else
  {
    SensStruct->Temperature_C = temperature;
    SensStruct->Temperature_F = temperature * 1.8 + 32;
    SensStruct->Humidity = humidity;
    SensStruct->state = 1;
    Serial.print("\r\nDHT22 (pin "); Serial.print(SensStruct->pin); Serial.print("):");
    Serial.print(" Temperature: "); 
    Serial.print((float)SensStruct->Temperature_C); Serial.print("°C / ");
    Serial.print((float)SensStruct->Temperature_F); Serial.print("°F");
    Serial.print(" Humidity: "); 
    Serial.print((float)SensStruct->Humidity); Serial.print("RH%");
  }
}

void Read_LoadCell(HX711 lcStruct, pLoadCell SensStruct)
{
  if (HX711_GetStatus(lcStruct) == false)
  {
    SensStruct->state = 0;
    Serial.printf("\r\nLoadCell (pins %d %d): error", SensStruct->DOUT_pin, SensStruct->PD_SCK_pin); 
    return;
  }
  
  SensStruct->units = lcStruct.read_average(10);
  SensStruct->state = 1;
  Serial.printf("\r\nLoadCell (pins %d %d): %d", SensStruct->DOUT_pin, SensStruct->PD_SCK_pin, SensStruct->units); 
}

void Read_DS18B20(void)
{
  sensors.requestTemperatures();
  printData(insideThermometer);
  printData(outsideThermometer);
}

// Read Battery Voltage in mV
void Read_VBat(void)
{
  Sens_OFF;
  delay(100);
  
  float val = ((float)adc1_to_voltage(ADC1_CHANNEL_5, &characteristics)) * 5.4; // Rdiv 5.4
  Sensors.BatVoltage = (uint16_t)val;
  Serial.printf("\r\nBatery Voltage: %dmV", Sensors.BatVoltage);
}

/*******************************************************************************
 * Sensors_Read - Read Data from all Sensors
 ******************************************************************************/
void Sensors_Read(void)
{
  Sens_ON;
  delay(100);
  
  Serial.print("\r\n\r\n*** Sensors Readings No "); Serial.print(Sensors.counter);
  Serial.print(" ***************************************");
  Sensors.counter++;
  
  // Read DHT22
  Read_DHT22(&(Sensors.DHT22_1));
  Read_DHT22(&(Sensors.DHT22_2));

  // Read ISL29125
  Read_ISL29125(&(Sensors.ISL29125));
  
  // Read TLS2561
  Read_TLS2561(&(Sensors.TLS2561));

  // Read HX711 (Load Cells)
  Read_LoadCell(lc_1, &(Sensors.LoadCell_1));
  Read_LoadCell(lc_2, &(Sensors.LoadCell_2));  

//  // Read DS18B20
//  Read_DS18B20();

  // Read Analog (AIN1-AIN6) inputs, Rdiv = 2
  Sensors.AIN[0] = adc1_to_voltage(ADC1_CHANNEL_6, &characteristics) * 2;
  Sensors.AIN[1] = adc1_to_voltage(ADC1_CHANNEL_7, &characteristics) * 2;
  Sensors.AIN[2] = adc1_to_voltage(ADC1_CHANNEL_4, &characteristics) * 2;
  Sensors.AIN[3] = adc1_to_voltage(ADC1_CHANNEL_5, &characteristics) * 2;
  Sensors.AIN[4] = adc1_to_voltage(ADC1_CHANNEL_3, &characteristics) * 2;
  Sensors.AIN[5] = adc1_to_voltage(ADC1_CHANNEL_0, &characteristics) * 2;

  Serial.print("\r\nAnalog pins: "); 
  Serial.printf("AIN1: %dmV, ",Sensors.AIN[0]);
  Serial.printf("AIN2: %dmV, ",Sensors.AIN[1]);
  Serial.printf("AIN3: %dmV, ",Sensors.AIN[2]);
  Serial.printf("AIN4: %dmV, ",Sensors.AIN[3]);
  Serial.printf("AIN5: %dmV, ",Sensors.AIN[4]);
  Serial.printf("AIN6: %dmV", Sensors.AIN[5]);
}


/*******************************************************************************
 * SSD1306_Init - LCD init
 ******************************************************************************/
void SSD1306_Init(void)
{
  LCD_ON;
  delay(100);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
}
void drawImageDemo() 
{
  // see http://blog.squix.org/2015/05/esp8266-nodemcu-how-to-create-xbm.html
  // on how to create xbm files
  display.drawXbm(34, 14, WiFi_Logo_width, WiFi_Logo_height, WiFi_Logo_bits);
}



/*******************************************************************************
 * Send_json - Send json message to server
 ******************************************************************************/
void Send_json(void)
{
  //-----------------------------------------------------------------------
  // Build json
  //char JSONMessage[] = "{\"hostname\":\"ESP32DEV\",\"core_version\":\"0.1\",\"now\":\"1508949067\",\"firmware\":\"filename\",\"environment\":{\"light\":{\"lux\":4681}, \"light-rgb\":{\"red\":255,\"green\":255,\"blue\":255},\"air-1\":{\"humidity\":43,\"temp\":87},\"air-2\":{\"humidity\":41.1,\"temp\":87},\"co2\":{\"k30\":1350,\"mg811\":400}},\"other\":{\"analog_1\":99,\"analog_2\":550,\"analog_3\":5.7,\"analog_4\":1063,\"gpio_1\":87, \"gpio_2\":0,\"i2c_1\":500,\"i2c_2\":0,\"battery\":55}}";
  
  // top part
  String STR_hostname = String("\"hostname\":\"" + (SysSettings.hostname) + "\",");
  String STR_version = String("\"core_version\":\"" + String(SysSettings.ver,1) + "\",");
  String STR_now = String("\"now\":\"" + String(SysSettings.UnixTimeStamp,DEC) + "\","); 
  String STR_firm = String("\"firmware\":\"filename\",");

  // environment
  String STR_light = String("\"light\":{\"lux\":" + String(Sensors.TLS2561.Lux) + "},");
  String STR_red = String("\"red\":" + String(Sensors.ISL29125.Red) + ",");
  String STR_green= String("\"green\":" + String(Sensors.ISL29125.Green) + ",");
  String STR_blue = String("\"blue\":" + String(Sensors.ISL29125.Blue));
  String STR_rgb = String("\"light-rgb\":{" + STR_red + STR_green + STR_blue + "},");
  String STR_hum1 = String("\"humidity\":" + String(Sensors.DHT22_1.Humidity) + ",");
  String STR_temp1 = String("\"temp\":" + String(Sensors.DHT22_1.Temperature_F));
  String STR_air1 = String("\"air-1\":{" + STR_hum1 + STR_temp1 + "},");
  String STR_hum2 = String("\"humidity\":" + String(Sensors.DHT22_2.Humidity) + ",");
  String STR_temp2 = String("\"temp\":" + String(Sensors.DHT22_2.Temperature_F));
  String STR_air2 = String("\"air-2\":{" + STR_hum2 + STR_temp2 + "},");
  String STR_k30 = String("\"k30\":" + String(1350) + ",");
  String STR_mg811 = String("\"mg811\":" + String(400));
  String STR_co2 = String("\"co2\":{" + STR_k30 + STR_mg811 + "}");
  String STR_environment = String("\"environment\":{" + STR_light + STR_rgb + STR_air1 + STR_air2 + STR_co2 + "},");

  // other
  String STR_A1 = String("\"analog_1\":" + String(Sensors.AIN[0]) + ",");
  String STR_A2 = String("\"analog_2\":" + String(Sensors.AIN[1]) + ",");
  String STR_A3 = String("\"analog_3\":" + String(Sensors.AIN[2]) + ",");
  String STR_A4 = String("\"analog_4\":" + String(Sensors.AIN[3]) + ",");
  String STR_GPIO1 = String("\"gpio_1\":" + String(0) + ",");
  String STR_GPIO2 = String("\"gpio_2\":" + String(0) + ",");
  String STR_I2C1 = String("\"i2c_1\":" + String(Sensors.LoadCell_1.units) + ",");
  String STR_I2C2 = String("\"i2c_2\":" + String(Sensors.LoadCell_2.units) + ",");  
  String STR_BAT = String("\"battery\":" + String(Sensors.BatVoltage)); 
  String STR_other = String("\"other\":{" + STR_A1 + STR_A2 + STR_A3 + STR_A4 + STR_GPIO1 + STR_GPIO2 + STR_I2C1 + STR_I2C2 + STR_BAT + "}");
 
  String JSONMessage = String("{" + STR_hostname + STR_version + STR_now + STR_firm + STR_environment + STR_other + "}");
  //Serial.println(JSONMessage);  // check json: https://jsonlint.com/


  //-----------------------------------------------------------------------
  // POST json message to HTTP
  uint8_t len = SysSettings.url.length()+1;
  char httpServer[len]={0};
  SysSettings.url.toCharArray(httpServer, len);
  http.begin(httpServer);                             // Specify destination for HTTP request
  http.addHeader("Content-Type", "application/json"); // Specify content-type header
  Serial.print("\r\n\r\n*** Sending json to server: ");
  int httpResponseCode = http.POST(JSONMessage);  
  String response = http.getString();
  Serial.print(response);
  http.end();  // Free resources


  //-----------------------------------------------------------------------
  // Save json to SD card
  Serial.print("\r\n\r\n*** Saving json to SD card: ");
  cardType = SD.cardType();
  if(cardType = !CARD_NONE)
  {
    SD.mkdir("/Data");  // make sure Data dir is exist
    File file = SD.open("/Data/SensorsData.txt", FILE_APPEND);
    if(!file){
      Serial.print("failed");}
    else
    {
      for (uint32_t index=0; index<JSONMessage.length(); index++)
      {
        file.print(JSONMessage.charAt(index));
      }
      file.print('\n');
      Serial.print("saved");
    }
    file.close();
  }
  else{
    Serial.print("SD card not found");}
}












// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.print("°C / ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
  Serial.print("°F");
}
// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));  
}
// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("\r\nDS18B20 (");
  printAddress(deviceAddress);
  Serial.print(") ");
  printTemperature(deviceAddress);
}




/*******************************************************************************
 * SD card functions
 ******************************************************************************/
void listDir(fs::FS &fs, const char * dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path)
{
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path)
{
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path)
{
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message)
{
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message)
{
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2)
{
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path)
{
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path)
{
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}
