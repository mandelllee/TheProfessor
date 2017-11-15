#ifndef VARDEF_h
#define VARDEF_h


#define Version         0.1

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

/*
 * SD Card | ESP32
 *    D2       -
 *    D3       SS
 *    CMD      MOSI
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      SCK
 *    VSS      GND
 *    D0       MISO
 *    D1       -
 */
//Define GPIO used by SD card in SPI mode
#define SD_CLK          14
#define SD_CMD          15
#define SD_DATA         13  // be careful !!!
#define SD_CD           25
#define I2C_speed       100000

// Select pins for K30
#define SERIAL1_RXPIN 16
#define SERIAL1_TXPIN 17

// Declare I2C for Sensors and LCD pins
#define Sens_SDA_PIN    26
#define Sens_SCL_PIN    27
#define LCD_SDA_PIN     Sens_SDA_PIN
#define LCD_SCL_PIN     Sens_SCL_PIN

#define MCP23008_add    0x27
#define SSD1306_add     0x78
#define K30co2Addr      0x68

// Declare DS18B20 pin and precision
#define ONE_WIRE_BUS            23
#define TEMPERATURE_PRECISION   9

// Declare pinDHT22 pins
#define pinDHT22_1      22
#define pinDHT22_2      18    // error in hardware !!!

// Declare LoadCells pins
#define pinDOUT_LC1     5     // be careful !!!
#define pinPD_SCK_LC1   4
#define pinDOUT_LC2     21    
#define pinPD_SCK_LC2   19

#define V_REF   1100  // ADC reference voltage

#define Sens_ENPin      5
#define LCD_ENPin       6

#define Sens_ON         MCP23008_SetLevel(Sens_ENPin, 0)    //mcp.digitalWrite(5, LOW);
#define Sens_OFF        MCP23008_SetLevel(Sens_ENPin, 1)    //mcp.digitalWrite(5, HIGH);
#define LCD_ON          MCP23008_SetLevel(LCD_ENPin, 0)     //mcp.digitalWrite(6, LOW);
#define LCD_OFF         MCP23008_SetLevel(LCD_ENPin, 1)     //mcp.digitalWrite(6, HIGH);

// Default Values for SysSettings
#define DefaultVal_ssid                   "WiFi_SSDI"
#define DefaultVal_password               "WiFi_Password"
#define DefaultVal_hostname               "ESP32DEV"
#define DefaultVal_url                    "http://api-quadroponic.rhcloud.com/v1/record/sensordata"
#define DefaultVal_Meas_RepeatTime_s      5
#define DefaultVal_BLE_Timout_s           60

typedef struct { 
  String    ssid;  
  String    password;
  String    hostname;
  String    url;
  uint16_t  Meas_RepeatTime_s;   // Measurments repeatition in minutes
  uint16_t  BLE_Timout_s;        // Bluetooth active time in seconds
  float     ver;
  volatile uint32_t UnixTimeStamp;  
} settings_t,*psettings;

typedef struct {
  uint16_t Red;  
  uint16_t Green;
  uint16_t Blue;
  uint8_t state;
} ISL29125_t,*pISL29125;

typedef struct {
  uint8_t DOUT_pin;
  uint8_t PD_SCK_pin;
  uint8_t Gain;
  long raw;
  uint8_t state;
} LoadCell_t,*pLoadCell;

typedef struct {
  uint16_t IR;
  uint16_t Full;
  uint16_t Visible;
  uint32_t Lux;
  uint8_t state;
} TLS2561_t,*pTLS2561;

typedef struct {
  float Temperature_C;
  float Temperature_F;
  float Humidity_RH;
  int pin;
} DHT22_t,*pDHT22;

typedef struct {
  DHT22_t     	DHT22_1;
  DHT22_t     	DHT22_2;
  TLS2561_t   	TLS2561;
  ISL29125_t  	ISL29125;
  LoadCell_t	  LoadCell_1;
  LoadCell_t	  LoadCell_2;
  float         DS18B20_Temp_F;
  uint16_t      co2_k30;
  uint32_t      co2_mg811;
  uint32_t      AIN[6];
  uint32_t      BatVoltage;
  uint32_t      counter;
} Sensors_t;


#endif /* VARDEF_h */
