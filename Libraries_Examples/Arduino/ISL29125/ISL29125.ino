#include <Wire.h>

#include <SparkFunISL29125.h>

// Declare sensor object
SFE_ISL29125 RGB_sensor;

// Declare I2C pins
#define SDA_PIN 26
#define SCL_PIN 27

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize the ISL29125 with simple configuration so it starts sampling
  if (RGB_sensor.init())
  {
    Serial.println("Sensor Initialization Successful\n\r");
  } else {
    Serial.println("Sensor Initialization Failed\n\r");
  }
}

// Read sensor values for each color and print them to serial monitor
void loop()
{
  // Read sensor values (16 bit integers)
  unsigned int red = RGB_sensor.readRed();
  unsigned int green = RGB_sensor.readGreen();
  unsigned int blue = RGB_sensor.readBlue();

  // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(red, HEX);
  Serial.print("Green: "); Serial.println(green, HEX);
  Serial.print("Blue: "); Serial.println(blue, HEX);
  Serial.println();
  delay(2000);
}
