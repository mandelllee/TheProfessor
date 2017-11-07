/*
 Setup your scale and start the sketch WITHOUT a weight on the scale
 Once readings are displayed place the weight on the scale
 Press +/- or a/z to adjust the calibration_factor until the output readings match the known weight
*/

#include "HX711.h"

#define DAT_PIN  21 //U6 21, U7 0
#define SCK_PIN  19 //U6 19, U7 4

HX711 scale(DAT_PIN, SCK_PIN); //arguments: DAT_PIN, SCK_PIN

float calibration_factor = 112; // this calibration factor is adjusted according to my load cell
float units;
float ounces;
long zero_factor;

void setup() {
  Serial.begin(115200);
  Serial.println("HX711 calibration sketch");
  Serial.println("Remove all weight from scale");
  Serial.println("After readings begin, place known weight on scale");
  Serial.println("Press + or a to increase calibration factor");
  Serial.println("Press - or z to decrease calibration factor");

  //Get some readings before calibration, may increase precission
  zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Test: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);

  //Tare scale
  scale.set_scale();
  scale.tare(30);  //Reset the scale to 0

  //Get zero_factor
  zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);
}

void loop() {

  scale.set_scale(calibration_factor); //Adjust to this calibration factor

  if (Serial.available())
  {
    char temp = Serial.read();
    if (temp == '+' || temp == 'a')
      calibration_factor += 1;
    else if (temp == '-' || temp == 'z')
      calibration_factor -= 1;
  } else {
    Serial.print("Reading: ");
    units = scale.get_units(), 10;

    //Remove negative values
    //if (units < 0)
    //{
    // units = 0.00;
    //}

    //Convert to ounces
    ounces = units * 0.035274;

    //Set to 1 ro read RAW value
    if (0) {
      long value = scale.read_average();
      Serial.print(value);
      Serial.print(" RAW; ");
      Serial.print(value - zero_factor);
      Serial.print(" RAW_ofst; ");
    }

    //Display
    Serial.print(units);
    Serial.print(" grams;");
    Serial.print(" calibration_factor: ");
    Serial.print(calibration_factor);
    Serial.println();
    delay(1000);
  }
}

