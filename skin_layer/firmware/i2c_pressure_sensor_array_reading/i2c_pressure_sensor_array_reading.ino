/*
 * I2C Expansion bus reading for fixed address pressure sensors using TCA9548 chip
 * Camilo F. Buscaron
 * 1/2017
 * Partially based on code c. 2009, Tod E. Kurt, http://todbot.com/blog/ and Sparkfun Libraries
 *
 */
 
#include "SparkFunMPL3115A2.h"
#include "Wire.h"

extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

MPL3115A2 pressure_sensors[7];

#define TCAADDR 0x70
 
void tcaselect(uint8_t i) 
{
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup()
{
    while (!Serial);
    delay(1000);
 
    Wire.begin();
    
    Serial.begin(115200);
    Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);
 
      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
      
        uint8_t data;
        if (! twi_writeTo(addr, &data, 0, 1, 1)) {
           Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
    Serial.println("\ndone");

    // Initialise sensors
    for (uint8_t i = 0; i < 8; i++)
    {
      tcaselect(i);

      pressure_sensors[i].begin(); // Get sensor online
      pressure_sensors[i].setModeActive();

      pressure_sensors[i].setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  
      pressure_sensors[i].setOversampleRate(0); // Set Oversample to 0 for fast reaction and small variations reading
      pressure_sensors[i].enableEventFlags(); // Enable all three pressure and temp event flags
    }
}
 
void loop() 
{
  
  for (uint8_t i = 0; i < 8; i++)
  {
    tcaselect(i);
    float pressure = pressure_sensors[i].readPressure();
    Serial.print(pressure);
    Serial.print(" ");
  }
  Serial.println();
}
