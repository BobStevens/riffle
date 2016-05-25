/* Riffle datalogging with BMP180 barometric pressure sensor
 * https://publiclab.org/wiki/riffle
 * https://www.adafruit.com/product/1603
 * 
 * This program will continously log the barometric pressure, altitude, and 
 * temperature from the BMP180 sensor along with an RTC datestamp 
 * and RTC temperature reading. The values are stored in a CSV file on the
 * microSD card. The blue LED on digital pin 9 blinks at 1Hz when functioning 
 * properly.
 * 
 * 
 * Riffle 2x7 connector <--> BMP180 sensor board
 *  1) GND ------------------ GND
 *  3) 3v3 ------------------ VIN
 *  9) SDA ------------------ SDA
 * 11) SCL ------------------ SCL
 *
 */
 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#include <DS3232RTC.h>        // http://github.com/JChristensen/DS3232RTC
#include <Time.h>             // https://github.com/PaulStoffregen/Time
#include <SPI.h>
#include <SD.h>

#define LED 9
#define chipSelect 7
#define SDpower 6

// Sensor reading delay
#define SENSOR_MS 1000

// Sea level pressure for accurate altitude calculations
// #define SEALEVEL_HPA SENSORS_PRESSURE_SEALEVELHPA
#define SEALEVEL_HPA 1014.5

// LED state
bool ledValue = 1;
#define LEDON_MS 10
#define LEDOFF_MS 990

File dataFile;
time_t myTime;
   
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// Displays some basic information on this sensor from the unified
//sensor API sensor_t type (see Adafruit_Sensor for more information)
void displaySensorDetails(void) {
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

// utility function to add leading zeros to single digit numbers / clock
String printDigits(int digits) {
  String s = "";
  if(digits < 10)
    s += "0";
    String("0" + digits);
  s += digits;
  return s;
}

void setup(void) {
  Serial.begin(9600);
  Serial.println("Pressure Sensor Test"); Serial.println("");
  
  // Initialise the sensor
  if (!bmp.begin()) {
    Serial.print("BMP085 not detected");
    while(1);
  }
  
  pinMode(LED, OUTPUT); 
  
  Serial.print("Initializing SD card...");
  pinMode(SDpower,OUTPUT);
  digitalWrite(SDpower,LOW);
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // indicate SD problem with fast blink
    while(1) {
      digitalWrite(LED,HIGH);
      delay(100);
      digitalWrite(LED,LOW);
      delay(50);
    }
  }
  Serial.println("card initialized.");
  
  
  // Display some basic information on this sensor
  displaySensorDetails();
}

void loop(void) {
  
  static unsigned long blink_ms = millis();
  static unsigned long sensor_ms = millis();
  
  // wait SENSOR_MS ms for next reading
  if (millis() - sensor_ms > SENSOR_MS) {
    
    File dataFile = SD.open("test.csv", FILE_WRITE);
    
    String dataString = "";
    myTime = RTC.get();
    
    dataString += String(printDigits(year(myTime)));
    dataString += String("-");
    dataString += String(printDigits(month(myTime)));
    dataString += String("-");
    dataString += String(printDigits(day(myTime)));
    dataString += String("T");
    dataString += String(printDigits(hour(myTime)));
    dataString += String(":");
    dataString += String(printDigits(minute(myTime)));
    dataString += String(":");
    dataString += String(printDigits(second(myTime)));
    
    dataString += String(",");
         
    // Get a new sensor reading 
    sensors_event_t event;
    bmp.getEvent(&event);
    if (event.pressure) {
  
      // Pressure hPa
      dataString += String(event.pressure, 2);
      dataString += String(",");
  
      // Altitude m
      dataString += String(bmp.pressureToAltitude(SEALEVEL_HPA ,event.pressure), 3);  
      dataString += String(",");
                                        
      // Temp 
      float temperature;
      bmp.getTemperature(&temperature);
      dataString += String(temperature, 1);
      dataString += String(",");
      
    } else {
      dataString += String("error,error,error");
      // error notification
      ledValue = 1;
    }
    
    int t = RTC.temperature();
    float celsius = t / 4.0;
    dataString += String(celsius, 1);
    
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      // print to the serial port
      Serial.println(dataString);
      dataFile.close();
    } else {
      // if the file isn't open, send an error:
      Serial.println("error opening data log");
    }

    // update sensor timer
    sensor_ms = millis();
  }
  
  // toggle the LED
  if  (ledValue) {
    if (millis() - blink_ms > LEDON_MS ) {
      ledValue = !ledValue;
      blink_ms = millis();
    }
  } else {
    if (millis() - blink_ms > LEDOFF_MS ) {
      ledValue = !ledValue;
      blink_ms = millis();
    }
  }
  digitalWrite(LED, ledValue);
  
}
