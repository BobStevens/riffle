/* Riffle Gas (Methane) with GPS data logger
 * https://publiclab.org/wiki/riffle
 *
 * MQ-4/MQ-6 or similar gas sensor
 * GP-735 TTL-UART GPS module
 * 
 * This program will continously log the analog reading from an MQ-4 gas sensor
 * module along with GPS coordinates and RTC datestamp. The analog reading
 * from the Riffle battery voltage divider on A3 is also recorded. The MQ-4 gas
 * sensor utilizes a built-in heater which is powered by the battery. Changes
 * in battery voltage may affect the sensor's analog reading value.
 * The data is appended to a CSV text file on the microSD card.
 *  
 * The blue LED on the Riffle blinks at a rate proportional to the sensor value.
 * A failed GPS fix is signaled by a series of four short blinks. Data will
 * continue to be logged without the GPS data.
 * A failed SD Card is signaled by rapid blinking at boot up.
 *
 * Communication with the GPS is through the MicroNMEA library because it seems
 * to use less resources on the ATmega328P than other GPS libraries.
 * 
 * Riffle 2x7 connector <-->  MQ-4 sensor board
 *  1) GND ------------------ HEATER GND
 *  3) 3v3 ------------------ A1
 * 13) A0  ------------------ B1 (see note)
 * 14) VBAT------------------ HEATER 1
 * Note: Pin B1 on the MQ-4 sensor module needs a 10K pulldown to GND.
 * 
 * Riffle 2x7 connector <-->  GP-735 GPS module
 *  1) GND ------------------ GND
 *  3) 3v3 ------------------ VDD
 *  5) D3  ------------------ TX
 *  7) D2  ------------------ RX
 *     NC  ------------------ VBAT
 *  3) 3v3 ------------------ EN
 *
 *
 */


#include <SD.h>
// GPS on alternate serial port
#include <SoftwareSerial.h>
#include <MicroNMEA.h>      // https://github.com/stevemarple/MicroNMEA
#include <DS3232RTC.h>        //http://github.com/JChristensen/DS3232RTC
#include <Time.h>             //https://github.com/PaulStoffregen/Time

// Logging
#define LOG_FILE "GAS-16.CSV"
// delay between data logging events in milliseconds
#define SENSOR_DELAY_MS 10000

// Riffle Status LED
#define LED 9

// SDcard
#define chipSelect 7
#define SDpower 6

// GPS communication
#define GPS_RX 2
#define GPS_TX 3
#define GPS_BAUD 9600

// analog sensor reading
#define GAS_IN A0
unsigned int sensorValue;

// analog battery level reading
#define BATTERY_IN A3

// main LED state
#define BLINK_DELAY_MS 1000
bool ledValue = 1;

// GPS error LED blinks
#define NO_GPS_BLINKS 4
#define NO_GPS_LED_ON_MS 50
#define NO_GPS_LED_OFF_MS 100
signed int no_gps_led_state = NO_GPS_BLINKS;

// log file
File dataFile;
time_t myTime;

// software serial for TTL-UART GPS module
SoftwareSerial gps = SoftwareSerial(GPS_TX, GPS_RX);
char nmeaBuffer[85];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// data to write to file
String dataString;

// pad single digit months/hours/seconds/etc with a leading zero
static String padDigits(int digits);

// debug / free RAM
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void gpsHardwareReset()
{
  // Empty input buffer
  while (gps.available())
    gps.read();
  
  digitalWrite(LED, LOW);
  delay(50);
  digitalWrite(LED, HIGH);

  // Reset is complete when the first valid message is received
  while (1) {
    while (gps.available()) {
      char c = gps.read();    
      if (nmea.process(c))
        digitalWrite(LED, LOW);
        return;
    }
  }
}

void setup()
{
  // sets the serial port for debuging
  Serial.begin(115200);
  Serial.println(F("Sniffle Gas Sensor with GPS"));

  pinMode(LED, OUTPUT);

  Serial.print(F("Initializing SD card..."));
  pinMode(SDpower,OUTPUT);
  digitalWrite(SDpower,LOW);
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // indicate SD problem with fast blink
    // this is a critical error
    while(1) {
      digitalWrite(LED,HIGH);
      delay(50);
      digitalWrite(LED,LOW);
      delay(100);
    }
  }
  Serial.println(F("Card initialized."));
  Serial.print(F("Logging to file: "));
  Serial.println(LOG_FILE);

  // initialize the GPS
  Serial.print(F("Initializing GPS serial port..."));
  gps.begin(GPS_BAUD);
  Serial.println(F("GPS serial port initialized."));
  Serial.println(F("Resetting GPS module ..."));
  gpsHardwareReset();
  Serial.println(F("GPS reset."));
  
  // Compatibility mode off
  // MicroNMEA::sendSentence(gps, "$PONME,2,6,1,0");
  MicroNMEA::sendSentence(gps, "$PONME,,,1,0");

   // Clear the list of messages which are sent.
  MicroNMEA::sendSentence(gps, "$PORZB");

  // Send RMC and GGA messages.
  MicroNMEA::sendSentence(gps, "$PORZB,RMC,1,GGA,1");
  // MicroNMEA::sendSentence(gps, "$PORZB,RMC,1,GGA,1,GSV,1");
  
  // reserve memory for the datastring, to reduce memory fragmentation
  dataString.reserve(100);
}

void loop()
{

  static unsigned long blink_ms = millis();
  static unsigned long sensor_ms = millis();

  // wait SENSOR_DELAY_MS for next logging attempt
  if (millis() - sensor_ms > SENSOR_DELAY_MS) {

    // put all values in the dataString, which will be written to the logfile
    dataString = "";

    // if a sentence is received, we can check the checksum, parse it...
    while (gps.available()) {
      char c = gps.read();
      Serial.print(c);
      nmea.process(c);
    }
    Serial.println();

    // timestamp
    myTime = RTC.get();
    dataString += padDigits(year(myTime));
    dataString += "-";
    dataString += padDigits(month(myTime));
    dataString += "-";
    dataString += padDigits(day(myTime));
    dataString += "T";
    dataString += padDigits(hour(myTime));
    dataString += ":";
    dataString += padDigits(minute(myTime));
    dataString += ":";
    dataString += padDigits(second(myTime));
    dataString += ",";

    // GPS
    if (nmea.isValid()) {
      // lat/lon
      dataString += String(nmea.getLatitude()/1000000, 8);
      dataString += ',';
      dataString += String(nmea.getLongitude()/1000000, 8);
      dataString += ',';
      
      long alt;
      dataString += String(nmea.getAltitude(alt)/1000, 8);
      dataString += ',';
    } else {
      // No GPS data
      dataString += F("NoLAT,NoLON,NoALT,");
    }

    // Analog sensor reading, pin A0
    sensorValue = analogRead(GAS_IN);
    dataString += String(sensorValue, DEC);
    dataString += ',';

    // Battery voltage reading, pin A3
    dataString += String(analogRead(BATTERY_IN), DEC);

    // print to the serial port too
    Serial.println(dataString);

    // if the file is available, write to it:
    File dataFile = SD.open(LOG_FILE, FILE_WRITE);
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    } else {
      // if the file isn't open, pop up an error:
      Serial.print(F("Error opening file: "));
      Serial.println(LOG_FILE);
    }
    // debug free RAM
    Serial.print(F("freeRam: "));
    Serial.println(freeRam());
    // ok to reset sensor delay
    sensor_ms = millis();

  }

  // no GPS data, blink the LED pattern
  if (!nmea.isValid()) {
    if (no_gps_led_state > 0) {
      if ( (ledValue && (millis() - blink_ms) > NO_GPS_LED_ON_MS)
        || (!ledValue && (millis() - blink_ms) > NO_GPS_LED_OFF_MS) ) {
        // decrement blink counter
        if (ledValue) no_gps_led_state --;
        // toggle it
        ledValue = !ledValue;
        // reset timer
        blink_ms = millis();
      }
    } else {
      ledValue = 0;
      if (millis() - blink_ms > BLINK_DELAY_MS) {
        // Serial.println(F("No GPS data"));
        // reset blink counter
        no_gps_led_state = NO_GPS_BLINKS;
        blink_ms = millis();
      }
    }
  } else {
    // toggle the LED based on sensor value
    // Analog sensor reading, pin A0
    sensorValue = analogRead(GAS_IN);
    if (sensorValue < 1024) {
      if (millis() - blink_ms > (1024 - sensorValue) ) {
        ledValue = !ledValue;
        blink_ms = millis();
      }
    } else {
      ledValue = 1;
    }
  }
  digitalWrite(LED, ledValue);

}

// utility function for time display: adds leading 0
// returns string
String padDigits(int digits){
  String s = "";
  if(digits < 10)
    s += "0";
    String("0" + digits);
  s += digits;
  return s;
}

