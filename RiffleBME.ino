/*
RiffleBME.ino
For a Riffle and one BME280 sensor
Includes function to read temperatures from the RTC below 0C
Displays results on the serial monitor
Blinks the blue LED twice when writing to SD and once for each logging event
C Fastie January 2018
*/
#include <Wire.h>                             // https://www.arduino.cc/en/Reference/Libraries
#include <RTClib.h>                           // https://github.com/adafruit/RTClib                           
#include <SPI.h>                              // https://www.arduino.cc/en/Reference/Libraries
#include <SdFat.h>                            // https://github.com/greiman/SdFat/
#include <Adafruit_BME280.h>                  // https://github.com/adafruit/Adafruit_BME280_Library

SdFat SD;                                     // SD will be the SdFat object
#define chipSelect 7                          // for SD card
RTC_DS3231 RTC;                               // RTC will be the RTC_DS3231 object
#define DS3231_I2C_ADDRESS 0x68               // RTC address
Adafruit_BME280 bme;                          // I2C

float BMEt;                                   // variables for the sensor:
float BMEh;
float BMEp;
float BMEa;
long utc;                                     // variables for the RTC:
float rtcTemp;   
float rtcTempC;      
byte tMSB = 0;
byte tLSB = 0;  
char TmeStrng[] = "0000/00/00,00:00:00";      // string template for RTC time stamp
File dataFile;

void setup(void) {
  pinMode(9, OUTPUT); 
  Serial.begin(9600);
  delay(100);

  Serial.println(F("BME280 test"));  
  bool status;                                      // initialize the BME  
  status = bme.begin(0x76);
  delay(100);
    if (!status) {
        Serial.println("Could not find BME280");
    }   
   Serial.print("Initializing SD card...");         // initialize the SD  
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
    }
    else {
       Serial.println("card initialized.");
    }
  Serial.println("");
     RTC.begin();                                    // initialize the RTC  
  // Set the RTC time to the time this sketch was last compiled. Uncomment the following line, load the 
  // sketch on the logger. Must comment out the line and reload for subsequent boots or the RTC time will be wrong:
  // RTC.adjust(DateTime((__DATE__), (__TIME__))); 
}                                           // end of setup

void loop() {
    DateTime now = RTC.now();               // read the time from the RTC, then construct a data string:
    sprintf(TmeStrng, "%04d/%02d/%02d,%02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());  
    utc = (now.unixtime());                 // time code (good for graphing) 
    rtcTemp = rtcMinus();                   // function to capture RTC temperatures below 0C

    BMEt = (bme.readTemperature());         // Read the BME sensor
    BMEh = (bme.readHumidity());
    BMEp = (bme.readPressure() / 100.0F); 
    
    File dataFile = SD.open("datalogTPL.txt", FILE_WRITE);       
  if (dataFile) {                           // write the data to the SD card:
                
          dataFile.print(TmeStrng);dataFile.print(",");
          dataFile.print(utc);dataFile.print(",");
          dataFile.print(rtcTemp);dataFile.print(",");
          dataFile.print(BMEt);dataFile.print(",");
          dataFile.print(BMEh);dataFile.print(",");
          dataFile.println(BMEp);    
          dataFile.flush();               
          dataFile.close();
          
    digitalWrite(9, HIGH);   // Blink twice when writing to SD card...
    delay(200);              // wait for a bit
    digitalWrite(9, LOW);    // turn the LED off by making the voltage LOW
    delay(200);
    digitalWrite(9, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(200);              // wait 
    digitalWrite(9, LOW);    // turn the LED off by making the voltage LOW
    delay(1000); 
  }
    Serial.println(TmeStrng);
    Serial.println(utc);
    Serial.print("RTC Temperature = ");
    Serial.print(rtcTemp);
    Serial.println(" *C");   
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");
    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
    Serial.println();
    
    digitalWrite(9, HIGH);   // Blink once per logging event
    delay(500);              // wait for a bit
    digitalWrite(9, LOW);    // turn the LED off by making the voltage LOW
    delay(297900);           // (300k millis) - (sum of all other delays) = (5 minute logging interval)
}                            // end of main loop

float rtcMinus() {                                    // allows for RTC temperatures below 0C
    Wire.beginTransmission(DS3231_I2C_ADDRESS); 
    Wire.write(0x11);                                 // the register where the temp data is stored
    Wire.endTransmission();
    Wire.requestFrom(DS3231_I2C_ADDRESS, 2);          // ask for two bytes of data
    if (Wire.available()) 
    {
       tMSB = Wire.read();                            // 2s complement int portion
       tLSB = Wire.read();                            // fraction portion
       rtcTempC = ((((short)tMSB << 8) | (short)tLSB) >> 6) / 4.0; 
    }
    return rtcTempC;
}                                                     // end of function