/*
 * Send Sensors Data To SD
 * 
 * Autor: Fran Romero https://github.com/yatan
 * 
 * Based on: 
 *  Arduino Sensors Sketch
 *  https://github.com/yatan/Arduino_Sensors_Sketch
 * 
 * 
 *
 */



//Libraries used

#include <SD.h>
#include <SPI.h>
#include "RTClib.h"
#include <Wire.h>
#include <DHT.h>
#include <DallasTemperature.h>
#include <BH1750.h>

// Debug mode for verbose info on serial monitor
boolean debug = true;


/****
 * PINS CONEXION
 ****/

/*

DHT x 2 (Temperature / Humidity)
DS18B20 x 3 (Temperature)
BH1750 x 1 (Lux Sensor)
------------------
SD Card Reader
RTC DS3231

*/

// Define DHT Sensor Type DHT11 / DHT21 / DHT22 : per anar bé haurien de ser DHT22 ja que tenen més precisió
#define DHTTYPE DHT22 
 
// Pins conexion DHT22: temperature and humidity sensor
#define DHT1_Pin 7
#define DHT2_Pin 8

// Pin donde se conecta el bus 1-Wire
#define pinDatosDQ 2
// Pin lector SD
#define sd_card_pin 4


// Instancia a las clases OneWire y DallasTemperature
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);

/****
 * TIME INTERVAL VARIABLES
 ****/

//Time interval to send data to SD card
unsigned long last_data_to_SD = 0;             
const unsigned long interval_data_to_SD = 60L * 1000L; // 60 segons * 1000 ms


// File handler
File myFile;
int fileCount = 0;
String fileName = "";

// RTC DS3231 (clock sensor)
RTC_DS3231 rtc;
// DHT Temp/Humity
DHT dht1(DHT1_Pin, DHTTYPE);
DHT dht2(DHT2_Pin, DHTTYPE);


String getDateTime()
{
  DateTime now = rtc.now();
  String hora = "";

  hora += now.day();
  hora += '/';
  hora += now.month();
  hora += '/';
  hora += now.year();
  hora += " ";
  hora += now.hour();
  hora += ':';
  hora += now.minute();
  hora += ':';
  hora += now.second();

  return hora;
}


void setup()
{
  // start serial port:
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
    
    
// Start BH1750 light sensor

 Wire.begin();
 
 // Initialize SD Card
 Serial.println(F("Initializing SD card..."));

  if (!SD.begin(sd_card_pin))
  {
    Serial.println(F("Initialization SD failed!"));
    //return;
  }
  else
  {
    Serial.println(F("Initialization SD done."));
  }

  // Comprobamos si tenemos el RTC conectado
  if (!rtc.begin())
  {
    Serial.println(F("No clock working"));
  }

  // Setting RTC time for first time programing RTC
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  Serial.println(getDateTime());

  // Obtain a free file name for writting to SD
  fileName += fileCount;
  fileName += ".txt";
  while (SD.exists(fileName))
  {
    fileCount += 1;
    fileName = "";
    fileName += fileCount;
    fileName += ".txt";
  }
  if(debug)
    Serial.println("Writing in file: " + fileName);


  // Initialize DS18B20 Temperature sensors
  if(debug)
    Serial.println(F("Initializing DS18B20 BUS..."));
  sensorDS18B20.begin();


  // Initialize DHT Temp/humidity sensors
  if(debug)
    Serial.println(F("Initializing DHT Sensor 1..."));
  dht1.begin();
  //Serial.println("Fallo DHT_1");
  
  if(debug)
    Serial.println(F("Initializing DHT Sensor 2..."));
  dht2.begin();
  //Serial.println("Fallo DHT_2");

  delay(1000);
}




// Read values and store to variables
void capture_data()
{

  // Temperature DS18B20 array sensors
  float array_temp1; 
  float array_temp2; 
  float array_temp3; 

  // DHT22 sensors
  float ambient1_temp; 
  float ambient1_humetat;
  float ambient2_temp;
  float ambient2_humetat;

  // BH1750 
  float lux_sensor1;


  // Requests culture temperatures from oneWire Bus
  sensorDS18B20.requestTemperatures();
  array_temp1 = sensorDS18B20.getTempCByIndex(0);
  array_temp2 = sensorDS18B20.getTempCByIndex(1);
  array_temp3 = sensorDS18B20.getTempCByIndex(2);

  // Request Ambient temperature
  ambient1_temp = dht1.readTemperature();
  ambient2_temp = dht2.readTemperature();

  // Request Ambient Humidity
  ambient1_humetat = dht1.readHumidity();
  ambient2_humetat = dht2.readHumidity();


  // Writting results to file
  myFile = SD.open(fileName, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile)
  {
    if (debug)
      Serial.println("Writing to: " + fileName);

    //DateTime from RTC
    myFile.print(getDateTime());
    
    //Temperatures del cultiu 
    //Sensor Temperatura 1
    myFile.print('#');
    myFile.print("sensor_1:");
    myFile.print(array_temp1);
    //Sensor Temperatura 2
    myFile.print('#');
    myFile.print("sensor_2:");
    myFile.print(array_temp2);
    //Sensor Temperatura 3
    myFile.print('#');
    myFile.print("sensor_3:");
    myFile.print(array_temp3);
    
    //Sensor Temperatura Ambient 1
    myFile.print('#');
    myFile.print("ambient_1_temp:");
    myFile.print(ambient1_temp);
    //Sensor Temperatura Ambient 2
    myFile.print('#');
    myFile.print("ambient_2_temp:");
    myFile.println(ambient2_temp);      

    //Sensor Humetat Ambient 1
    myFile.print('#');
    myFile.print("ambient_1_humetat:");
    myFile.print(ambient1_humetat);
    //Sensor Humetat Ambient 2
    myFile.print('#');
    myFile.print("ambient_2_humetat:");
    myFile.println(ambient2_humetat);     
    
    // close the file:
    myFile.close();
    if (debug)
      Serial.println("Writting SD done.");
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("Error opening: " + fileName);
  }

}

void loop()
{

  if (millis()-last_data_to_SD > interval_data_to_SD)
  {
    // Llegir informacio dels sensors
    capture_data();
    last_data_to_SD = millis(); 
  }    

}


