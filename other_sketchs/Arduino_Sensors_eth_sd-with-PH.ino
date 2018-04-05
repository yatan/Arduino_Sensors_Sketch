/*
 * Send Basic Sensors Data To SD and Ethernet
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
#include <Ethernet.h>
#include "RTClib.h"
#include <Wire.h>
#include <DHT.h>
#include <DallasTemperature.h>
#include <BH1750.h>

// Debug mode for verbose info on serial monitor
boolean debug = true;

/*****

AUTHENTICATION ARDUINO

*****/

// Define de identity of Arduino
const int id_arduino = 10;


/****
 * PINS CONEXION
 ****/

/*

DHT x 2 (Temperature / Humidity)
DS18B20 x 3 (Temperature)
BH1750 x 1 (Lux Sensor)
PH Sensor x 1
------------------
SD Card Reader
RTC DS3231
------------------
Ethernet

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

// Pin laser for lux sensor
#define laser_pin 5

// Pin analog PH Sensor
#define PHSensor_pin A0


// Instancia a las clases OneWire y DallasTemperature
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);

// Start lux sensor from BH1750 library using 0x23 address
BH1750 ir_laser1(0x23); 



/****
 * TIME INTERVAL VARIABLES
 ****/

//Time interval to send data to SD card
unsigned long last_data = 0;             
const unsigned long interval_data = 15L * 60L * 1000L; // 15 minuts * 60 segons * 1000 ms


// File handler
File myFile;
int fileCount = 0;
String fileName = "";

// RTC DS3231 (clock sensor)
RTC_DS3231 rtc;
// DHT Temp/Humity
DHT dht1(DHT1_Pin, DHTTYPE);
DHT dht2(DHT2_Pin, DHTTYPE);
// Network library
EthernetClient client;


/****

NETWORK SETTINGS 

****/

// Server connect for sending data
char server[] = "sensors.openspirulina.com";

// assign a MAC address for the ethernet controller:
byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE};
// fill in an available IP address on your network here,
// for MANUAL configuration:
IPAddress ip(192, 168, 1, 177);
// the router's gateway address:
IPAddress gateway(192, 168, 1, 1);
// the subnet:
IPAddress subnet(255, 255, 255, 0);
// fill in your Domain Name Server address here:
IPAddress myDns(8, 8, 8, 8);


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
 
  if (ir_laser1.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {
    if (debug)
      Serial.println(F("light BH1750 sensor 1 started"));
  }
  else {
    if (debug)
      Serial.println(F("Error initialising light sensor 1 BH1750"));
  }
 
 // Initialize SD Card
  if (debug)
    Serial.println(F("Initializing SD card..."));

  if (!SD.begin(sd_card_pin))
  {
    if (debug)
      Serial.println(F("Initialization SD failed!"));
  }
  else
  {
    if (debug)
      Serial.println(F("Initialization SD done."));
  }

  // Comprobamos si tenemos el RTC conectado
  if (!rtc.begin())
  {
    if (debug)
      Serial.println(F("No clock working"));
  }

  // Setting RTC time for first time programing RTC
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  if (debug)
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

  if (debug)
    Serial.println("Writing in file: " + fileName);
  

  // Initialize DS18B20 Temperature sensors
  if (debug)
    Serial.println(F("Initializing DS18B20 BUS..."));
  sensorDS18B20.begin();


  // Initialize DHT Temp/humidity sensors
  if (debug)
    Serial.println(F("Initializing DHT Sensor 1..."));
  dht1.begin();
  
  if (debug)
    Serial.println(F("Initializing DHT Sensor 2..."));
  dht2.begin();

  // Writting title headers to file
  myFile = SD.open(fileName, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile)
  {
    myFile.print(F("DateTime#"));
    myFile.print(F("sensor_1#"));
    myFile.print(F("sensor_2#"));
    myFile.print(F("sensor_3#"));
    myFile.print(F("ambient_1_temp#"));
    myFile.print(F("ambient_2_temp#"));  
    myFile.print(F("ambient_1_humetat#"));  
    myFile.print(F("ambient_2_humetat#"));        
    myFile.print(F("lux#"));
    myFile.println(F("PH"));
    
    // close the file:
    myFile.close();
  }

  delay(2000);
  // start the Ethernet connection using a fixed IP address and DNS server:
  // Ethernet.begin(mac, ip, myDns, gateway, subnet);
  // DHCP IP ( For automatic IP )
  Ethernet.begin(mac);
  
  // print the Ethernet board/shield's IP address:
  if(debug)
  {
    Serial.print("My IP address: ");
    Serial.println(Ethernet.localIP());
  }
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

  // PH Sensor
  int phSuma = 0;
  for(int i=0; i<5; i++)
  {
    phSuma += analogRead(PHSensor_pin);
    delay(10);
  }

  float PH_mitja = (float)phSuma / 5;
  float PH_voltage = PH_mitja * (5.0 / 1023.0);
  // Adaptació a la recta y=mx+b  y = -5.70 * PH_V + 21.34
  float ph_Value = -5.70 * PH_voltage + 21.34;
  

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

  // Request Lux light
  unsigned long iir=0;

  for (int i=0; i<3; i++) 
  {
    unsigned long iir1 = ir_laser1.readLightLevel();
    iir += iir1;
    delay(50);
  }

  lux_sensor1 = iir / 3;

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
    myFile.print(array_temp1);
    //Sensor Temperatura 2
    myFile.print('#');
    myFile.print(array_temp2);
    //Sensor Temperatura 3
    myFile.print('#');
    myFile.print(array_temp3);
    
    //Sensor Temperatura Ambient 1
    myFile.print('#');
    myFile.print(ambient1_temp);
    //Sensor Temperatura Ambient 2
    myFile.print('#');
    myFile.println(ambient2_temp);      

    //Sensor Humetat Ambient 1
    myFile.print('#');
    myFile.print(ambient1_humetat);
    //Sensor Humetat Ambient 2
    myFile.print('#');
    myFile.print(ambient2_humetat);     

    //Lux sensor
    myFile.print('#');
    myFile.print(lux_sensor1);

    //PH values
    myFile.print('#');
    myFile.println(ph_Value);

    // close the file:
    myFile.close();

    if (debug)
      Serial.println(F("Writting SD done."));
  }
  else
  {
    // if the file didn't open, print an error:
    if (debug)
      Serial.println("Error opening: " + fileName);
  }

  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  client.stop();
  // if there's a successful connection:
  if (client.connect(server, 80))
  {
    if (debug)
      Serial.println("connecting...");
    // send the HTTP GET request:
    
    String cadena = "GET /afegir.php?temp1=";
    cadena += array_temp1;
    cadena += "&temp2=";
    cadena += array_temp2;
    cadena += "&temp3=";
    cadena += array_temp3;
    // Append Ambient temperatures
    cadena += "&ta1=";
    cadena += ambient1_temp;
    cadena += "&ta2=";
    cadena += ambient2_temp; 
    // Append Ambient humidity
    cadena += "&ha1=";
    cadena += ambient1_humetat;
    cadena += "&ha2=";
    cadena += ambient2_humetat;     

    // Append Light Sensor
    cadena += "&ldr1=";
    cadena += lux_sensor1;

    // Append PH Sensor Value
    cadena += "&ph1=";
    cadena += ph_Value;
    
    // Append our ID Arduino
    cadena += "&idarduino=";
    cadena += id_arduino;
    cadena += " HTTP/1.1";

    if(debug)
      Serial.println(cadena);
    
    // Send string to internet  
    client.println(cadena);
    
    client.println("Host: sensors.openspirulina.com");
    client.println("User-Agent: arduino-ethernet-1");
    client.println("Connection: close");
    client.println();

  }
  else
  {
    // if you couldn't make a connection:
    Serial.println("Connection Failed");
  }
}

void loop()
{

  if (millis()-last_data > interval_data)
  {
    // Llegir informacio dels sensors
    capture_data();
    last_data = millis(); 
  }    

}


