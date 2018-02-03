/*
 * Send Sensors Data To HTTP (PHP) Server
 * 
 * Autor: Fran Romero https://github.com/yatan
 * 
 * Based on: 
 * Ethernet web client sketch:
 *  Repeating Web client
 *  http://www.arduino.cc/en/Tutorial/WebClientRepeating
 *  
 * DS18B20 sensor de temperatura para líquidos con Arduino:
 *  https://programarfacil.com/blog/arduino-blog/ds18b20-sensor-temperatura-arduino/
 * 
 */

#include <SD.h>
#include <SPI.h>
#include <Ethernet.h>
#include "RTClib.h"
#include <Wire.h>
#include <DHT.h>

#include <DallasTemperature.h>

#define DHTTYPE DHT22

const int DHT1_Pin = 5;
const int DHT2_Pin = 6;

// Pin donde se conecta el bus 1-Wire
const int pinDatosDQ = 9;
 
// Instancia a las clases OneWire y DallasTemperature
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);


// Debug mode for verbose info on serial monitor
boolean debug = false;

/*****

AUTHENTICATION ARDUINO

*****/

const int id_arduino = 1;

/****

NETWORK SETTINGS 

****/

// Server connect for sending data
char server[] = "sensors.openspirulina.com";

// assign a MAC address for the ethernet controller:
byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
// fill in an available IP address on your network here,
// for manual configuration:
IPAddress ip(192, 168, 137, 177);
// the router's gateway address:
IPAddress gateway(192, 168, 137, 1);
// the subnet:
IPAddress subnet(255, 255, 255, 0);
// fill in your Domain Name Server address here:
IPAddress myDns(8, 8, 8, 8);

/*****

TIME INTERVAL

******/

unsigned long lastConnectionTime = 0;               // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 10L * 1000L;  // delay between updates, in milliseconds
                                                    // the "L" is needed to use long type numbers


/*****
Global variables for internal use
 *****/
 
// File handler
File myFile;
int fileCount = 0;
String fileName = "";
// RTC DS3231
RTC_DS3231 rtc;
// Initialize the network library instance:
EthernetClient client;
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

  // Initialize DS18B20 sensors
  if(debug)
    Serial.println("Initializing DS18B20 BUS...");
  sensorDS18B20.begin(); 

  // Initialize DHT sensors
  if(debug)
    Serial.println("Initializing DHT Sensor 1...");
  dht1.begin();
  
  if(debug)
    Serial.println("Initializing DHT Sensor 2...");
  dht2.begin();

  // Initialize SD Card
  Serial.println("Initializing SD card...");

  if (!SD.begin(4))
  {
    Serial.println("Initialization SD failed!");
    return;
  }
  Serial.println("Initialization SD done.");

  // Comprobamos si tenemos el RTC conectado
  if (!rtc.begin())
  {
    Serial.println("No hay un módulo RTC");
    while (1)
      ;
  }

  // Setting RTC time
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  Serial.println(getDateTime());

  //First free file, write into
  fileName += fileCount;
  fileName += ".txt";
  while (SD.exists(fileName))
  {
    fileCount += 1;
    fileName = "";
    fileName += fileCount;
    fileName += ".txt";
  }
  Serial.println("Writing in file: " + fileName);

  // give the ethernet module time to boot up:
  delay(2000);
  // start the Ethernet connection using a fixed IP address and DNS server:
  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  // DHCP IP ( For automatic IP )
  //Ethernet.begin(mac);
  
  // print the Ethernet board/shield's IP address:
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
}

void loop()
{
  // if there's incoming data from the net connection.
  // send it out the serial port.  This is for debugging
  // purposes only:
  if (client.available())
  {
    char c = client.read();
    Serial.write(c);
  }

  // if ten seconds have passed since your last connection,
  // then connect again and send data:
  if (millis() - lastConnectionTime > postingInterval)
  {
    httpRequest();
  }
}


 /****f* Arduino_Sensors_Sketch/writeDataToSD
  *  NAME
  *    writeDataToSD
  *  SYNOPSIS
  *    writeDataToSD( int sensor1, ... )
  *  FUNCTION
  *    When the sensor read a value and whant to save to any destination, we use this function to save that to 
  *    persisntent file on SD Card.
  *  INPUTS
  *    sensor1    - Readed value of sensor 1.
  *  RESULT
  *    Write the input values to SD Card on fileName.txt with current DateTime.
  ******
  */

void writeDataToSD(int sensor1, int sensor2, int sensor3, int sensor4, int sensor5)
{
    // Writting results to file
    myFile = SD.open(fileName, FILE_WRITE);

    // if the file opened okay, write to it:
    if (myFile)
    {
      if (debug)
        Serial.println("Writing to: " + fileName);

      myFile.print(getDateTime());
      //Sensor 1
      myFile.print('#');
      myFile.print("sensor_1:");
      myFile.print(sensor1);
      //Sensor 2
      myFile.print('#');
      myFile.print("sensor_2:");
      myFile.print(sensor2);
      //Sensor 3
      myFile.print('#');
      myFile.print("sensor_3:");
      myFile.print(sensor3);
      //Sensor 4
      myFile.print('#');
      myFile.print("sensor_4:");
      myFile.print(sensor4);
      //Sensor 5
      myFile.print('#');
      myFile.print("sensor_5:");
      myFile.println(sensor5);
      
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

float lecturaTemperatura(int posicio){
  return sensorDS18B20.getTempCByIndex(posicio);
}

float dht1_temp(){
  return dht1.readTemperature();
}

float dht1_humidity(){
  return dht1.readHumidity();
}

float dht2_temp(){
  return dht2.readTemperature();
}

float dht2_humidity(){
  return dht2.readHumidity();
}


// this method makes a HTTP connection to the server:
void httpRequest()
{
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  client.stop();

  //Read values before send to server
  
  // Requests aqua temperatures from oneWire Bus
  sensorDS18B20.requestTemperatures();
  float tempSensor1 = lecturaTemperatura(0);
  float tempSensor2 = lecturaTemperatura(1);
  float tempSensor3 = lecturaTemperatura(2);
  float tempSensor4 = lecturaTemperatura(3);
  float tempSensor5 = lecturaTemperatura(4);

  // Request  Ambient temperature
  float temp_ambient1 = dht1_temp();
  float temp_ambient2 = dht2_temp();

  // if there's a successful connection:
  if (client.connect(server, 80))
  {
    Serial.println("connecting...");
    // send the HTTP GET request:
    
    String cadena = "GET /afegir.php?temp1=";
    cadena += tempSensor1;
    cadena += "&temp2=";
    cadena += tempSensor2;
    cadena += "&temp3=";
    cadena += tempSensor3;
    cadena += "&temp4=";
    cadena += tempSensor4;
    cadena += "&temp5=";
    cadena += tempSensor5;
    // Append Ambient temperatures
    cadena += "&ta1=";
    cadena += temp_ambient1;
    cadena += "&ta2=";
    cadena += temp_ambient2;    
    // Append our ID Arduino
    cadena += "&idarduino=";
    cadena += id_arduino;
    cadena += " HTTP/1.1";
    client.println(cadena);

    client.println("Host: sensors.openspirulina.com");
    client.println("User-Agent: arduino-ethernet-1");
    client.println("Connection: close");
    client.println();

    writeDataToSD(tempSensor1, tempSensor2, tempSensor3, tempSensor4, tempSensor5);

    // note the time that the connection was made:
    lastConnectionTime = millis();
  }
  else
  {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}
