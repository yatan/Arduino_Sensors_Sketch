/*
 * Send Sensors Data To HTTP (PHP) Server
 * 
 * Autor: Fran Romero https://github.com/yatan
 * 
 * Based on ethernet web client sketch:
 * Repeating Web client
 * http://www.arduino.cc/en/Tutorial/WebClientRepeating
 */

#include <SD.h>
#include <SPI.h>
#include <Ethernet.h>
#include "RTClib.h"
#include <Wire.h>

boolean debug = false;

// File handler
File myFile;
int fileCount = 0;
String fileName = "";

// Declaramos un RTC DS3231
RTC_DS3231 rtc;

int id_arduino = 1;

// assign a MAC address for the ethernet controller.
// fill in your address here:
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

// initialize the library instance:
EthernetClient client;

char server[] = "sensors.openspirulina.com";

unsigned long lastConnectionTime = 0;              // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 10L * 1000L; // delay between updates, in milliseconds
// the "L" is needed to use long type numbers

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
      myFile.println(sensor1);
      //Sensor 2
      myFile.print('#');
      myFile.print("sensor_2:");
      myFile.println(sensor2);
      //Sensor 3
      myFile.print('#');
      myFile.print("sensor_3:");
      myFile.println(sensor3);
      //Sensor 4
      myFile.print('#');
      myFile.print("sensor_4:");
      myFile.println(sensor4);
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

int lecturaTemperatura1(){
  return analogRead(0);
}

int lecturaTemperatura2(){
  return analogRead(0);
}

int lecturaTemperatura3(){
  return analogRead(0);
}

int lecturaTemperatura4(){
  return analogRead(0);
}

int lecturaTemperatura5(){
  return analogRead(0);
}

// this method makes a HTTP connection to the server:
void httpRequest()
{
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  client.stop();

  //Read values before send to server

  int tempSensor1 = lecturaTemperatura1();
  int tempSensor2 = lecturaTemperatura2();
  int tempSensor3 = lecturaTemperatura3();
  int tempSensor4 = lecturaTemperatura4();
  int tempSensor5 = lecturaTemperatura5();

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