/*
 * Send Sensors Data To HTTP (PHP) Server
 * 
 * Based on ethernet web client sketch:
 * Repeating Web client
 * http://www.arduino.cc/en/Tutorial/WebClientRepeating
 */

#include <SD.h>
#include <SPI.h>
#include <Ethernet.h>
#include <RTClib.h>
#include <Wire.h>

boolean debug = false;

// File handler
File myFile;
int fileCount = 0;
String fileName = "";

// Declaramos un RTC DS3231
RTC_DS3231 rtc;

// assign a MAC address for the ethernet controller.
// fill in your address here:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
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

unsigned long lastConnectionTime = 0;             // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 10L * 1000L; // delay between updates, in milliseconds
// the "L" is needed to use long type numbers

void setup() {
  // start serial port:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("Initialization SD failed!");
    return;
  }
  Serial.println("Initialization SD done.");

   // Comprobamos si tenemos el RTC conectado
   if (! rtc.begin()) {
   Serial.println("No hay un módulo RTC");
   while (1);
   }

   // Setting RTC time
   //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
   
   DateTime now = rtc.now();
   Serial.print(now.day());
   Serial.print('/');
   Serial.print(now.month());
   Serial.print('/');
   Serial.print(now.year());
   Serial.print(" ");
   Serial.print(now.hour());
   Serial.print(':');
   Serial.print(now.minute());
   Serial.print(':');
   Serial.print(now.second());
   Serial.println();


  //First free file, write into
  fileName += fileCount;
  fileName += ".txt";  
  while (SD.exists(fileName)){
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
  // print the Ethernet board/shield's IP address:
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
}

void loop() {
  // if there's incoming data from the net connection.
  // send it out the serial port.  This is for debugging
  // purposes only:
  if (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  // if ten seconds have passed since your last connection,
  // then connect again and send data:
  if (millis() - lastConnectionTime > postingInterval) {
    httpRequest();
  }

}

// this method makes a HTTP connection to the server:
void httpRequest() {
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  client.stop();

  // if there's a successful connection:
  if (client.connect(server, 80)) {
    Serial.println("connecting...");
    // send the HTTP GET request:
    int sensorReading = analogRead(0);
    
    String cadena = "GET /afegir.php?value=";
    cadena += sensorReading;
    cadena += " HTTP/1.1";
    client.println(cadena);
    
    client.println("Host: sensors.openspirulina.com");
    client.println("User-Agent: arduino-ethernet-1");
    client.println("Connection: close");
    client.println();


    
    // Writting results to file
    myFile = SD.open(fileName, FILE_WRITE);
    
    // if the file opened okay, write to it:
    if (myFile) {
      if(debug)
        Serial.println("Writing to: " + fileName);
      myFile.print("Sensor read: ");
      myFile.println(sensorReading);
      // close the file:
      myFile.close();
      if(debug)
        Serial.println("Writting SD done.");
    } else {
      // if the file didn't open, print an error:
      Serial.println("Error opening test.txt");
    }

    
    // note the time that the connection was made:
    lastConnectionTime = millis();
  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}
