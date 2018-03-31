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

// Define DHT Sensor Type DHT11 / DHT21 / DHT22 : per anar bé haurien de ser DHT22 ja que tenen més precisió
#define DHTTYPE DHT22 


/****
 * PINS CONEXION
 ****/
 
// Pins conexion DHT22: temperature and humidity sensor
const int DHT1_Pin = 30;
const int DHT2_Pin = 40;

/**
// Pins for LDR 1 to 4 sensors 
#define LDR_sensor1_pin A3  //Segurament no s'utiltizarà aquest tipus de sensor...
#define LDR_sensor2_pin A4  //és massa poc precís...
#define LDR_sensor3_pin A5
#define LDR_sensor4_pin A6

**/

/*
// Pins for LDR laser 1 to 3 receivers
#define laser1_sensor_pin A0   // Passa a ser I2C, és a dir al a4, pero també s'ha de dir
#define laser2_sensor_pin A1    // que només en pot haver 2...així que haurem de mirar com ho fem
#define laser3_sensor_pin A2

*/

// Start lux sensor from BH1750 library

BH1750 ir_laser1(0x23); //Si el ADDR està inactiu

BH1750 ir_laser2(0x5C); //Si el ADDR està amb més de 0.7V

// Pins per connectar els emisors de llum laser

#define laser1_pin 44
#define laser2_pin 45

//#define laser3_pin 42


//Pin on es conectar el relé ONOFF de l'agitació
#define agitation_pin 41 //M'he inventat aquest PIN fran...no sé si seria correcte..

// Pin donde se conecta el bus 1-Wire
const int pinDatosDQ = 35; 



/****
 * TIME INTERVAL VARIABLES
 ****/

//Time interval to get metheo data
unsigned long last_get_metheo_data = 0;             
const unsigned long interval_get_metheo_data = 30L * 1000L;


//Time interval to get culture data
unsigned long last_get_culture_data = 0;             
const unsigned long interval_get_culture_data = 0L * 1000L;


//Time interval to get Optical Density data
unsigned long last_get_OD_data = 0;             
const unsigned long interval_get_OD_data = 30L * 1000L;

//Time of agitation on to get Optical Density
unsigned long t_needed_agitation_on = 2L*60L*1000L;
unsigned long t_needed_agitation_off = 15L*60L*1000L;


//Time interval to send data to server
unsigned long last_data_to_server = 0;             
const unsigned long interval_data_to_server = 30L * 1000L;


//Time interval to send data to SD card
unsigned long last_data_to_SD = 0;             
const unsigned long interval_data_to_SD = 30L * 1000L;


//Time interval to onoff_agitation
unsigned long last_onoff_agitation = 0;             
const unsigned long interval_1_onoff_agitation = 30L * 1000L;
const unsigned long interval_2_onoff_agitation = 60L * 1000L;



/****
 * 
 * CULTURE VARIABLES
 *
 ****/



/****
 * 
 *  GET OD DATA VARAIBLES
 *
 ****/


int laser_sensor1 = 0; //Valor de la irradiancia, així doncs millor anomenarlo ir, però com que això cambiar-ho s'ha de cambiar a molttsp uestos abans t'ho pregunto. 

//Irradiancia del laser 1
float ir1 = 0 ;
float ir10 = 500;  //Aquest valor s'hauria de contrastar un primer cop

//Irradiancia del laser 2
float ir2 = 0 ;
float ir20 = 500; //Aquest valor s'hauria de contrastar un primer cop


//Irradiancia del laser 3
float ir3 = 0 ;
float ir30 = 500; //Aquest valor s'hauria de contrastar un primer cop

// Waiting for opening laser
unsigned long wait_opening_laser = 2000;


/****
 * 
 * AGITATION VARIABLES
 *
 ****/



//Time measuring agitation is on
unsigned long t_agitation_on = 0L;
//Time measuring agitation is off
unsigned long t_agitation_off = 0L;

//
int temp_difference = 3; //La diferència de temperatura que fa que l'agitació comenci
int lux_min_agitation = 10; //Els mínims lux que ha d'haver per comenar a agitar. 
int lux_max_agitation = 50000; //Els lux perque l'agitació estigui en continuo.



/***
 * prendre varies mesures
 ***/
int samples_number = 8;



// Instancia a las clases OneWire y DallasTemperature
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);


// Debug mode for verbose info on serial monitor
boolean debug = true;

/*****

AUTHENTICATION ARDUINO

*****/
//Define de identity of Arduino
const int id_arduino = 1;


/*****
Global variables for internal use
 *****/

//Irradiancia for get_OD_function
 unsigned long iir = 0L; 
 int iir1 = 0;

//Temperature
float temp1; 
float temp2; 
float temp3; 
float temp4; 
float temp5; 
float ambient1; 
float ambient2;




 
// File handler
File myFile;
int fileCount = 0;
String fileName = "";
// RTC DS3231 (clock sensor)
//RTC_DS3231 rtc; not needed now
// Initialize the network library instance:
EthernetClient client;
// DHT Temp/Humity
DHT dht1(DHT1_Pin, DHTTYPE);
DHT dht2(DHT2_Pin, DHTTYPE);

/****
 * Per agafar l'hora, de moment no ho farem anar...

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


**********/

void setup()
{
  // start serial port:
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
    
    
 //Establint modo output per als lasers
 pinMode(laser1_pin, OUTPUT);
 pinMode(laser2_pin, OUTPUT);
 //pinMode(laser3_pin, OUTPUT);
 //Establint modo output per l'agitació
 pinMode(agitation_pin, OUTPUT);

 //start BH1740 light sensor

 Wire.begin();
 
 if (ir_laser1.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {
    Serial.println(F("light BH1750 sensor 1 started"));
 }
 else {
    Serial.println(F("Error initialising light sensor 1 BH1750"));
 }
 
 if (ir_laser2.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {
    Serial.println(F("light BH1750 sensor 2 started"));
 }
 else {
    Serial.println(F("Error initialising light sensor 2 BH1750"));
 }

  
1
  // Initialize SD Card
  Serial.println("Initializing SD card...");

  if (!SD.begin(4))
  {
    Serial.println("Initialization SD failed!");
    //return;
  }
  else
  {
    Serial.println("Initialization SD done.");
  }
/*
  // Comprobamos si tenemos el RTC conectado
  if (!rtc.begin())
  {
    Serial.println("No clock working");
    while (1)
      ;
  }

  // Setting RTC time for first time programing RTC
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

 // Serial.println(getDateTime());
*/
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
    Serial.println("Initializing DS18B20 BUS...");
  sensorDS18B20.begin();


  // Initialize DHT Temp/humidity sensors
  if(debug)
    Serial.println("Initializing DHT Sensor 1...");
  dht1.begin();
  //Serial.println("Fallo DHT_1");
  
  if(debug)
    Serial.println("Initializing DHT Sensor 2...");
  dht2.begin();
  //Serial.println("Fallo DHT_2");



  // give the ethernet module time to boot up:
  delay(2000);
}



// this method makes a HTTP connection to the server:
void data_to_server()
{
  //Read values before send to server
  
  // Requests culture temperatures from oneWire Bus
  sensorDS18B20.requestTemperatures();
  temp1 = lecturaTemperatura(0);
  temp2 = lecturaTemperatura(1);
  temp3 = lecturaTemperatura(2);
  temp4 = lecturaTemperatura(3);
  temp5 = lecturaTemperatura(4);

  // Request  Ambient temperature
  ambient1 = dht1_temp();
  ambient2 = dht2_temp();
  // ! Borrar la seguent linea, ja que dona error si no hi ha sensor connectat !
  ambient2 = 0;

/*
  // Read Lux LDR Sensors
  int lux_sensor1 = ldr1_lux();
  int lux_sensor2 = ldr2_lux();
  int lux_sensor3 = ldr3_lux();
  int lux_sensor4 = ldr4_lux();
*/

    // note the time that the connection was made:
    last_data_to_server = millis();
  }
  else
  {
    // if you couldn't make a connection:
    Serial.println("Connection Failed");
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

void data_to_SD()
{
    // Writting results to file
    myFile = SD.open(fileName, FILE_WRITE);

    // if the file opened okay, write to it:
    if (myFile)
    {
      if (debug)
        Serial.println("Writing to: " + fileName);

      //DateTime from RTC
      //myFile.print(getDateTime());
      
      //Temperatures del cultiu 
      //Sensor Temperatura 1
      myFile.print('#');
      myFile.print("sensor_1:");
      myFile.print(temp1);
      //Sensor Temperatura 2
      myFile.print('#');
      myFile.print("sensor_2:");
      myFile.print(temp2);
      //Sensor Temperatura 3
      myFile.print('#');
      myFile.print("sensor_3:");
      myFile.print(temp3);
      //Sensor Temperatura 4
      myFile.print('#');
      myFile.print("sensor_4:");
      myFile.print(temp4);
      //Sensor Temperatura 5
      myFile.print('#');
      myFile.print("sensor_5:");
      myFile.print(temp5);
      
      //Sensor Temperatura Ambient 1
      myFile.print('#');
      myFile.print("ambient_1:");
      myFile.print(ambient1);
      //Sensor Temperatura Ambient 2
      myFile.print('#');
      myFile.print("ambient_2:");
      myFile.println(ambient2);      
      
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
    Serial.print("iradiancia 1: ");
    Serial.println(ir1);
    Serial.print("iradiancia 2: ");
    Serial.println(ir2);
}


float laser1()
{
digitalWrite(laser1_pin, HIGH);
  // Llegim valors amb el laser obert
  delay(wait_opening_laser);
  iir1=0;
  iir=0;
  for (int i=0; i<samples_number; i++) 
  {
    iir1 = ir_laser1.readLightLevel();
    iir = iir1 + iir;
    delay(500);
  }
  digitalWrite(laser1_pin, LOW);
  float mean =  iir / samples_number;
  return mean;

}


float laser2()
{
digitalWrite(laser2_pin, HIGH);
  // Llegim valors amb el laser obert
  delay(wait_opening_laser);
  iir1=0;
  iir=0;
  for (int i=0; i<samples_number; i++) 
  {
    iir1 = ir_laser1.readLightLevel();
    iir = iir1 + iir;
    delay(500);
  }
  digitalWrite(laser2_pin, LOW);
  float mean =  iir / samples_number;
  return mean;

}

/*

int laser3(){
  // Llegim valors amb el laser tancat
  int LDRRead_low = analogRead(laser3_sensor_pin);
  delay(500);
  // Encenem laser
  digitalWrite(laser3_pin, HIGH);
  // Llegim valors amb el laser obert
  int LDRRead_high = analogRead(laser3_sensor_pin);
  delay(500);
  // Tanquem els lasers
  digitalWrite(laser3_pin, LOW);
  // Calculem valor mitja entre els 2 valors.
  // Aqui aplicar el calcul necessari
  int mean = ( LDRRead_low + LDRRead_high ) / 2;
  return mean;
}

*/
/*
int ldr1_lux(){
  return analogRead(LDR_sensor1_pin);
}

int ldr2_lux(){
  return analogRead(LDR_sensor2_pin);
}

int ldr3_lux(){
  return analogRead(LDR_sensor3_pin);
}

int ldr4_lux(){
  return analogRead(LDR_sensor4_pin);
}

*/

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




void loop()
{
  //Function sequence
      
  if (millis()- last_get_metheo_data > interval_get_metheo_data)
  {
      get_metheo_data();
      last_get_metheo_data = millis();
  }

  if (millis()-last_get_culture_data > interval_get_culture_data)
  {
    get_culture_data();
    last_get_culture_data = millis();
  }

  if (millis()-last_get_OD_data > interval_get_culture_data)
  
  {
    get_OD_data();
    last_get_OD_data = millis();
  }

  if (millis()-last_data_to_server > interval_data_to_server)
  {
    data_to_server();
    last_data_to_server = millis(); 
  }

  if (millis()-last_data_to_SD > interval_data_to_SD)
  {
    // Write data sensors to SD
    data_to_SD();

    last_data_to_SD = millis(); 
  }


  if (millis()-last_onoff_agitation > interval_1_onoff_agitation)
  {

    onoff_agitation();
    last_onoff_agitation = millis();
  }
     

}










