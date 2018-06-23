/*
 * INTRO
 *
 */


// Includes
#include <SD.h>
#include <SPI.h>
#include <Ethernet.h>
#include "RTClib.h"
#include <DHT.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <BH1750.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>


// Define Number of Sensors

const int num_T = 4;   // Temperature of the culture. Sensor DS18B20.MAX 6
						// T1_s T1_b
const int num_DHT = 1; //Humidity and temperature ambient sensor. MAX 3
#define DHTTYPE DHT22
const int num_PIR = 1;  //PIR movement sensor. MAX 3
const int num_DO = 1;   // Optical Density Sensor Module made by OpenSpirulina includes a RGB led + BH1705 lux sensor
const int option_lux = 2; // 0: No sensor. 1: ldr sensor. 2: lux BH1750
const int num_pH = 1;   //pH sensor. MAX 3
const boolean option_internet = modem or ethernet; //if ethernet conexion possible(=1) or not (=0)
const boolean option_LCD true; // if LCD 20x04 possible (=1) or not (=0)
const boolean option_SD = true;   //if SD connexion posible (=1) or not (=0)
const boolean option_clock = true; //if clock posible (=1) or not (=0)

// Debug mode for verbose info on serial monitor
boolean debug = true;

// Pins
#define pin_onewire 0    // where 1-wire is connected
// Pin lector SD
#define sd_card_pin 4
// DHT Pins
const int DHT_Pins = {7, 8};



// Global sensors variables
OneWire oneWireObjeto(pin_onewire);
DallasTemperature sensorDS18B20(&oneWireObjeto);
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
// RTC DS3231 (clock sensor)
RTC_DS3231 rtc;
// Array of DHT sensors
DHT array_DHT[num_DHT];

// File handler to SD
File myFile;
int fileCount = 0;
String fileName = "";

// Retorna dia i hora via RTC
String getDateTime()
{
  String hora = "";
  // En cas que no hi haigui RTC retorna cadena buida
  if(option_clock) {
    DateTime now = rtc.now();
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
  }
  return hora;
}

// Captura les temperatures via array de sensors
void capture_temps(int *array_temperatures){
   // Requests culture temperatures from oneWire Bus
   sensorDS18B20.requestTemperatures();
  // Lectura temperatures array
  for(int i = 0; i < num_temp; i++)
  {
    array_temperatures[i] = sensorDS18B20.getTempCByIndex(i);
    delay(10);
  }
}

// Deteccio si hi ha moviment via PIR
boolean detecta_PIR() {

  return true;
}

void mostra_LCD() {


}

void loop() {

  // Definim un array de temperatures amb tamany num_temp sensors assignats, aquesta variable pot ser global
  int array_temps[num_temp];
  // Cridem la funció per a la recepció de temperatures enviant la direcció de memòria de la variable del array
  capture_temps(array_temps);

  // Exemple sortida dades
  for(int i = 0; i < num_temp; i++)
  {
    Serial.print("Sensor: ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(array_temps[i]);
  }
  delay(2000);
}

void setup() {
  // start serial port:
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Declaring array of DHT22
  if(num_DHT > 0) {
    for(int i=0; i < num_DHT; i++) {
      array_DHT[i](DHT_Pins[i], DHTTYPE);
    }
  }

  // Inicialitza LCD en cas que n'hi haigui
  if(option_LCD) {
    if (debug)
      Serial.println(F("Initialization LCD"));
    lcd.begin (20,4);
    lcd.setBacklightPin(3,POSITIVE);
    lcd.setBacklight(HIGH);

    lcd.home ();                   // go home
    lcd.print("sensors.openspirulina.com");
    lcd.setCursor ( 0, 1 );        // go to the 2nd line
    lcd.print("Sensors OpenSpirulina");
  }

  // Inicialitza SD en cas que n'hi haigui
  if(option_SD) {
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
  }

  // Inicialitza RTC en cas de disposar
  if(option_clock) {
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
  }

}
