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
const int num_DHT22 = 1; //Humidity and temperature ambient sensor. MAX 3
const int num_PIR = 1;  //PIR movement sensor. MAX 3
const int num_DO = 1;   // Optical Density Sensor Module made by OpenSpirulina includes a RGB led + BH1705 lux sensor
const int option_lux = 2; // 0: No sensor. 1: ldr sensor. 2: lux BH1750
const int num_pH = 1;   //pH sensor. MAX 3
const boolean option_internet = modem or ethernet; //if ethernet conexion possible(=1) or not (=0)
const boolean option_LCD true; // if LCD 20x04 possible (=1) or not (=0)
const boolean option_SD = true;   //if SD connexion posible (=1) or not (=0)
const boolean option_clock = true; //if clock posible (=1) or not (=0)

// Pins
#define pin_onewire 0    // where 1-wire is connected

// Global sensors variables
OneWire oneWireObjeto(pin_onewire);
DallasTemperature sensorDS18B20(&oneWireObjeto);
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);



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
  // Inicialitza LCD en cas que n'hi haigui
  if(option_LCD) {
    lcd.begin (20,4);
    lcd.setBacklightPin(3,POSITIVE);
    lcd.setBacklight(HIGH);

    lcd.home ();                   // go home
    lcd.print("sensors.openspirulina.com");
    lcd.setCursor ( 0, 1 );        // go to the 2nd line
    lcd.print("Sensors OpenSpirulina");
  }

  if(option_SD) {

  }

}
