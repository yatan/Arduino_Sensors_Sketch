/*
 * Send Sensors Data To HTTP (PHP) Server
 *
 * Autor: Fran Romero https://github.com/yatan
 *        OpenSpirulina http://www.openspirulina.com
 *
 * Based on:
 * Ethernet web client sketch:
 *  Repeating Web client
 *  http://www.arduino.cc/en/Tutorial/WebClientRepeating
 *
 * DS18B20 sensor de temperatura para líquidos con Arduino:
 *  https://programarfacil.com/blog/arduino-blog/ds18b20-sensor-temperatura-arduino/
 *
 * pH
 *  http://scidle.com/es/como-usar-un-sensor-de-ph-con-arduino/
 *
 * BH1750: lux sensor to mesure spirulina's biomass concentration.
 *  library:   https://github.com/claws/BH1750
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
#include <LiquidCrystal_I2C.h>


// Debug mode for verbose info on serial monitor
const boolean debug = true;


/*
  _   _ _    _ __  __ ____  ______ _____     _____ ______ _   _  _____  ____  _____   _____
 | \ | | |  | |  \/  |  _ \|  ____|  __ \   / ____|  ____| \ | |/ ____|/ __ \|  __ \ / ____|
 |  \| | |  | | \  / | |_) | |__  | |__) | | (___ | |__  |  \| | (___ | |  | | |__) | (___
 | . ` | |  | | |\/| |  _ <|  __| |  _  /   \___ \|  __| | . ` |\___ \| |  | |  _  / \___ \
 | |\  | |__| | |  | | |_) | |____| | \ \   ____) | |____| |\  |____) | |__| | | \ \ ____) |
 |_| \_|\____/|_|  |_|____/|______|_|  \_\ |_____/|______|_| \_|_____/ \____/|_|  \_\_____/
 */

const int num_T = 4;   // Temperature of the culture. Sensor DS18B20.MAX 6
						// T1_s T1_b
const int num_DHT = 1; //Humidity and temperature ambient sensor. MAX 3
#define DHTTYPE DHT22
const int num_PIR = 1;  //PIR movement sensor. MAX 3
const int num_DO = 1;   // Optical Density Sensor Module made by OpenSpirulina includes a RGB led + BH1750 lux sensor
const int num_pH = 1;   //pH sensor. MAX 3
enum option_lux_type {  // Valid option_lux_type
  None,
  LDR,
  BH1750
};
const option_lux_type option_lux = BH1750; // No sensor | ldr sensor | lux BH1750
enum option_internet_type { // Valid internet types
  None,
  Ethernet,
  GPRS,
  Wifi
};
const option_internet_type option_internet = Ethernet; // None | Ethernet | GPRS Modem | Wifi <-- Why not ? Dream on it
const boolean option_LCD = true; // if LCD 20x04 possible (=1) or not (=0)
const boolean option_SD = true;   //if SD connexion posible (=1) or not (=0)
const boolean option_clock = true; //if clock posible (=1) or not (=0)


/*
  _____ _____ _   _  _____
 |  __ \_   _| \ | |/ ____|
 | |__) || | |  \| | (___
 |  ___/ | | | . ` |\___ \
 | |    _| |_| |\  |____) |
 |_|   |_____|_| \_|_____/
*/
#define pin_onewire 0    // where 1-wire is connected
// Pin lector SD
#define pin_sd_card 4
// DHT Pins
const int pins_dht[num_DHT] = {7};
// PIR Pins
const int pins_pir[num_PIR] = {20};
// pH Pins Analog
const int pins_ph[num_pH] = {1};
// DO Pins Analog
const int pins_do[num_DO] = {2};


/*
   _____ _      ____  ____          _       __      __     _____   _____
  / ____| |    / __ \|  _ \   /\   | |      \ \    / /\   |  __ \ / ____|
 | |  __| |   | |  | | |_) | /  \  | |       \ \  / /  \  | |__) | (___
 | | |_ | |   | |  | |  _ < / /\ \ | |        \ \/ / /\ \ |  _  / \___ \
 | |__| | |___| |__| | |_) / ____ \| |____     \  / ____ \| | \ \ ____) |
  \_____|______\____/|____/_/    \_\______|     \/_/    \_\_|  \_\_____/
*/

OneWire oneWireObjeto(pin_onewire);
DallasTemperature sensorDS18B20(&oneWireObjeto);
// Array de temperatures amb tamany num_temp sensors assignats
int array_temps[num_T];
// LCD I2C
LiquidCrystal_I2C lcd(0x27, 20, 4);
// RTC DS3231 (clock sensor)
RTC_DS3231 rtc;
// Array of DHT sensors
DHT* array_DHT[num_DHT];
// Array temperatures of DHT
float array_DHT_T[num_DHT];
// Array Humiditys of DHT
float array_DHT_H[num_DHT];
// Array of pH sensors
int array_ph[num_pH];
// Array of DO sensors
int array_do[num_DO];

// File handler to SD
File myFile;
int fileCount = 0;
String fileName = "";

/*
  ______ _    _ _   _  _____ _______ _____ ____  _   _  _____  
 |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____| 
 | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___   
 |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \  
 | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) | 
 |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/  
 */

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
void capture_temps(){
   // Requests culture temperatures from oneWire Bus
   sensorDS18B20.requestTemperatures();
  // Read temperatures array
  for(int i = 0; i < num_T; i++)
  {
    array_temps[i] = sensorDS18B20.getTempCByIndex(i);
    delay(10);
  }
}

// Captura les dades temperatures/humitat dels DHT
void capture_dht() {
  for(int i=0; i<num_DHT; i++) {
    // Read Temperature
    array_DHT_T[i] = array_DHT[i].readTemperature();
    // Read Humidity
    array_DHT_H[i] = array_DHT[i].readHumidity();
  }
}

// Deteccio si hi ha moviment via PIR
boolean detecta_PIR() {
  for(int i=0; i<num_PIR; i++){
    if(digitalRead(pins_pir[i]) == HIGH)
      return false;
  }
  return true;
}

void mostra_LCD() {
  lcd.home ();                   // go home
  lcd.print("sensors.openspirulina.com");
  lcd.setCursor ( 0, 1 );        // go to the 2nd line
  lcd.print("Sensors OpenSpirulina");
}

void init_SD_FileName() {
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
}

void write_SD_Headers() {
  // Writing title headers to file
  myFile = SD.open(fileName, FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile)
  {
    if(option_clock)
      myFile.print(F("DateTime#"));
    // Sensor_1#......#Sensor_n#
    for(int i=0; i<num_T; i++) {
      myFile.print(F("sensor_"));
      myFile.print(i);
      myFile.print(F("#"));
    }
    // ambient1_temp#ambient1_humetat#
    for(int i=0; i<num_DHT; i++) {
      myFile.print(F("ambient_"));
      myFile.print(i);
      myFile.print(F("_temp#"));
      myFile.print(F("ambient_"));  
      myFile.print(i);
      myFile.print(F("_humetat#"));      
    }
    
    myFile.println(F("lux#"));    
    // close the file:
    myFile.close();
  }
}

void write_SD() {
  // Writing results to file
  myFile = SD.open(fileName, FILE_WRITE);

  // if the file opened okay, write on it:
  if (myFile)
  {
    if (debug)
      Serial.println("Writing to: " + fileName);

    //DateTime if have RTC
    if(option_clock)
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
    myFile.println(lux_sensor1);
    
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
}

/*
  _      ____   ____  _____
 | |    / __ \ / __ \|  __ \
 | |   | |  | | |  | | |__) |
 | |   | |  | | |  | |  ___/
 | |___| |__| | |__| | |
 |______\____/ \____/|_|
 */

void loop() {

  // Si tenim sondes de temperatura
  if(num_T > 0) {
    // Cridem la funció per a la recepció de temperatures enviant la direcció de memòria de la variable del array
    capture_temps(array_temps);
    // Exemple sortida dades
    for(int i = 0; i < num_T; i++)
    {
      Serial.print("Sensor: ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(array_temps[i]);
    }
  }

  if(num_pH > 0) {
    capture_ph();
  }

  if(num_DHT > 0) {
    capture_dht();
  }

  if(option_lux != None) {
    capture_lux();
  }

  if(num_PIR > 0) {
    if ( detecta_PIR() == true )
    {
        delay(1000);
        // Esperar i tornar a mirar si hi ha moviment
    }
    else
    {
      // No hi ha moviment
    }
  }

  if(option_internet == ethernet) {
    send_data();
  }

  if(option_SD) {
    save_to_SD();
  }

  if(option_LCD) {
    mostra_LCD();
  }


  // END Loop()
  delay(2000);
}

/*
   _____ ______ _______ _    _ _____
  / ____|  ____|__   __| |  | |  __ \
 | (___ | |__     | |  | |  | | |__) |
  \___ \|  __|    | |  | |  | |  ___/
  ____) | |____   | |  | |__| | |
 |_____/|______|  |_|   \____/|_|
*/

void setup() {
  // start serial port:
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Comprovació que el numero sensors de temperatura sigui parell
  if( (num_T % 2) != 0 ) {
    // if(debug) // Mostrar sempre el error per Serial
    Serial.println(F("[ERROR] On number of temperature sensors."));
  }

  // Declaring array of DHT22
  if(num_DHT > 0) {
    for(int i=0; i < num_DHT; i++) {
      array_DHT[i] = new DHT(pins_dht[i], DHTTYPE);
      // Init DHT
      if(debug) {
        Serial.print("Initializing DHT Sensor ");
        Serial.print(i);
        Serial.println(" ...");
      }
      array_DHT[i].begin();
    }
  }

  // Inicialitza LCD en cas que n'hi haigui
  if(option_LCD) {
    if (debug)
      Serial.println(F("Initialization LCD"));
    lcd.begin (20,4);
    lcd.backlight();
    lcd.setBacklight(HIGH);
  }

  // Inicialitza SD en cas que n'hi haigui
  if(option_SD) {
    if (!SD.begin(pin_sd_card))
    {
      if (debug)
        Serial.println(F("Initialization SD failed!"));
    }
    else
    {
      if (debug)
        Serial.println(F("Initialization SD done."));
      // Generate fileName to write
      init_SD_FileName();
      // Write File headers
      write_SD_Headers();
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
    else
    {
      // RTC Work - Print current time
      if (debug)
        Serial.println(getDateTime());
    }

    // Setting RTC time for first time programing RTC
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

}
