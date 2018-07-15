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
 *  phMeterSample.ino from: YouYou
 *
 * BH1750: lux sensor to mesure spirulina's biomass concentration.
 *  library:   https://github.com/claws/BH1750
 * 
 * LCD LiquidCrystal_I2C LiquidCrystal Arduino library for the DFRobot I2C LCD displays:
 * https://github.com/marcoschwartz/LiquidCrystal_I2C
 * 
 * GSM/GPRS A6 modem library:
 * https://github.com/vshymanskyy/TinyGSM
 *
 */

// Select your GPRS modem:
#define TINY_GSM_MODEM_A6
// Increase the buffer
#define TINY_GSM_RX_BUFFER 512
// Set serial for debug console (to the Serial Monitor, speed 115200)
#define SerialMon Serial
// Uncomment this if you want to see all AT commands
// #define DUMP_AT_COMMANDS

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
#include <TinyGsmClient.h>


// Debug mode for verbose info on serial monitor
const boolean debug = true;

/*****
AUTHENTICATION ARDUINO
*****/
//Define de identity of Arduino
const int id_arduino = 1;
const int pin_arduino = 1234;

/****
NETWORK SETTINGS 
****/

// Server connect for sending data
const char server[] = "sensors.openspirulina.com";
const int  port = 80;
// assign a MAC address for the ethernet controller:
const byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "internet";
const char user[] = "";
const char pass[] = "";


/*
  _   _ _    _ __  __ ____  ______ _____     _____ ______ _   _  _____  ____  _____   _____
 | \ | | |  | |  \/  |  _ \|  ____|  __ \   / ____|  ____| \ | |/ ____|/ __ \|  __ \ / ____|
 |  \| | |  | | \  / | |_) | |__  | |__) | | (___ | |__  |  \| | (___ | |  | | |__) | (___
 | . ` | |  | | |\/| |  _ <|  __| |  _  /   \___ \|  __| | . ` |\___ \| |  | |  _  / \___ \
 | |\  | |__| | |  | | |_) | |____| | \ \   ____) | |____| |\  |____) | |__| | | \ \ ____) |
 |_| \_|\____/|_|  |_|____/|______|_|  \_\ |_____/|______|_| \_|_____/ \____/|_|  \_\_____/
 */

const int num_T = 4;    // Temperature of the culture. Sensor DS18B20.MAX 6
                        // T1_s T1_b -- T2_s T2_b = 4
const int num_DHT = 1;  // Humidity and temperature ambient sensor. MAX 3
#define DHTTYPE DHT22   // Type of DHT sensor DHT11 - DHT22
const int num_PIR = 1;  // PIR movement sensor. MAX 3
const int num_DO = 1;   // Optical Density Sensor Module made by OpenSpirulina includes a RGB led + BH1750 lux sensor
const int num_pH = 1;   // pH sensor. MAX 3
enum option_lux_type {  // Valid option_lux_type
  lux_none,
  lux_ldr,
  lux_BH1750
};
const option_lux_type option_lux = lux_BH1750; // No sensor | ldr sensor | lux BH1750
enum option_internet_type { // Valid internet types
  internet_none,
  internet_ethernet,
  internet_gprs,
  internet_wifi
};
const option_internet_type option_internet = internet_gprs; // None | Ethernet | GPRS Modem | Wifi <-- Why not ? Dream on it
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

      /*   ANALOG PINS  */
const int pins_ph[num_pH] = {1};      // pH Pins (Analog)
const int pins_do[num_DO] = {2};      // DO Ps (Analog)
#if option_lux == lux_ldr             // Lux sensor with LDR
const int ldr_pin = 3;                // LDR pin (Analog)
#endif
      /*   DIGITAL PINS  */
const int pin_switch_calibracio = 30; // Pin for pH calibration switch
#define pin_onewire 3                 // where 1-wire is connected
#define pin_sd_card 4                 // Pin lector SD
const int pins_rgb[3] = {24,25,26};      // DO RGB Laser Pins (Digital)
const int pins_dht[num_DHT] = {8};    // DHT Pins
const int pins_pir[num_PIR] = {9};    // PIR Pins
#define SerialAT Serial2              // Serial port for GPRS Modem


/*
   _____ _      ____  ____          _       __      __     _____   _____
  / ____| |    / __ \|  _ \   /\   | |      \ \    / /\   |  __ \ / ____|
 | |  __| |   | |  | | |_) | /  \  | |       \ \  / /  \  | |__) | (___
 | | |_ | |   | |  | |  _ < / /\ \ | |        \ \/ / /\ \ |  _  / \___ \
 | |__| | |___| |__| | |_) / ____ \| |____     \  / ____ \| | \ \ ____) |
  \_____|______\____/|____/_/    \_\______|     \/_/    \_\_|  \_\_____/
*/

const unsigned long wait_opening_led = 1000; // Waiting ms for opening led
const int samples_number = 10;        // Number of samples of DO

OneWire oneWireObjeto(pin_onewire);
DallasTemperature sensorDS18B20(&oneWireObjeto);
// Array de temperatures amb tamany num_temp sensors assignats
float array_temps[num_T];
// LCD I2C
#define I2C_ADDR    0x3F                    // LCD I2C address 0x27 - Alter address
LiquidCrystal_I2C lcd(I2C_ADDR, 20, 4);     // LCD Type Columns * Lines
// RTC DS3231 (clock sensor)
RTC_DS3231 rtc;
// Array of DHT sensors
DHT* array_DHT[num_DHT];
// Array temperatures of DHT
float array_DHT_T[num_DHT];
// Array Humiditys of DHT
float array_DHT_H[num_DHT];
// Array of pH sensors
float array_ph[num_pH];
// Array of PIR sensors
int array_pir[num_PIR];
// Array of DO sensors [R,G,B,RGB]
float array_do1[4];
BH1750 ir_led1(0x23);    //Si el ADDR està inactiu
// Lux sensor with BH1750
BH1750 lux_sensor(0x5C);      //Si el ADDR està amb més de 0.7V
const int pin_lux_addr = 36;  // Pin ADDR

DeviceAddress sensor_t1_b = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
DeviceAddress sensor_t1_s = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

DeviceAddress sensor_t2_b = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
DeviceAddress sensor_t2_s = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

DeviceAddress* array_tSensor_addrs[num_T];

// Lux ambient value
float lux;
// Last time sended data
String last_send;
// LCD view counter
int counts_lcd = 0;

// Var for check time to next loop
uint32_t time_next_loop;
// 3 minute delay
const uint32_t delay_next_loop = 5L * 60L;
// 20 seconds step delay
const unsigned long step_delay_time = 20L * 1000L;

// GPRS Modem
#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);

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
    if(now.hour() < 10)     // For hours less than 2 digits
      hora += "0";
    hora += now.hour();
    hora += ':';
    if(now.minute() < 10)   // For minute less than 2 digits
      hora += "0";    
    hora += now.minute();
    hora += ':';
    if(now.second() < 10)   // For second less than 2 digits
      hora += "0";    
    hora += now.second();
  }
  return hora;
}

// Retorna hora via RTC
String getTime()
{
  String hora = "";
  // En cas que no hi haigui RTC retorna cadena buida
  if(option_clock) {
    DateTime now = rtc.now();
    if(now.hour() < 10)     // For hours less than 2 digits
      hora += "0";
    hora += now.hour();
    hora += ':';
    if(now.minute() < 10)   // For minute less than 2 digits
      hora += "0";    
    hora += now.minute();
  }
  return hora;
}

// Retorna dia i hora via RTC
String getDate()
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
  }
  return hora;
}

// Setup DS18B20 array address
void setup_DS18B20_addr() {
  array_tSensor_addrs[0] = &sensor_t1_b;
  array_tSensor_addrs[1] = &sensor_t1_s;
  array_tSensor_addrs[2] = &sensor_t2_b;
  array_tSensor_addrs[3] = &sensor_t1_s;
}

// Captura les temperatures via array de sensors
void capture_temps() {
   // Requests culture temperatures from oneWire Bus
   sensorDS18B20.requestTemperatures();
  // Read temperatures array
  for(int i = 0; i < num_T; i++)
  {
    array_temps[i] = sensorDS18B20.getTempC(*array_tSensor_addrs[i]);
    delay(10);
  }
}

// Captura les dades temperatures/humitat dels DHT
void capture_dht() {
  for(int i=0; i<num_DHT; i++) {
    // Read Temperature
    array_DHT_T[i] = array_DHT[i]->readTemperature();
    // Read Humidity
    array_DHT_H[i] = array_DHT[i]->readHumidity();
  }
}

// Return ph value from SensorPin
float capture_ph(int SensorPin) {
  unsigned long int avgValue;  //Store the average value of the sensor feedback
  int buf[10],temp;
  for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(SensorPin);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=3.5*phValue;                      //convert the millivolt into pH value
  
  // Return phValue for that SensorPin
  return phValue;
}


// Deteccio si hi ha moviment via PIR
boolean detecta_PIR(int pin) {
  //for(int i=0; i<num_PIR; i++){
  // Si hi ha moviment al pin PIR retorna true
  if(digitalRead(pin) == HIGH)
    return true;
  else
    return false;
}

// Functions for Optical Density (DO)

// Red light values for DO.
float R1_led()
{
  digitalWrite(pins_rgb[0], HIGH);
  // Llegim valors amb el led obert
  delay(wait_opening_led);
  float iir1[samples_number];
  float iir=0;
  for (int i=0; i<samples_number; i++) 
  {
    iir1[i] = ir_led1.readLightLevel();
    iir = iir1[i] + iir;
    delay(500);
  }
  digitalWrite(pins_rgb[0], LOW);
  return (float)iir / samples_number;
}

// Green light values for DO.
float G1_led()
{
  digitalWrite(pins_rgb[1], HIGH);
  float iir1=0;
  float iir=0;
  delay(wait_opening_led);
  for (int i=0; i<samples_number; i++) 
  {
    iir1 = ir_led1.readLightLevel();
    iir = iir1 + iir;
    delay(500);
  }
  digitalWrite(pins_rgb[1], LOW);
  return (float)iir / samples_number;
}


// Blue light values for DO.
float B1_led()
{
  digitalWrite(pins_rgb[2], HIGH);
  // Llegim valors amb el led obert
  delay(wait_opening_led);
  float iir1=0;
  float iir=0;
  for (int i=0; i<samples_number; i++) 
  {
    iir1 = ir_led1.readLightLevel();
    iir = iir1 + iir;
    delay(500);
  }
  digitalWrite(pins_rgb[2], LOW);
  return (float)iir / samples_number;
}

// White light values for DO.
float RGB1_led()
{
  digitalWrite(pins_rgb[0], HIGH);
  digitalWrite(pins_rgb[1], HIGH);
  digitalWrite(pins_rgb[2], HIGH);
  // Llegim valors amb el led obert
  delay(wait_opening_led);
  float iir1=0;
  float iir=0;
  for (int i=0; i<samples_number; i++) 
  {
    iir1 = ir_led1.readLightLevel();
    iir = iir1 + iir;
    delay(500);
  }
  digitalWrite(pins_rgb[0], LOW);
  digitalWrite(pins_rgb[1], LOW);
  digitalWrite(pins_rgb[2], LOW);
  return (float)iir / samples_number;
}

// Capture DO values
void capture_DO() {
  array_do1[0] = R1_led();    // R value
  array_do1[1] = G1_led();    // G value
  array_do1[2] = B1_led();    // B value
  array_do1[3] = RGB1_led();  // All value
}

// Capture Lux ambient
float capture_lux() {
  #if option_lux == lux_BH1750          // Return Lux value with BH1750
    return lux_sensor.readLightLevel();
  #elif option_lux == lux_ldr           // Return Lux value with LDR
    return analogRead(ldr_pin);
  #endif
}

void lcd_init_msg() {
    lcd.clear();
    lcd.home ();                   // go home
    lcd.setCursor ( 3, 0 );        
    lcd.print("OpenSpirulina");
    if(option_clock) {
      lcd.setCursor ( 0, 2 );        // go to the 2nd line
      lcd.print(getDate());
    }
    lcd.setCursor ( 0, 3 );        // go to the 3nd line
    lcd.print("Getting data...");
}

// Capture data in calibration mode
void do_a_calibration_ph() {
  if(debug)
    Serial.println(F("Starting calibration mode..."));
  char buffer_L[6];              // String buffer
  
  lcd.clear();                  // Clear screen
  lcd.home ();                  // Linea 1
  lcd.print("pH Calibration");             

  if(num_pH > 0) {
    lcd.setCursor ( 0, 1 );       // go to the 2nd line
    lcd.print("pH1:");
    dtostrf(capture_ph(pins_ph[0]),4,2,buffer_L);
    lcd.print(buffer_L);
  }
  if(num_pH > 1) {
    lcd.setCursor ( 0, 2 );       // go to the 3rd line
    lcd.print("pH2:");
    dtostrf(capture_ph(pins_ph[1]),4,2,buffer_L);
    lcd.print(buffer_L);
  }
  lcd.setCursor ( 3, 3 );       // go to the 4th line
  lcd.print("OpenSpirulina");
}

// Mostra per LCD les dades
void mostra_LCD() {
  counts_lcd += 1;          // Every 10 show_screen => Reset LCD
  if(counts_lcd > 10) {
    lcd.init();
    lcd.begin (20,4);
    lcd.backlight();
    lcd.setBacklight(HIGH);
    counts_lcd = 0;
  }
  char buffer_L[6];              // String buffer

  lcd.clear();                  // Clear screen
  lcd.home ();                  // Linea 1
  if(num_T > 0) {
    lcd.print("T1:");             // (T1_s + T1_b) / 2
    float t1_mitja = ( array_temps[0] + array_temps[1] ) / 2;
    dtostrf(t1_mitja,4,2,buffer_L);
    lcd.print(buffer_L);
  }
  if(num_T > 2) {
    lcd.print(" T2:");             // (T2_s + T2_b) / 2
    float t2_mitja = ( array_temps[2] + array_temps[3] ) / 2;
    dtostrf(t2_mitja,4,2,buffer_L);
    lcd.print(buffer_L);  
  }

  lcd.setCursor ( 0, 1 );       // go to the 2nd line
  if(num_pH > 0) {
    lcd.print("pH1:");
    dtostrf(array_ph[0],4,2,buffer_L);
    lcd.print(buffer_L);
  }
  if(num_pH > 1) {
    lcd.print("pH2:");
    dtostrf(array_ph[1],4,2,buffer_L);
    lcd.print(buffer_L);
  }

  lcd.setCursor ( 0, 2 );       // go to the 3rd line
  if(last_send != "") {
    lcd.print("LAST: ");
    lcd.print(last_send);
  }

  lcd.setCursor ( 3, 3 );       // go to the 4th line
  lcd.print("OpenSpirulina");
}

// Inicialitza el nom del fitxer a escriure
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

// Writing title headers to file
void write_SD_Headers() {
  myFile = SD.open(fileName, FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile)
  {
    // If Have RTC
    if(option_clock)
      myFile.print(F("DateTime#"));
    // T1_s#T1_b#......#Tn_s#Tn_b#
    for(int i=0, j=0; i<num_T; i++) {
      if(i%2==0) {
        myFile.print(F("T"));
        myFile.print(j);
        myFile.print(F("_s#"));
      }
      else {
        myFile.print(F("T"));
        myFile.print(j);
        myFile.print(F("_b#"));
        j++;
      }
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
    // Lux sensor
    if(option_lux != lux_none) {
      myFile.print(F("lux#"));
    }
    // pH Sensor
    for(int i=0; i<num_pH; i++) {
      myFile.print(F("pH_"));
      myFile.print(i);
      myFile.print(F("#"));
    }
    // DO Sensor
    for(int i=0; i<num_DO; i++) {
      // R
      myFile.print(F("DO_"));
      myFile.print(i);
      myFile.print(F("_R#"));
      // G
      myFile.print(F("DO_"));
      myFile.print(i);
      myFile.print(F("_G#"));
      // B
      myFile.print(F("DO_"));
      myFile.print(i);
      myFile.print(F("_B#"));
      // RGB
      myFile.print(F("DO_"));
      myFile.print(i);
      myFile.print(F("_RGB#"));                  
    }
    // End of line
    myFile.println(F(""));  
    // close the file:
    myFile.close();
  }
}

void save_to_SD() {
  // Writing results to file
  myFile = SD.open(fileName, FILE_WRITE);

  // if the file opened okay, write on it:
  if (myFile)
  {
    if (debug)
      Serial.println("Writing to: " + fileName);

    //DateTime if have RTC
    if(option_clock) {
      myFile.print(getDateTime());
      myFile.print(F("#"));
    }
    // Temperatures del cultiu Tn_s, Tn_b, ...
    for(int i=0; i<num_T; i++) {
      myFile.print(array_temps[i]);
      myFile.print(F("#"));
    }
    // Sensors DHT
    for(int i=0; i<num_DHT; i++) {
      myFile.print(F("#"));
      myFile.print(array_DHT_T[i]);
      myFile.print(F("#"));   
      myFile.print(array_DHT_H[i]);
      myFile.print(F("#"));
    }
    // Lux sensor
    if(option_lux != lux_none) {
      myFile.print(lux);
      myFile.print("#");
    }
    // pH Sensor
    for(int i=0; i<num_pH; i++) {
      myFile.print(array_ph[i]);
      myFile.print(F("#"));
    }
    // DO Sensor
    if(num_DO > 0) {                    // If have DO sensor
      for(int i=0; i<4; i++) {          // R-G-B-RGB 
        myFile.print(array_do1[i]);     // 0-1-2-3
        myFile.print(F("#"));          
      }
    }

    // End of line
    myFile.println("");
    // close the file:
    myFile.close();

    if (debug)
      Serial.println(F("Writting SD done."));
  }
  else
  {
    // if the file didn't open, print an error:
    if (debug)
      Serial.println("Error opening SD file: " + fileName);
  }
}

boolean send_data_server() {

  String cadena = "GET /afegir.php?";

  // Append our ID Arduino
  cadena += "idarduino=";
  cadena += id_arduino;

  // Append PIN for that ID
  cadena += "&pin=";
  cadena += pin_arduino;

  // Append temperatures
  for( int i=0; i<num_T; i++){
    cadena += "&temp";
    cadena += i+1;
    cadena += "=";
    cadena += array_temps[i];
  }
  
  // Append Ambient temperatures and Humidity
  for( int i=0; i<num_DHT; i++) {
    cadena += "&ta";
    cadena += i+1;
    cadena += "=";
    cadena += array_DHT_T[i];
    cadena += "&ha";
    cadena += i+1;
    cadena += "=";
    cadena += array_DHT_H[i];    
  }
  
  // Append Lux sensors
  if(option_lux != lux_none) {
    if(option_lux == lux_ldr)
      cadena += "&ldr1=";
    else if(option_lux == lux_BH1750)
      cadena += "&lux1=";
    cadena += lux;
  }  

  // Append pH Sensor
  for(int i=0; i<num_pH; i++) {
    cadena += "&ph";
    cadena += i+1;
    cadena += "=";
    cadena += array_ph[i];
  }  

  // Append DO Sensor
  if(num_DO > 0) {                    // If have DO sensor
    // R
    cadena += "&do1_R=";
    cadena += array_do1[0];
    // G
    cadena += "&do1_G=";
    cadena += array_do1[1];
    // B
    cadena += "&do1_B=";
    cadena += array_do1[2];
    // RGB
    cadena += "&do1_RGB=";
    cadena += array_do1[3];     
  }

  // Append PIR Sensor
  for( int i=0; i<num_PIR; i++) {
    cadena += "&pir";
    cadena += i+1;
    cadena += "=";
    if(array_pir[i] == 0)
      cadena += "0";
    else if(array_pir[i] == 1)
      cadena += "1";
  }  
  
  if(debug) {
    Serial.print(F("Server petition: "));
    Serial.println(cadena);
  }

  // Send data to specific hardware
  if(option_internet == internet_ethernet) {
    // Add end of GET petition for Ethernet and send
    cadena += " HTTP/1.1";
    return send_data_ethernet(cadena);
  }
  else if(option_internet == internet_gprs) {
    // Send petition to GPRS Modem
    return send_data_modem(cadena, false);
  }
}

boolean send_data_ethernet(String cadena) {
  return true;
}

boolean connect_network() {
  if(debug)
    SerialMon.println("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(F("Network [fail]"));
    delay(1000);
    return false;
  }
  else {
    if(debug)
      SerialMon.println(F("Network [OK]"));  
    return true;
  }
  
}

boolean send_data_modem(String cadena, boolean step_retry) {

  // Set GSM module baud rate
  SerialAT.begin(38400);
  delay(3000);
  if (debug)
    SerialMon.println("Initializing modem...");
    
  if (!modem.restart()) {
    SerialMon.println(F("Modem init [fail]"));
    delay(1000);
    return false;
  }

  if (debug) {
    SerialMon.println(F("Modem init [OK]"));    
    String modemInfo = modem.getModemInfo();
    SerialMon.print(F("Modem: "));
    SerialMon.println(modemInfo);
  }
  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");    
  if (connect_network()) {
    // Network OK
    SerialMon.print("Connecting to ");
    SerialMon.println(apn);
    if (!modem.gprsConnect(apn, user, pass)) {
      SerialMon.println(F("GRPS [fail]"));
      delay(1000);
      if(step_retry == false) {
        Serial.println(F("[Modem] Retrying connection !"));
        send_data_modem(cadena, true);  // Reconnect modem and init again
      }      
      return false;
    }
    SerialMon.println(F("GPRS [OK]"));
  
    IPAddress local = modem.localIP();
    SerialMon.print("Local IP: ");
    SerialMon.println(local);
  
    SerialMon.print(F("Connecting to "));
    SerialMon.println(server);
    if (!client.connect(server, port)) {
      SerialMon.println(F("Server [fail]"));
      delay(1000);
      if(step_retry == false) {
        Serial.println(F("[Modem] Retrying connection !"));
        send_data_modem(cadena, true);  // Reconnect modem and init again
      }      
      return false;
    }
    SerialMon.println(F("Server [OK]"));
  
      // Make a HTTP GET request:
    client.print(cadena + " HTTP/1.0\r\n");
    client.print(String("Host: ") + server + "\r\n");
    client.print("Connection: close\r\n\r\n");
    /*
      // Wait for data to arrive
      while (client.connected() && !client.available()) {
        delay(100);
        SerialMon.print('.');
      };
      SerialMon.println("Received data: ");
      // Skip all headers
      //client.find("\r\n\r\n");
      // Read data
      unsigned long timeout = millis();
      unsigned long bytesReceived = 0;
      while (client.connected() && millis() - timeout < 5000L) {//client.connected() &&
        //while (client.available()) {
        char c = client.read();
        SerialMon.print(c);
        bytesReceived += 1;
        timeout = millis();
        //}
      }
      */
      SerialMon.println();
      client.stop();
      if(debug)
        SerialMon.println(F("Server disconnected"));
    
      modem.gprsDisconnect();
      if(debug)
        SerialMon.println(F("GPRS disconnected"));
      /*if(debug) {
        SerialMon.println(F("************************"));
        SerialMon.print  (F("Received: "));
        SerialMon.print(bytesReceived);
        SerialMon.println(F(" bytes"));
      }*/
      return true;
  }
  else {
    Serial.println(F("[Modem] Fail !"));
    // Try one more time, if continue fails, continue
    if(step_retry == false) {
      Serial.println(F("[Modem] Retrying connection !"));
      send_data_modem(cadena, true);  
    }
    return false; 
  }
  return false;
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
  // Start loop
  if(debug)
    Serial.println(F("Capturing data..."));

  // If pin calibration ph switch is HIGH
  while( digitalRead(pin_switch_calibracio) == HIGH ) {
    do_a_calibration_ph();
    delay(10000);
  }
  // Show init LCD msg
  lcd_init_msg();

  // Set next timer loop for actual time + delay time (3mins)
  if(option_clock) {
    DateTime now = rtc.now();
    time_next_loop = now.unixtime() + delay_next_loop;
    delay(25);
  }

  // Si tenim sondes de temperatura
  if(num_T > 0) {
    capture_temps();
  }
  
  // Capture PH for each pH Sensor
  for(int i=0; i<num_pH; i++) {
    array_ph[i] = capture_ph(pins_ph[i]);
  }

  if(num_DHT > 0) {
    capture_dht();
  }

  if(option_lux != lux_none) {
    lux = capture_lux();
  }

  // Capture status of PIR Sensors
  for(int i=0; i<num_PIR; i++) {
    if ( detecta_PIR(pins_pir[i]) == true )
      array_pir[i] = 1;
    else
      array_pir[i] = 0;
  } 
  
  //Capture DO values (Red, Green, Blue, and White)
  if(num_DO > 0) {
    capture_DO(); 
    delay(1000);
  } 
  
  if(option_LCD) {
    mostra_LCD();
    delay(1000);
  }
  
  if(option_internet != internet_none) {
    if(send_data_server()) {
      // Si s'envia correctament actualitzar last_send
      delay(200);
    last_send = getTime();
      delay(100);
      last_send += " OK";      
      mostra_LCD();
  }
    else {
      delay(200);
      last_send = getTime();
      delay(100);
      last_send += " FAIL";      
    mostra_LCD();
    }
  }

  if(option_SD) {
    save_to_SD();
  }

  // END Loop()
  // If have RTC clock make timer delay if not internal delay
  if(option_clock) 
  {
    DateTime now;
    do {
      now = rtc.now();
      Serial.print(".");
      delay(step_delay_time);                    // Wait (step_delay_time) seconds more
    } while(now.unixtime() < time_next_loop);    // Wait until timer passed
    Serial.println(".");
  }
  else
  {
  delay(60000); // 60s * 1000ms
  }

  //Serial.flush();
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
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // Establint pin entrada per el switch de calibracio
  pinMode(pin_switch_calibracio, INPUT);
  
  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();

  // Comprovació que el numero sensors de temperatura sigui parell
  if( (num_T % 2) != 0 ) {
    // if(debug) // Mostrar sempre el error per Serial
    Serial.println(F("[ERROR] On number of temperature sensors."));
  }

  // If have DS18B20 Sensors, init them
  if(num_T > 0) {
    if (debug)
      Serial.println(F("Initializing DS18B20 BUS..."));
    sensorDS18B20.begin();
    // Verify number of detected devices
    if(sensorDS18B20.getDeviceCount() != num_T) {
      Serial.print(F("[Error] Incorrect number DS18B20 Devices Detected ! ["));
      Serial.print(sensorDS18B20.getDeviceCount());
      Serial.print(F("] of: "));
      Serial.println(num_T);
    }
    setup_DS18B20_addr();
  }

  // Declaring array of DHT22
  if(num_DHT > 0) {
    for(int i=0; i < num_DHT; i++) {
      array_DHT[i] = new DHT(pins_dht[i], DHTTYPE);
      // Init DHT
      if(debug) {
        Serial.print(F("Initializing DHT Sensor "));
        Serial.print(i);
        Serial.println(F(" ..."));
      }
      array_DHT[i]->begin();
    }
  }

  //Establint modo output per al LED si hi ha DO
  if(num_DO > 0)
  {
    //LED 1  
    pinMode(pins_rgb[0], OUTPUT); // R
    pinMode(pins_rgb[1], OUTPUT); // G
    pinMode(pins_rgb[2], OUTPUT); // B

    if (ir_led1.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {
      if(debug)
        Serial.println(F("[DO] Light BH1750 sensor started"));
      }
    else {
      if(debug)
        Serial.println(F("[DO] Error initialising light sensor BH1750"));
    }
  }

  // Initialize BH1750 light sensor
  if(option_lux == lux_BH1750) {
    pinMode(pin_lux_addr, OUTPUT);
    digitalWrite(pin_lux_addr, HIGH);
    delay(200);
    if (lux_sensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {
      if(debug)
        Serial.println(F("Light BH1750 sensor started"));
      }
    else {
      if(debug)
        Serial.println(F("Error initialising light sensor BH1750"));
    }
  }

  // Inicialitza LCD en cas que n'hi haigui
  if(option_LCD) {
    if (debug)
      Serial.println(F("Initialization LCD"));
    lcd.init();
    lcd.begin (20,4);
    lcd.backlight();
    lcd.setBacklight(HIGH);
    lcd_init_msg();
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
    if (debug)
        Serial.println(F("Init RTC Clock"));
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
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }  

  // Initialize Ethernet shield
  if(option_internet == internet_ethernet) {
    // give the ethernet module time to boot up:
    delay(2000);
    // start the Ethernet connection using a fixed IP address and DNS server:
    // Ethernet.begin(mac, ip, myDns, gateway, subnet);
    // DHCP IP ( For automatic IP )
    Ethernet.begin(mac);
    
    // print the Ethernet board/shield's IP address:
    if(debug) {
      Serial.print(F("My IP address: "));
      Serial.println(Ethernet.localIP());
    }
  }
  // Initialize GPRS Modem
  else if(option_internet == internet_gprs) {

  }
}
