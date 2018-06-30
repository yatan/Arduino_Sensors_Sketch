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
 */

// Select your GPRS modem:
#define TINY_GSM_MODEM_A6
// Increase the buffer
#define TINY_GSM_RX_BUFFER 512
// Set serial for debug console (to the Serial Monitor, speed 115200)
#define SerialMon Serial
// Uncomment this if you want to see all AT commands
#define DUMP_AT_COMMANDS

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
						            // T1_s T1_b
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

#define pin_onewire 0                 // where 1-wire is connected
#define pin_sd_card 4                 // Pin lector SD
const int pins_dht[num_DHT] = {7};    // DHT Pins
const int pins_pir[num_PIR] = {20};   // PIR Pins
const int pins_ph[num_pH] = {1};      // pH Pins (Analog)
const int pins_do[num_DO] = {2};      // DO Ps (Analog)
const unsigned long wait_opening_led = 1000; // Waiting ms for opening led
const int samples_number = 10;        // Number of samples of DO
const int pins_rgb[3] = {5,6,7};      // DO RGB Laser Pins (Digital)
#define SerialAT Serial2              // Serial port for GPRS Modem
#if option_lux == lux_ldr             // Lux sensor with LDR
  const int ldr_pin = 3;              // LDR pin (Analog)
#endif

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
#define I2C_ADDR    0x27                    // LCD I2C address
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
int array_ph[num_pH];
// Array of DO sensors [R,G,B,RGB]
int array_do1[4];
BH1750 ir_led1(0x23);    //Si el ADDR està inactiu
// Define lux sensor Type
#if option_lux == lux_BH1750  // Lux sensor with BH1750
  BH1750 lux_sensor(0x5C);    //Si el ADDR està amb més de 0.7V
/*
  #elif option_lux == lux_ldr   // Lux sensor with LDR
*/
#endif
// Lux ambient value
float lux;

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
    array_DHT_T[i] = array_DHT[i]->readTemperature();
    // Read Humidity
    array_DHT_H[i] = array_DHT[i]->readHumidity();
  }
}

// Return ph value from SensorPin
float capture_ph(int SensorPin) {
  unsigned long int avgValue;  //Store the average value of the sensor feedback
  float b;
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
boolean detecta_PIR() {
  for(int i=0; i<num_PIR; i++){
    if(digitalRead(pins_pir[i]) == HIGH)
      return false;
  }
  return true;
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

// Mostra per LCD les dades
void mostra_LCD() {
  lcd.clear();                  // Clear screen
  lcd.home ();                  // go home
  lcd.print("T1: 12,3 T2: 45,6");
  lcd.setCursor ( 0, 1 );       // go to the 2nd line
  lcd.print("pH1: 11,3 pH2: 11,4");
  lcd.setCursor ( 0, 2 );       // go to the 3rd line
  lcd.print("LAST: ");
  lcd.setCursor ( 2, 3 );       // go to the 4th line
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
    // Temperatures del cultiu Sensors_T
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
    for(int i=0; i<4; i++) {          // R-G-B-RGB 
      myFile.print(array_do1[i]);     // 0-1-2-3
      myFile.print(F("#"));          
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

void send_data_server() {

  String cadena = "GET /afegir.php?";

  // Append our ID Arduino
  cadena += "idarduino=";
  cadena += id_arduino;

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
  
  /*
  // Append LDR sensors
  cadena += "&ldr1=";
  cadena += lux_sensor1;
  cadena += "&ldr2=";
  cadena += lux_sensor2;
  cadena += "&ldr3=";
  cadena += lux_sensor3;
  cadena += "&ldr4=";
  cadena += lux_sensor4;
  */
  /*
  // Append LDR Laser Sensors
  cadena += "&irradiancia1=";
  cadena += ir1;    
  cadena += "&irradiancia1_0=";
  cadena += ir10;  
  cadena += "&irradiancia2=";
  cadena += ir2;    
  cadena += "&irradiancia2_0=";
  cadena += ir20;  
  //cadena += "&laser3=";
  //cadena += laser_sensor3;  
*/
  if(debug) {
    Serial.print("Server petition: ");
    Serial.println(cadena);
  }

  // Send data to specific hardware
  if(option_internet == internet_ethernet) {
    // Add end of GET petition for Ethernet and send
    cadena += " HTTP/1.1";
    send_data_ethernet(cadena);
  }
  else if(option_internet == internet_gprs) {
    // Send petition to GPRS Modem
    send_data_modem(cadena);
  }
}

void send_data_ethernet(String cadena) {

}

void send_data_modem(String cadena) {
  
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(F("Network [fail]"));
    delay(10000);
    return;
  }
  SerialMon.println(F("Network [OK]"));

  SerialMon.print("Connecting to ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println(F("GRPS [fail]"));
    delay(10000);
    return;
  }
  SerialMon.println(F("GPRS [OK]"));

  IPAddress local = modem.localIP();
  SerialMon.print("Local IP: ");
  SerialMon.println(local);

  SerialMon.print(F("Connecting to "));
  SerialMon.print(server);
  if (!client.connect(server, port)) {
    SerialMon.println(F("Server [fail]"));
    delay(10000);
    return;
  }
  SerialMon.println(F("Server [OK]"));
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
  
  /* La discrimanació sobre si les dades de DO son vàlides o no ho farem mitjançant el php...
  if(num_PIR > 0) {
    if ( detecta_PIR() == true )
    {
        delay(1000);        // Esperar i tornar a mirar si hi ha moviment
        if ( detecta_PIR() == true )
        {
          capture_DO();     // Continua existint moviment per tant mesurar DO
        }
    }
    else
    {
      // No hi ha moviment
    }
  }	
  */
  
  //Capture DO values (Red, Green, Blue, and White)
	if(num_DO > 0) {
	  capture_DO();	
	}	
	
  if(option_internet != internet_none) {
    send_data_server();
  }

  if(option_SD) {
    save_to_SD();
  }

  if(option_LCD) {
    mostra_LCD();
  }


  // END Loop()
  delay(10000);
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
      Serial.println(F("[Error] Incorrect number DS18B20 Devices Detected !"));
    }
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
      array_DHT[i]->begin();
    }
  }

  //Establint modo output per al LED si hi ha DO
  if(pins_do > 0)
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
  #if option_lux == lux_BH1750
    if (lux_sensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {
      if(debug)
        Serial.println(F("Light BH1750 sensor started"));
      }
    else {
      if(debug)
        Serial.println(F("Error initialising light sensor BH1750"));
    }
  #endif

  // Inicialitza LCD en cas que n'hi haigui
  if(option_LCD) {
    if (debug)
      Serial.println(F("Initialization LCD"));
    lcd.init();
    lcd.begin (20,4);
    lcd.backlight();
    lcd.setBacklight(HIGH);
    lcd.home ();                   // go home
    lcd.print("OpenSpirulina");
    lcd.setCursor ( 0, 2 );        // go to the 2nd line
    lcd.print("LOADING...");
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
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
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
      Serial.print("My IP address: ");
      Serial.println(Ethernet.localIP());
    }
  }
  // Initialize GPRS Modem
  else if(option_internet == internet_gprs) {
    // Set GSM module baud rate
    SerialAT.begin(38400);
    delay(3000);
    if (debug)
      SerialMon.print("Initializing modem...");
    if (!modem.restart()) {
      SerialMon.println(F("Modem init [fail]"));
      delay(5000);
    }
    else {
      if (debug) {
        SerialMon.println(F("Modem init [OK]"));    
        String modemInfo = modem.getModemInfo();
        SerialMon.print(F("Modem: "));
        SerialMon.println(modemInfo);
      }
      // Unlock your SIM card with a PIN
      //modem.simUnlock("1234");    
    }
  }
}
