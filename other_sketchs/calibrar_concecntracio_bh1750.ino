


//Libraries used

#include <SD.h>
#include <SPI.h>
//#include <Ethernet.h>
//#include "RTClib.h"
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
//const int DHT1_Pin = 30;
//const int DHT2_Pin = 40;




// Start lux sensor from BH1750 library

BH1750 ir_led1(0x23); //Si el ADDR està inactiu

BH1750 ir_led2(0x5C); //Si el ADDR està amb més de 0.7V


// Pin lector SD
  #define sd_card_pin 4

// Pins per connectar els emisors de llum laser
    // LED 1
    #define R1_pin 5
    #define G1_pin 6
    #define B1_pin 7
    // LED 2
    #define R2_pin 8
    #define G2_pin 9
    #define B2_pin 10



boolean debug = true;



/****
 * TIME INTERVAL VARIABLES
 ****/
/*
//Time interval to get metheo data
unsigned long last_get_metheo_data = 0;             
const unsigned long interval_get_metheo_data = 30L * 1000L;


//Time interval to get culture data
unsigned long last_get_culture_data = 0;             
const unsigned long interval_get_culture_data = 30L * 1000L;
*/

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
 *  GET OD DATA VARAIBLES
 *
 ****/


int laser_sensor1 = 0; //Valor de la irradiancia, així doncs millor anomenarlo ir, però com que això cambiar-ho s'ha de cambiar a molttsp uestos abans t'ho pregunto. 

//Irradiancia del laser 1
int ir1 = 0 ;
int ir10 = 500;  //Aquest valor s'hauria de contrastar un primer cop

//Irradiancia del laser 2
int ir2 = 0 ;
int ir20 = 500; //Aquest valor s'hauria de contrastar un primer cop


//Irradiancia del laser 3
int ir3 = 0 ;
int ir30 = 500; //Aquest valor s'hauria de contrastar un primer cop

// Waiting for opening led
unsigned long wait_opening_led = 1000;


//Variables internet get_OD_data

float iR1;
float iG1;
float iB1;
float iRGB1;

float iR2;
float iG2;
float iB2;
float iRGB2;


int iir1;
int iir2;


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
int samples_number = 10;







 



/*****
Global variables for internal use
 *****/

//Irradiancia for get_OD_function
 unsigned long iir = 0L; 




// File handler
File myFile;
int fileCount = 0;
String fileName = "";


 


void setup()
{
      // start serial port:
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
    
    
    //Establint modo output per al LED
//LED 1  
 pinMode(R1_pin, OUTPUT);
 pinMode(G1_pin, OUTPUT);
 pinMode(B1_pin, OUTPUT);


//LED 2  
 pinMode(R2_pin, OUTPUT);
 pinMode(G2_pin, OUTPUT);
 pinMode(B2_pin, OUTPUT);

 
 

  Wire.begin();
 
 if (ir_led1.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {
    Serial.println(F("light BH1750 sensor 1 started"));
  }
  else {
    Serial.println(F("Error initialising light sensor 1 BH1750"));
  }
 
 if (ir_led2.begin(BH1750::CONTINUOUS_HIGH_RES_MODE_2)) {
    Serial.println(F("light BH1750 sensor 2 started"));
  }
  else {
    Serial.println(F("Error initialising light sensor 2 BH1750"));
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


 // Obtain a free file name for writting to SD
/*  fileName += fileCount;
  fileName += ".txt";
  while (SD.exists(fileName))
  {
    fileCount += 1;
    fileName = "";
    fileName += fileCount;
    fileName += ".txt";
  }
*/

fileName = "OD.txt";

  if (debug)
    Serial.println("Writing in file: " + fileName);
  
 // Writting title headers to file
  myFile = SD.open(fileName, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile)
  {
    myFile.print(F("iR1#"));
    myFile.print(F("iG1#"));
    myFile.print(F("iB1#"));
    myFile.print(F("iRGB1#"));
      myFile.print(F("iR2#"));
    myFile.print(F("iG2#"));
    myFile.print(F("iB2#"));
    myFile.println(F("iRGB2"));
    
    // close the file:
    myFile.close();
  }

}









float R1_led()
{

digitalWrite(R1_pin, HIGH);
  // Llegim valors amb el led obert
  delay(wait_opening_led);
               // Serial.println("Valors amb R1 obert");
  iir1=0;
  iir=0;
  for (int i=0; i<samples_number; i++) 
  {
    iir1 = ir_led1.readLightLevel();
  
    iir = iir1 + iir;
              //Serial.print(iir1);
              //Serial.print(" ");

    
    delay(500);
  }

  digitalWrite(R1_pin, LOW);
        //  Serial.println();
    iR1=  iir / samples_number;
  return iR1;
  
}

float G1_led()
{

digitalWrite(G1_pin, HIGH);
            //Serial.println("Valors amb G1 obert");
  iir1=0;
  iir=0;
  delay(wait_opening_led);
  for (int i=0; i<samples_number; i++) 
  {
    iir1 = ir_led1.readLightLevel();
    iir = iir1 + iir;
             // Serial.print(iir1);
             // Serial.print(" ");
    delay(500);
  }
  
  digitalWrite(G1_pin, LOW);
          //Serial.println();
  iG1 =  iir / samples_number;
  return iG1;

  
  
}



float B1_led()
{

digitalWrite(B1_pin, HIGH);
  // Llegim valors amb el led obert
  delay(wait_opening_led);
              // Serial.println("Valors amb R1 obert");
  iir1=0;
  iir=0;
  for (int i=0; i<samples_number; i++) 
  {
    iir1 = ir_led1.readLightLevel();
    iir = iir1 + iir;
                //Serial.print(iir1);
               // Serial.print(" ");
    delay(500);
  }

  digitalWrite(B1_pin, LOW);
                //Serial.println();
    iB1=  iir / samples_number;
  return iB1;
  
}


float RGB1_led()
{

    digitalWrite(R1_pin, HIGH);
    digitalWrite(G1_pin, HIGH);
    digitalWrite(B1_pin, HIGH);
  // Llegim valors amb el led obert
  delay(wait_opening_led);
              // Serial.println("Valors amb RGB1 obert");
  iir1=0;
  iir=0;
  for (int i=0; i<samples_number; i++) 
  {
    iir1 = ir_led1.readLightLevel();
    iir = iir1 + iir;
              //Serial.print(iir1);
              //Serial.print(" ");
    delay(500);
  }

    digitalWrite(R1_pin, LOW);
    digitalWrite(G1_pin, LOW);
    digitalWrite(B1_pin, LOW);
                //Serial.println();
    iRGB1=  iir / samples_number;
  return iRGB1;
  
}



///// LED 2  /////

float R2_led()
{

digitalWrite(R2_pin, HIGH);
  // Llegim valors amb el led obert
  delay(wait_opening_led);
            //Serial.println("Valors amb R2 obert");
  iir2=0;
  iir=0;
  for (int i=0; i<samples_number; i++) 
  {
    iir2 = ir_led2.readLightLevel();
  
    iir = iir2 + iir;
                //Serial.print(iir2);
                //Serial.print(" ");

    
    delay(500);
  }

  digitalWrite(R2_pin, LOW);
                //Serial.println();
    iR2=  iir / samples_number;
  return iR2;
  
}

float G2_led()
{

digitalWrite(G2_pin, HIGH);
              //Serial.println("Valors amb G2 obert");
  iir2=0;
  iir=0;
  delay(wait_opening_led);
  for (int i=0; i<samples_number; i++) 
  {
    iir2 = ir_led2.readLightLevel();
    iir = iir2 + iir;
              //Serial.print(iir2);
              //Serial.print(" ");
    delay(500);
  }
  
  digitalWrite(G2_pin, LOW);
              //Serial.println();
  iG2 =  iir / samples_number;
  return iG2;

  
  
}



float B2_led()
{

digitalWrite(B2_pin, HIGH);
  // Llegim valors amb el led obert
  delay(wait_opening_led);
              //Serial.println("Valors amb B2 obert");
  iir2=0;
  iir=0;
  for (int i=0; i<samples_number; i++) 
  {
    iir2 = ir_led2.readLightLevel();
    iir = iir2 + iir;
              //Serial.print(iir2);
              //Serial.print(" ");
    delay(500);
  }

  digitalWrite(B2_pin, LOW);
              //Serial.println();
    iB2=  iir / samples_number;
  return iB2;
  
}


float RGB2_led()
{

digitalWrite(R2_pin, HIGH);
digitalWrite(G2_pin, HIGH);
digitalWrite(B2_pin, HIGH);
  // Llegim valors amb el led obert
  delay(wait_opening_led);
          // Serial.println("Valors amb RGB2 obert");
  iir2=0;
  iir=0;
  for (int i=0; i<samples_number; i++) 
  {
    iir2 = ir_led2.readLightLevel();
    iir = iir2 + iir;
   // Serial.print(iir2);
    //Serial.print(" ");
    delay(500);
  }

  digitalWrite(R2_pin, LOW);
  digitalWrite(G2_pin, LOW);
  digitalWrite(B2_pin, LOW);
        // Serial.println();
    iRGB2=  iir / samples_number;
  return iRGB2;
  
}




void data_to_sd ()
{
   myFile = SD.open(fileName, FILE_WRITE);

   // if the file opened okay, write to it:
   if (myFile)
   {
     if (debug)
       Serial.println("Writing to: " + fileName);

     //DateTime from RTC
     myFile.print(iR1);
       myFile.print('#');
     myFile.print(iG1);
       myFile.print('#');  
     myFile.print(iB1);
       myFile.print('#');
     myFile.print(iRGB1);
       myFile.print('#');      

     myFile.print(iR2);
       myFile.print('#');
     myFile.print(iG2);
       myFile.print('#');  
     myFile.print(iB2);
       myFile.print('#');
     myFile.println(iRGB2);
       

  
     // close the file:
     myFile.close();

     if (debug)
       Serial.println("Writting SD done.");
   }
   else
   {
     // if the file didn't open, print an error:
     if (debug)
       Serial.println("Error opening: " + fileName);
   }

}

void loop()
{
Serial.println();
//laser 1
delay(60000); //Per tal que s'homogenitzi bé abans de començar a mesurar
    R1_led();
    G1_led();
    B1_led();
    RGB1_led();


//laser 2
    R2_led();
    G2_led();
    B2_led();
    RGB2_led();
 

//write to sd
data_to_sd();
  
  
if(debug)
{
  Serial.print("Irradiancia 1 amb Roig: ");
  Serial.println(iR1);
  Serial.print("Irradiancia 1 amb Verd: ");
  Serial.println(iG1);
  Serial.print("Irradiancia 1 amb Blau: ");
  Serial.println(iB1);
  Serial.print("Irradiancia 1 amb Blanc: ");
  Serial.println(iRGB1);
  
  Serial.print("Irradiancia 2 amb Roig: ");
  Serial.println(iR2);
  Serial.print("Irradiancia 2 amb Verd: ");
  Serial.println(iG2);
  Serial.print("Irradiancia 2 amb Blau: ");
  Serial.println(iB2);
  Serial.print("Irradiancia 2 amb Blanc: ");
  Serial.println(iRGB2);
  
} 
    
  delay(500000);

}
