/*
 * sketch to turn on/off spirulina agitation
 * depending on:
 *    day/night (light sensor)
 *    5 minuts off
 *    1 minut on
 *    
 * Using arduino uno
 */



/****
 * PIN CONNEXION
 */

const int LDR1 = A3;
const int rele = 3;

// Time Interval for agitation on and off
unsigned long agitation_on = 1L*5L*1000L;
unsigned long agitation_off = 4L*5L*1000L;

// maximum light during night
int nightlight = 200;

 
void setup()
{
  pinMode(rele,OUTPUT);
  Serial.println("inicialitzant");
}
 
void loop()
{
  //Nota tenemos el relé conectado como Normalmente Abierto
  //así solo se activará la carga cuando activemos la bobina
  //del relé, para que funcione al revés cambiaremos el cable
  //a la posición Normalmente Cerrado

if (analogRead(LDR1) > nightlight )
{  
  digitalWrite(rele,HIGH);  //Agitation on 
  delay(agitation_on);              
  digitalWrite(rele,LOW);   //Agitation off
  delay(agitation_off);           
  Serial.println( analogRead(LDR1)); 
}
}

