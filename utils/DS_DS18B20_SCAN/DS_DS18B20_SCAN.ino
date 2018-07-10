#include <OneWire.h>
#include <DallasTemperature.h>

// Pin donde se conecta el bus 1-Wire
const int pinDatosDQ = 9;

// Instancia a las clases OneWire y DallasTemperature
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);

void setup() {
    // Iniciamos monitor serie y sensor de temperatura DS18B20
    Serial.begin(9600);
    sensorDS18B20.begin();

    // Buscamos los sensores conectados
    Serial.println("Buscando dispositivos...");
    Serial.println("Encontrados: ");
    int numeroSensoresConectados = sensorDS18B20.getDeviceCount();
    Serial.print(numeroSensoresConectados);
    Serial.println(" sensores");

    // Si hemos encontrado uno mostramos su dirección
    if(numeroSensoresConectados==1){
        
        // Tipo definido como una array de 8 bytes (uint8_t)
        DeviceAddress sensorTemperatura;
        // Obtenemos dirección
        sensorDS18B20.getAddress(sensorTemperatura, 1);

        // Mostamos por el monitor serie
        Serial.print("Sensor encontrado: ");

        // Recorremos los 8 bytes del identificador único
        for (uint8_t i = 0; i < 8; i++)
        {
          // Si solo tiene un dígito rellenamos con un cero a la izquierda
          if (sensorTemperatura[i] < 16) Serial.print("0");

          // Mostramos los datos que van en HEXADECIMAL
          Serial.print(sensorTemperatura[i], HEX);
        }
    }    
}

void loop() {
}
