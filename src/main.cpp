#include <RadioLib.h>
//Struct del paquete:
//Struct del paquete:
struct TelemetryPacket {
    uint8_t TYPE;
    uint16_t VOLT;   // mV  (V * 1000)
    int16_t  INCX;   // rad * 1000
    int16_t  INCY;   // rad * 1000
    int32_t  LON;    // deg * 1e7
    int32_t  LAT;    // deg * 1e7
    uint32_t TIME;   // s * 10  -> décimas de segundo
    int16_t  VVEL;   // m/s * 10
    uint32_t PRES;   // Pa
    uint16_t TEMP;   // K * 100
    uint16_t ECO2;   // ppm
    uint16_t UV;     // UV * 100
    int16_t  GYRX;   // rad/s * 1000
    int16_t  GYRY;   // rad/s * 1000
    int16_t  GYRZ;   // rad/s * 1000
    uint16_t ALT;    // m * 10
    uint16_t CHK;    // CRC-16
};

//Definir funciones
bool verificarPaqueteDato(String str);


SX1276 radio = new Module(5, 4, 22, 3);
volatile bool operationDone = false;
bool transmitFlag = false; //   Si Verdadero, entonces estaba transmitiendo
int generalState = 0;
const int BTN_PIN = 21; // Conectado a GND
bool lastBtnState = HIGH;

volatile bool botonPresionado = false;

volatile unsigned long lastInterrupt = 0;



void setFlag(void) {
  // we sent or received  packet, set the flag
  operationDone = true;
}


void IRAM_ATTR isrBoton() {
  if(millis() - lastInterrupt > 1000){
      botonPresionado = true;
      lastInterrupt = millis();
  }
}

void setup () {
    Serial.begin(9600);
    Serial.print("Inicio");
    int state = radio.begin(915.0); // Aca seteo la frecuenca a 915 como debe ser p // REVISAR DOCU: dice que en verdad es para SPI.
    radio.setSpreadingFactor(7);   // SF7: más rápido, suficiente para 400m
    radio.setBandwidth(500.0);      // 500 kHz: más rápido, suficiente para 400m
    
    pinMode(BTN_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(21), isrBoton, FALLING); // Interrupcion si es que se presiona el boton bro

    if(state == RADIOLIB_ERR_NONE) {
        Serial.print("Se inicio bien el modulo");
    } else {
        Serial.print(F("Inicialio mal, codigo "));
        Serial.println(state);
        while (true) { delay(10); }         // Si no se inicializa bien te dice y se queda congelado en el loop
    }
    radio.setDio0Action(setFlag, RISING); // Cada vez que termina de TRANSMITIR o RECIBIR se ejecuta la funcion 
    radio.startReceive(); // Empieza escuchando de frente
}


void loop (){
    if(operationDone) {
        operationDone = false;
        if(transmitFlag){           // ***
            transmitFlag = false;
            radio.startReceive();   // **** 
           
            
        } else {                    // Si le llega algo escuchando
            uint8_t buffer[64];    // (YA NO LO RECIBO COMO STRING)
            int state = radio.readData(buffer, sizeof(buffer)); // Como le llego algo lo leo p, y es general para todos, por eso lo saco.
            uint8_t type = buffer[0];
            if(state == RADIOLIB_ERR_NONE){
                Serial.print("Llego algo causa.");
                if (type == 0){
                    Serial.print("Me llego un pulso, mandando su ack");
                    radio.startTransmit("ack"); //(si transmitFlag es true se pone en recivece revisar en ***) Le mando su ack p
                    transmitFlag = true;
                } else if(type == 1){
                    // No haces nada p
                    radio.startReceive(); // Aca vuelvo a escuchar, xq el cambio que hago en **** es solo si es que estaba TX a RX, por lo que le debo decir que siga escuchando, LUEGO DE QUE YA RECIBIO ALGO
                } else if(type == 2){
                    // guardarSD()
                    Serial.print("telemetria recibida");
                    radio.startReceive();   // Aca vuelvo a escuchar, xq el cambio que hago en **** es solo si es que estaba TX a RX, por lo que le debo decir que siga escuchando, LUEGO DE QUE YA RECIBIO ALGO
                }
            }
        }

    }

    if(botonPresionado && !transmitFlag){
        botonPresionado = false;
        radio.startTransmit("TomarDatosTotales");
        transmitFlag = true;
    }
}


bool verificarPaqueteDato(String str) {
    // Aca deberia verificar que es un paquete de datos, tipo por el header, o la longitud, etc.
    if(str != "Pulso" && str != "Estoy vivo en standby XD" && str != "ack"){
        return true;
    }
    return false;
} 