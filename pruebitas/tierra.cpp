#include <RadioLib.h>
//Definir funciones
bool verificarPaqueteDato(String str);


SX1278 radio = new Module(5, 4, 22, 3);
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
    int state = radio.begin(); // Aca seteo la frecuenca a 915 como debe ser p // REVISAR DOCU: dice que en verdad es para SPI.

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
            String str;
            int state = radio.readData(str);    // Como le llego algo lo leo p, y es general para todos, por eso lo saco.
            if(state == RADIOLIB_ERR_NONE){
                Serial.print("Llego algo causa.");
                if (str == "Pulso"){
                    Serial.print("Me llego un pulso, mandando su ack");
                    radio.startTransmit("ack"); //(si transmitFlag es true se pone en recivece revisar en ***) Le mando su ack p
                    transmitFlag = true;
                } else if(str == "Estoy vivo en standby XD"){
                    // No haces nada p
                    radio.startReceive(); // Aca vuelvo a escuchar, xq el cambio que hago en **** es solo si es que estaba TX a RX, por lo que le debo decir que siga escuchando
                } else if(verificarPaqueteDato(str)){
                    // guardarSD()
                    Serial.print(str);
                    radio.startReceive();   // Aca vuelvo a escuchar, xq el cambio que hago en **** es solo si es que estaba TX a RX, por lo que le debo decir que siga escuchando
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