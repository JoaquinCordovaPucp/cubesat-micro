#include <RadioLib.h>

SX1278 radio = new Module(5, 4, 22, 3);
int transmissionState = RADIOLIB_ERR_NONE;  // Aca se guardara el estado del radio, osea los errores (codigo), o no error (codigo 0)

unsigned long previousMillis = 0; // Stores the last time the function was executed
const long interval1Sec = 1000; 
const long interval100ms= 100; 
volatile bool operationDone = false;
int generalState = 0; // This stores the state of the machine
bool transmitFlag = false; //   Si Verdadero, entonces estaba transmitiendo

void setFlag(void) {
  // we sent or received  packet, set the flag
  operationDone = true;
}

// Definicion de funciones
int getStateByCmd(String cmd);
bool verificarPaqueteDato(String str);

void setup () {
    Serial.begin(9600);
    Serial.print("Inicio");
    int state = radio.begin(); // Aca seteo la frecuenca a 915 como debe ser p // REVISAR DOCU: dice que en verdad es para SPI.

    if(state == RADIOLIB_ERR_NONE) {
        Serial.print("Se inicio bien el modulo");
    } else {
        Serial.print(F("Inicialio mal, codigo "));
        Serial.println(state);
        while (true) { delay(10); }         // Si no se inicializa bien te dice y se queda congelado en el loop
    }

      radio.setDio0Action(setFlag, RISING); // Cada vez que termina de TRANSMITIR o RECIBIR se ejecuta la funcion 
}

void loop() {
    if(generalState == 0){
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis >= interval1Sec) {
            previousMillis = currentMillis;
            // Functions:
            Serial.print("Mandando pulso(Heart Beat)");
            // transmissionState = radio.startTransmit("Pulso");   
            // Nota: No uso starTransmit(), xq esta es no bloqueante y requeriria usar la iterrupcion cuando termina,
            // pero no es necesario realizar otras cosas en paralelo. Por lo que uso el transmit, que bloquea el codigo
            // hasta que se envia
            transmissionState = radio.startTransmit("Pulso");    // Transmito pulso desde el cubesat, y espero que se mande completamente
            transmitFlag = true;
            // ESTO HUBISESE HECHO SI ES QUE ESPERO QUE SE MANDE, PERO MEJOR "LANZO LA ORDEN DE MANDAR", Y CUANDO OPERATIONDONE Y 
            // EL FLAG DE TRASMIT, EN EL LOOP LO PONE POR DEFECTO A ESCUCHAR NUEVAMENTE:
            // if(transmissionState == RADIOLIB_ERR_NONE) {    
            //     Serial.print("Escuchando a Tierra");        // Empiezo a escuchar a tierra
            //     int stateRecieve = radio.startReceive();    // Esto hace que el modulo se quede escuchando continuamente, 
            //     if (stateRecieve == RADIOLIB_ERR_NONE) {    // y levanta el DIO0 cuando llega algo. 
            //         Serial.println(F("success!"));
            //         } else {
            //         Serial.print(F("failed, code "));
            //         Serial.println(stateRecieve);
            //         while (true) { delay(10); }
            //     }
            // }



        }
    }

    if(generalState == 1){
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis >= interval1Sec){
            previousMillis = currentMillis;
            Serial.print("StandBy");
            radio.startTransmit("Estoy vivo en standby XD");  // Solo manda pa saber que esta vivo, cada 1 segundo
            transmitFlag = true;
        }
    }


    if(generalState == 2){
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis >= interval100ms){
            previousMillis = currentMillis;
            Serial.print("Mando Dato Basico");
            // Aca se tomarian datos ps y calculos 
            delay(12); // Delay imaginario
            radio.startTransmit("Datos Basico XD"); 
            transmitFlag = true;
        }
    } 

    if(generalState == 3){
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis >= interval100ms){
            previousMillis = currentMillis;
            Serial.print("Mando Datos Completos");
            // Aca se tomarian datos ps y calculos 
            delay(12); // Delay imaginario
            radio.startTransmit("Datos Completos XD"); 
            transmitFlag = true;
        }
    }

    if(generalState == -1){
        Serial.print("Morimos");
        while(1){
            delay(100);
        }
    }




    if(operationDone) { // Si acaba de acabar algo
        operationDone = false;

        if(transmitFlag){   
            // Siempre que se mande algo, y llege la confirmacion de que termino (operationDone), debo pasar a que escuche,
            // por defecto debe estar siempre escuchando. Solo TX cuando sea necesario.
            radio.startReceive();
            transmitFlag = false;
        } else {        // Si acaba de recibir algo
            String str;
            int stateReceive = radio.readData(str);

            if( generalState == 0) { // Recibio algo y estamos en la fase 0 (Pulsos del CB y espera del ACK)
                if (stateReceive == RADIOLIB_ERR_NONE){
                    Serial.println(F("Se recibio un paqueton"));
                    // Imprimir el content
                    Serial.print(F("Data:\t\t"));
                    Serial.println(str);
                    int pos = str.indexOf('&');
                    String parte1;
                    String parte2;
                    if(pos != -1) {
                        parte1 = str.substring(0, pos);
                        parte2 = str.substring(pos + 1);
                    } else {
                        parte1 = str;
                        parte2 = "";
                    }
        
                    if(parte1 == "ack"){
                        generalState = 1;
                        Serial.print("Se recibio ack de la estacion a Tierra");
                        if(parte2 != ""){
                            generalState = getStateByCmd(parte2);
                            Serial.print("Se recibio comando a cambiar a estado de: ");
                            Serial.print(generalState);
                        }
                    }
        
                }
            } else { // Si ya paso el inicio generalState = 0, solo me fijo en los cmd
                if(stateReceive == RADIOLIB_ERR_NONE) {
                    Serial.print("Se recibio un comando");
                    Serial.print("Data:\t\t");
                    Serial.println(str);
                    generalState = getStateByCmd(str);
                }
            }
        }
    }
}

int getStateByCmd(String cmd) {

    if (cmd == "Stand By") {
        return 1;
    }

    if (cmd == "TomarDatosBasicos") {
        return 2;
    }

    else if (cmd == "TomarDatosTotales") {
        return 3;
    }

    return generalState; // Si no es ninguno de los comandos, se queda en el mismo estado. Nota, error cuando 
}