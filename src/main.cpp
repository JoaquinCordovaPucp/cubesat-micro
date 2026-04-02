#define interval1Sec 1000
#define interval100ms 100
#define interval10ms 10
#define voltagePin 34 //TODO: Elegir el pin

#define ROLL_PIN 25
#define PITCH_PIN 34

const float desiredRoll = 0.0; // Ángulo deseado de roll (en grados)
const float desiredPitch = 0.0; // Ángulo deseado de pitch (en grados)
float Kp = 1.0; // Ganancia proporcional
float Ki = 0.0; // Ganancia integral
float Kd = 0.0; // Ganancia derivativa

#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h> //GPS
#include "sensors.hpp" //Libreria creada para sensores, para facilitar la lectura de datos y el guardado en el struct TelemetryPacket
#include "acs.hpp"     //Libreria creada para el control del ACS, para facilitar el control de los reaction wheels, y el calculo de las incx e incy a partir de los datos del acelerometro.
#include "complementary.hpp" // Filtro complementario para estimar la altitud y velocidad

Sensors dataSensors;        //Objeto de la clase Sensors(viene de la libreria sensors.hpp) Cambio RICK
ACSController acs;          //Objeto de la clase ACSController(viene de la libreria acs.hpp) Cambio RICK
AltitudeFilter altFilter; // Tu nuevo filtro
unsigned long filterLastime = 0;
float baroAltitudeRef = 0.0f;

struct TelemetryPacket pckt;    //Struct que se va a mandar, con todos los datos de los sensores, y el CRC-16. (viene de la libreria sensors.hpp) Cambio RICK
SX1276 radio = new Module(5, 4, 22, 3);
TinyGPSPlus gps;
HardwareSerial GPSserial(2);

int transmissionState = RADIOLIB_ERR_NONE;  // Aca se guardara el estado del radio, osea los errores (codigo), o no error (codigo 0)

unsigned long previousMillis = 0; // Stores the last time the function was executed
volatile bool operationDone = false;
int generalState = 0; // This stores the state of the machine (0: recien prendido y emitiendo un pulso, 1: standby, 2: trasmitiendo datos basico, 3: transmitiendo datos completos)
bool transmitFlag = false; //   Si Verdadero, entonces estaba transmitiendo

void setFlag(void) {        // Callback de la interrupcion de finalizacion de transmision o recepecion
  operationDone = true;     // Cuando termina una operacion se coloca en true, ya sea TX o RX. Util cuando se usa metodos no bloqueantes como startTransmit() o startReceive()
}

// Definicion de funciones axuliares
int getStateByCmd(String cmd);
 //PID
float previousErrorRoll = 0.0;
float previousErrorPitch = 0.0;
float integralRoll = 0.0;
float integralPitch = 0.0;
unsigned long lastTime = 0;

void setup () {
    Wire.begin(21, 26); //Iniciar I2C (Lo uso para todos los sensores con I2C)
    //Esta funcion inicia Serial y los sensores(incluido pin voltaje)
    dataSensors.init(&Serial); // Le paso el objeto Serial como puntero(Este objeto esta definido por Arduino.h)  
    acs.begin(ROLL_PIN, PITCH_PIN); // Inicializar el controlador ACS en los pines de roll/pitch usando rangos por defecto (1000-2000 us)
    GPSserial.begin(9600, SERIAL_8N1, 16, 17); // Inciar GPS, con los pines en rx y tx seleccionados(rx del gps va al tx, y sucesivamente)

    //ACCESO A TRAVES DEL OBJETO DE LA CLASE SENSORS, PARA CONFIGURAR LOS SENSORES.
    dataSensors.mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    dataSensors.mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    dataSensors.mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);         //Configuracion del Acel y Giro MPU6050. TODO: Ver la configuracion idonea para el caso de uso
    dataSensors.ltr390.setMode(LTR390_MODE_UVS);
    dataSensors.ltr390.setGain(LTR390_GAIN_3);
    dataSensors.ltr390.setResolution(LTR390_RESOLUTION_16BIT);
    dataSensors.startENS160StandardMeasure();
    //INICIAR LORA
    int state = radio.begin(915.0); // Aca seteo la frecuenca a 915 como debe ser p // REVISAR DOCU: dice que en verdad es para SPI, pero esta funcionando correctamente.
    radio.setSpreadingFactor(7);   // SF7: más rápido, suficiente para 400m
    radio.setBandwidth(500.0);      // 500 kHz: más rápido, suficiente para 400m
    //TODO: se puede configurar mas las configuraciones para el LoRa; por defecto no conseguia alcanzar 10Hz de frecuencia, con esto parece ser suficiente. Falta testeo
    if(state == RADIOLIB_ERR_NONE) {
        Serial.print("Se inicio bien el modulo");
    } else {
        Serial.print(F("Inicialio mal, codigo "));
        Serial.println(state);
        while (true) { delay(10); }         // Si no se inicializa bien te dice y se queda congelado en el loop
    }
    radio.setDio0Action(setFlag, RISING); // INTERRUCPCION: Cada vez que termina de TRANSMITIR o RECIBIR se ejecuta la funcion 
    // Inicializar filtro complementario
    // Valores de sigma tomados de la liberíá AltitudeEstimation
    // (simondlevy/AltitudeEstimation) como referencia:
    //   sigmaAccel = 0.0005 m/s²  → ruido del MPU6050
    //   sigmaBaro  = 0.018  m     → ruido del BME280
    //   accelThreshold = 0.1 m/s² → umbral ZUPT para detectar reposo
    altFilter.begin(0.12f,0.003f); // los pesos para el filtro complementario de la altura y la velocidad vertical, recordar que es un filtro complementario de 2do orden 
    // Calibrar barómetro: promedio de 50 lecturas como referencia
    // El filtro trabajará con altitud relativa a este punto (= 0 m)
    float sum = 0.0f;
    for (int i = 0; i < 50; i++) {
        sum += dataSensors.getBaroAltitude();
        delay(10);
    }
    baroAltitudeRef = sum / 50.0f;
    Serial.print("Altitud de referencia barómetro: ");
    Serial.print(baroAltitudeRef);
    Serial.println(" m");
 
    // Inicializar temporizador del filtro
    filterLastime = micros();
}

void loop() {
    
    // //Control ACS
    // if(millis() - lastTime >= (interval10ms * 2)){ // Cada 20ms actualizo el control del ACS, para mantener la estabilidad del Cubesat. Se puede ajustar el intervalo segun se vea necesario, pero no es necesario que sea mas rapido que esto, y si es mas lento podria afectar la estabilidad.{
    //     ACSData dataACS;
    //     dataSensors.getACSData(&dataACS); //Guardo los datos del mput TODO: FALTA OTROS PARA ALTITUD
    //     unsigned long now = millis();
    //     float dt = (now - lastTime) / 1000.0; // Convertir a segundos
        
    //     //PID
    //     //Proporcional
    //     float errorRoll = desiredRoll - dataACS.roll;
    //     float errorPitch = desiredPitch - dataACS.pitch;
    //     //Integral
    //     integralRoll += errorRoll * dt;
    //     integralPitch += errorPitch * dt;
    //     //Derivativo
    //     float derivativeRoll = (errorRoll - previousErrorRoll) / dt;
    //     float derivativePitch = (errorPitch - previousErrorPitch) / dt;
    //     //Salida del PID
    //     float outputRoll = Kp * errorRoll + Ki * integralRoll + Kd * derivativeRoll;
    //     float outputPitch = Kp * errorPitch + Ki * integralPitch + Kd * derivativePitch;
    //     //Actualizar errores anteriores y tiempo
    //     previousErrorRoll = errorRoll;
    //     previousErrorPitch = errorPitch;
    //     lastTime = now;

    //     //Controlar Reaction Wheel
    //     acs.setPitchOutput(outputPitch);
    //     acs.setRollOutput(outputRoll); // IMPORTNTE TODO: CLAMPEAR LOS OUPUTS PARA QUE SEAN LOGICOS
    // }

    
    //ETAPA DE VOLVER A ESCUCHAR DESPUES DE TRANSMITIR O RECIBIR
    if(operationDone) { // Si acaba de termina algo(mandar o incluso recibir, notese que cuando el starReceive recibe tmb deja de escuchar)
        operationDone = false;
    
        if(transmitFlag){   
            // Siempre que se mande algo, y llege la confirmacion de que termino (operationDone), debo pasar a que escuchar,
            // por defecto debe estar siempre escuchando. Solo TX cuando sea necesario.
            radio.startReceive();
            transmitFlag = false;
        } else {        // Si acaba de recibir algo
            String str;
            int stateReceive = radio.readData(str); // Lo guarda en str, y el estado de la recepcion en stateReceive (si hubo error o no)
    
            if( generalState == 0) { // Recibio algo y estamos en la fase 0 (Pulsos del CB y espera del ACK)
                if (stateReceive == RADIOLIB_ERR_NONE){
                    Serial.println(F("Se recibio un paquete "));
                    // Imprimir el content
                    Serial.print(F("Data:\t\t"));
                    Serial.println(str);
                    int pos = str.indexOf('&'); // El formato del mensaje que se espera es "ack&comando", por lo que busco el & para separar 
                    String parte1;              // el ack del comando, si es que llega un comando junto con el ack. Si no llega comando, entonces el mensaje es solo "ack"
                    String parte2;
                    if(pos != -1) {
                        parte1 = str.substring(0, pos);
                        parte2 = str.substring(pos + 1);
                    } else {
                        parte1 = str;
                        parte2 = "";
                    }
                    if(parte1 == "ack"){
                        generalState = 1; // Paso a estado standby
                        Serial.print("Se recibio ack de la estacion a Tierra");
                        if(parte2 != ""){   // Si ademas del ack, llega un comando para cambiar de estado, lo cambio, sino, me quedo en standby esperando comandos futuros
                            generalState = getStateByCmd(parte2);
                            Serial.print("Se recibio comando a cambiar a estado de: ");
                            Serial.print(generalState); // IMPORTANTE: ACA HAY UN PROEBLEA, NO VUELVO A ESCUCHAR IMPORANTE !!! Soluciuonado en *(*)
                        }
                    }
        
                }
            } else { // Si ya paso el inicio generalState = 0, solo me fijo en los cmd
                if(stateReceive == RADIOLIB_ERR_NONE) {
                    Serial.print("Se recibio un comando \\n");
                    Serial.print("Data:\t\t");
                    Serial.println(str);
                    generalState = getStateByCmd(str);
                }
            }
            radio.startReceive();   //  *(*) Vuelvo a escuchar despues de recibir, para seguir recibiendo futuros comandos. Siempre debe estar escuchando por defecto, solo transmite cuando es necesario.
        }
    }

    while (GPSserial.available()) {     //Si que hay una lectura entonces se lo paso al encoder(parser) siempre. TODO: Revisar si esto seria util colocarlo
        gps.encode(GPSserial.read());   
    }
    if(generalState == 0){
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis >= interval1Sec) {
            previousMillis = currentMillis;
            // transmissionState = radio.startTransmit("Pulso");   
            // Nota: No uso starTransmit(), xq esta es no bloqueante y requeriria usar la iterrupcion cuando termina,
            // pero no es necesario realizar otras cosas en paralelo. Por lo que uso el transmit, que bloquea el codigo
            // hasta que se envia(Al final si uso startTransmit, no espero a que termine, TODO: revisar cual seria mejor si bloqueante o no)
            TelemetryPacket hb = {}; // Creo un paquete vacio
            hb.TYPE = 0; // Le asigno tipo 0 (pulso) al paquete para que el receptor sepa que solo es un pulso
            transmissionState = radio.startTransmit((uint8_t*)&hb, sizeof(hb));    // Transmito pulso desde el cubesat, y NO espero  que se mande completamente
            transmitFlag = true;    //// startTransmit trabaja con datos binarios (bytes). Le paso la direccion de mi struct y su tamaño para que lo trate como un arreglo de bytes y lo transmita completo.
            Serial.print("Heart Beat sent. \n");
        }
    }

    if(generalState == 1){ // Estado de STANDBTY, solo manda un pulso cada 1 segundo para decir que esta vivo, y escucha a tierra por si le llegan comandos para cambiar de estado
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis >= interval1Sec){
            previousMillis = currentMillis;
            TelemetryPacket stbyPkt = {}; // Creo paquete vacio
            stbyPkt.TYPE = 1; // Tipo 1 para el StandBy
            stbyPkt.VOLT = analogRead(voltagePin); // TODO: CONVERSION A VOLTIOS
            radio.startTransmit((uint8_t*)&stbyPkt, sizeof(stbyPkt));  // Solo manda pa saber que esta vivo, cada 1 segundo
            transmitFlag = true;
            Serial.print("StandBy Packet Sent. \n");
        }
    }

    if(generalState == 2){     //Estado de tomar datos basicos, manda un paquete con algunos datos basicos cada 100 ms, y escucha a tierra por si le llegan comandos para cambiar de estado
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis >= interval100ms){
            previousMillis = currentMillis;
            delay(12); // Delay imaginario
            TelemetryPacket bd = {};
            bd.TYPE = 2; // Tipo 2 para el Dato Basico
            radio.startTransmit((uint8_t*)&bd, sizeof(bd)); 
            transmitFlag = true;
            Serial.print("Basic Data Packet Sent. \n");
        }
    } 

    if(generalState == 3){      //Estado de tomar datos AVANZADOS, manda un paquete con algunos datos basicos cada 100 ms, y escucha a tierra por si le llegan comandos para cambiar de estado
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis >= interval100ms){
            previousMillis = currentMillis;
            TelemetryPacket pkt; // CREO EL PAQUETE EN BASE AL STRUCT DEFINIDO EN SENSORS.HPP
            dataSensors.save_bmeDATA(&pkt); //GUARDA LOS DATOS DEL BME280 EN EL PAQUETE
            dataSensors.save_ens160DataNATH21(&pkt); //GUARDA LOS DATOS DEL ENS160 EN EL PAQUETE
            dataSensors.save_ltr390DATA(&pkt); //GUARDA LOS DATOS DEL LTR390 EN EL PAQUETE
            dataSensors.save_voltage(&pkt); //GUARDA EL VOLTAJE EN EL PAQUETE
            dataSensors.save_acsDATA(&pkt); //GUARDA LOS DATOS DEL ACS EN EL PAQUETE, INCLUYENDO INCX E INCY CALCULADOS A PARTIR DE LOS DATOS DEL ACELEROMETRO Y GIROSCOPIO
            dataSensors.saveTime(&pkt); //GUARDA EL TIEMPO EN EL PAQUETE, EN DECIMAS DE SEGUNDOS DESDE QUE SE PRENDIO EL CUBESAT, PARA QUE LA ESTACION A TIERRA PUEDA SABER CUANDO FUERON TOMADOS LOS DATOS DE LOS SENSORES, Y ASI PODER HACER ANALISIS TEMPORALES O COSAS ASI. SI NO SE GUARDA EL TIEMPO, LA ESTACION A TIERRA NO VA A SABER CUANDO FUERON TOMADOS LOS DATOS, Y ESO VA A LIMITAR MUCHO LO QUE SE PUEDE HACER CON LOS DATOS.
            //GPS
            while(GPSserial.available() > 0){
                gps.encode(GPSserial.read());
            }
            //TODO: revisar si es que esta updateadad con .isUpdated() o algo asi, para no leer valores viejos, o si es que esta validos con .isValid(), para no leer valores erroneos. Por ahora lo dejo asi, pero seria bueno mejorar eso.
            float lat_deg = gps.location.isValid() ? gps.location.lat() : 0.0f; // grados, valor por defecto en caso de que no se pueda leer el GPS
            float lon_deg = gps.location.isValid() ? gps.location.lng() : 0.0f; // grados, valor por defecto en caso de que no se pueda leer el GPS
            float alt_m_gps = gps.altitude.isValid() ? (float)gps.altitude.meters() : 0.0f;
            float hvel_ms = gps.speed.isValid()     ? (float)gps.speed.mps() : 0.0f; // m/s, velocaidad horizontal calculada a partir de la velocidad sobre el suelo (ground speed) que da el GPS.
                        
        // 1. Tiempo transcurrido
        unsigned long now = micros();
        float dt = (float)(now - filterLastime) / 1000000.0f;
        filterLastime = now;

        // 2. Preparar aceleración (restar gravedad y convertir a metros)
        float az_neta = ((float)pkt.ACCZ / 1000.0f) - 9.81f;

        // 3. Preparar altura barométrica (relativa al suelo)
        float baro_relativa = dataSensors.getBaroAltitude() - baroAltitudeRef;

        // 4. ¡LLAMAR A LA FUNCIÓN QUE CREAMOS!
        altFilter.estimate(az_neta, baro_relativa, dt);

            // === ARMAR PAQUETE ===
            pkt.TYPE = 3;
            pkt.LON  = (int32_t)  lroundf(lon_deg  * 10000000.0f);
            pkt.LAT  = (int32_t)  lroundf(lat_deg  * 10000000.0f);
            pkt.VVEL = (int16_t)  lroundf(altFilter.estimatedVelocity  * 10.0f); //Aca deberia ser el 
            pkt.ALT = (int16_t)  lround (altFilter.estimatedAltitude * 10.0f); // m*10
            //TODO: revisar calculo de altitud, ahora solo estoy guardando de frente la altura del bme(libreria de sensores)
            radio.startTransmit((uint8_t*)&pkt, sizeof(pkt));  
            transmitFlag = true;                                //MUY IMPORTANTE COLOCAR EL TRANSMIT FLAG EN TRUE, PORQUE SI NO, CUANDO SE MANDE EL PAQUETE, Y SE LEVANTE LA INTERRUPCION DE QUE SE TERMINO DE MANDAR, NO VA A SABER QUE TENIA QUE VOLVER A ESCUCHAR, POR LO QUE SE QUEDARIA SIN HACER NADA DESPUES DE MANDAR EL PRIMER PAQUETE. CON EL FLAG EN TRUE, CUANDO SE LEVANTE LA INTERRUPCION DE QUE SE TERMINO DE MANDAR, VA A PONER POR DEFECTO A ESCUCHAR NUEVAMENTE.
            Serial.print("Mando Datos Completos \n");
        }
    }

    if(generalState == -1){     //ESTADO DEBUG, NOSE, POR PARA TESTAR
        Serial.print("Morimos");
        while(1){
            delay(100);
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

    return generalState;
}


