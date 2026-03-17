#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "SparkFun_ENS160.h"
#include <Adafruit_AHTX0.h>
#include "Adafruit_LTR390.h"
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h> //GPS

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
    int16_t  ACCX;   // m/s^2 * 1000
    int16_t  ACCY;   // m/s^2 * 1000
    int16_t  ACCZ;   // m/s^2 * 1000
    uint16_t ALT;    // m * 10
    uint16_t CHK;    // CRC-16
};

SX1276 radio = new Module(5, 4, 22, 3);
Adafruit_BME280 bme;
SparkFun_ENS160 ens160;
Adafruit_AHTX0 aht;
Adafruit_LTR390 ltr390;
TinyGPSPlus gps;
HardwareSerial GPSserial(2);
Adafruit_MPU6050 mpu;

int transmissionState = RADIOLIB_ERR_NONE;  // Aca se guardara el estado del radio, osea los errores (codigo), o no error (codigo 0)

unsigned long previousMillis = 0; // Stores the last time the function was executed
const long interval1Sec = 1000; 
const long interval100ms= 100;  //
volatile bool operationDone = false;
int generalState = 0; // This stores the state of the machine (0: recien prendido y emitiendo un pulso, 1: standby, 2: trasmitiendo datos basico, 3: transmitiendo datos completos)
bool transmitFlag = false; //   Si Verdadero, entonces estaba transmitiendo

void setFlag(void) {
  operationDone = true;     // Cuando termina una operacion se coloca en true, ya sea TX o RX. Util cuando se usa metodos no bloqueantes como startTransmit() o startReceive()
}

// Definicion de funciones
int getStateByCmd(String cmd);
bool verificarPaqueteDato(String str);
float readUVI();

void setup () {
    Serial.begin(115200);    
    GPSserial.begin(9600, SERIAL_8N1, 16, 17); // Inciar GPS, con los pines en rx y tx seleccionados(rx del gps va al tx, y sucesivamente)
    Wire.begin(21, 26); //Iniciar I2C (Lo uso para todos los sensores con I2C)
    if (!bme.begin(0x76)) {
        Serial.println("No se encontro BME280");
        while (1);
    }
    if(!ens160.begin(Wire, 0x52)){
        Serial.println("No se encontro ENS160");
        while(1);
    }
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);         //Configuracion del Acel y Giro MPU6050. TODO: Ver la configuracion idonea para el caso de uso

    ens160.setOperatingMode(SFE_ENS160_STANDARD);
    if ( !ltr390.begin() ) {
        Serial.println("Couldn't find LTR sensor!");
        while (1) delay(10);
    }
    ltr390.setMode(LTR390_MODE_UVS);
    ltr390.setGain(LTR390_GAIN_3);
    ltr390.setResolution(LTR390_RESOLUTION_16BIT);
    
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
    radio.setDio0Action(setFlag, RISING); // Cada vez que termina de TRANSMITIR o RECIBIR se ejecuta la funcion 
    
    // int ensStatus = ens160.getFlags();       CODIGO PARA VER SI ES QUE EL SENSOR DE CO2 ESTA TOMANDO MEDIDAS UTILES
	// Serial.print("Gas Sensor Status Flag (0 - Standard, 1 - Warm up, 2 - Initial Start Up): ");
	// Serial.println(ensStatus);
}

void loop() {
    //ETAPA DE VOLVER A ESCUCHAR DESPUES DE TRANSMITIR O RECIBIR
    if(operationDone) { // Si acaba de termina algo(mandar o incluso recibir, notese que cuando el starReceive recibe tmb deja de escuchar)
        operationDone = false;
    
        if(transmitFlag){   
            // Siempre que se mande algo, y llege la confirmacion de que termino (operationDone), debo pasar a que escuche,
            // por defecto debe estar siempre escuchando. Solo TX cuando sea necesario.
            radio.startReceive();
            transmitFlag = false;
        } else {        // Si acaba de recibir algo
            String str;
            int stateReceive = radio.readData(str); // Lo guarda en str, y el estado de la recepcion en stateReceive (si hubo error o no)
    
            if( generalState == 0) { // Recibio algo y estamos en la fase 0 (Pulsos del CB y espera del ACK)
                if (stateReceive == RADIOLIB_ERR_NONE){
                    Serial.println(F("Se recibio un paqueton"));
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
                    Serial.print("Se recibio un comando");
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
            // Functions:
            Serial.print("Mandando pulso(Heart Beat) \n");
            // transmissionState = radio.startTransmit("Pulso");   
            // Nota: No uso starTransmit(), xq esta es no bloqueante y requeriria usar la iterrupcion cuando termina,
            // pero no es necesario realizar otras cosas en paralelo. Por lo que uso el transmit, que bloquea el codigo
            // hasta que se envia(Al final si uso startTransmit, no espero a que termine, TODO: revisar cual seria mejor si bloqueante o no)
            TelemetryPacket hb = {}; // Creo un paquete vacio
            hb.TYPE = 0; // Le asigno tipo 0 (pulso) al paquete para que el receptor sepa que solo es un pulso
            transmissionState = radio.startTransmit((uint8_t*)&hb, sizeof(hb));    // Transmito pulso desde el cubesat, y espero NO que se mande completamente
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

    if(generalState == 1){ // Estado de StandBy, solo manda un pulso cada 1 segundo para decir que esta vivo, y escucha a tierra por si le llegan comandos para cambiar de estado
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis >= interval1Sec){
            previousMillis = currentMillis;
            Serial.print("StandBy \n");
            TelemetryPacket stby = {}; // Creo paquete vacio
            stby.TYPE = 1; // Tipo 1 para el StandBy
            radio.startTransmit((uint8_t*)&stby, sizeof(stby));  // Solo manda pa saber que esta vivo, cada 1 segundo
            transmitFlag = true;
        }
    }

    if(generalState == 2){     //Estado de tomar datos basicos, manda un paquete con algunos datos basicos cada 100 ms, y escucha a tierra por si le llegan comandos para cambiar de estado
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis >= interval100ms){
            previousMillis = currentMillis;
            Serial.print("Mando Dato Basico \n");
            // Aca se tomarian datos ps y calculos 
            delay(12); // Delay imaginario
            TelemetryPacket bd = {};
            bd.TYPE = 2; // Tipo 2 para el Dato Basico
            radio.startTransmit((uint8_t*)&bd, sizeof(bd)); 
            transmitFlag = true;
        }
    } 

    if(generalState == 3){      //Estado de tomar datos AVANZADOS, manda un paquete con algunos datos basicos cada 100 ms, y escucha a tierra por si le llegan comandos para cambiar de estado
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis >= interval100ms){
            previousMillis = currentMillis;
            //EL CODIGO EN COMENTARIOS SI FUNCIONA, pero para probar estoy usando el paquete definido al principio del loop, con datos inventados.
            //TODO: Descomentar y usar los datos reales, y revisar que el paquete se arme correctamente con los datos reales.
            // Serial.print("Temp: ");
            // Serial.println(bme.readTemperature());
            // if(ens160.checkDataStatus()){
            //     Serial.print("CO2 concentration: ");
            //     Serial.print(ens160.getECO2());
            //     Serial.println("ppm");
            // }

            // if (ltr390.newDataAvailable()) {
            //     uint32_t uvs = ltr390.readUVS();
            //     Serial.print(" | UVI: ");
            //     Serial.println(readUVI());
            // }
            // if (gps.location.isUpdated()) {
            //     Serial.print("Lat: ");
            //     Serial.println(gps.location.lat(), 6);

            //     Serial.print("Lon: ");
            //     Serial.println(gps.location.lng(), 6);

            //     Serial.print("Alt: ");
            //     Serial.println(gps.altitude.meters());

            //     Serial.print("Sats: ");
            //     Serial.println(gps.satellites.value());
            // }
            // Serial.print("Mando Datos Completos \n");
            // // Aca se tomarian datos ps y calculos 
            // delay(12); // Delay imaginario
            // radio.startTransmit("Datos Completos XD"); 
            // transmitFlag = true;
            //LEER SENSORES PARA ARMAR EL PAQUETE DE DATOS COMPLETOS (ANTES LO HACIA EN EL LOOP GENREAL, PERO ALGUNOS SENSORES SE DEMORABAN, Y ME BUGEABAN EL LOOP(NO ME LLEGAN LOS ACK)) 
            TelemetryPacket pkt;
            //BME280: Alta frecuencia por lo que no hay problema en leerlo a cada rato
            float temp_k  = bme.readTemperature() + 273.15f;        // Kelvin
            float alt_m_bme280 = bme.readAltitude(1013.25f);        // m   // TODO: que significa 1013.25f el "f"???
            uint32_t pres_pa = bme.readPressure();                  // Pa   // que significa el "t" del uint32_t????
            
            //ENS160: Este sensor tiene una funcion para verificar si esta listo para nueva lectura, sino lo dejo por defecto en -1 para saber. TODO: Mejrar metodo de no saber (-1)
            uint16_t eco2_ppm = 0; // ppm, valor por defecto en caso de que no se pueda leer el sensor
            if(ens160.checkDataStatus() ){ // Segun la libreria: Gas Sensor Status Flag (0 - Standard, 1 - Warm up, 2 - Initial Start Up), por lo que si es 0 esta listo para medidas utiles.
                eco2_ppm = ens160.getECO2(); // ppm
                // ens160.getAQI();     // TODO: adicionar otras lecturas
                // ens160.getETOH();    
                // ens160.getTempKelvin(); // El sensor tmb da temp , pero podria ser util usarlas para un promedio, pero necesita calibrar o compensansio o algo asi.
            }

            //LTR390
            float uv_index = 0.0f;
            if(ltr390.newDataAvailable()){
                uv_index = readUVI(); // Indice UV, calculado segun la ganancia y resolucion configurada, y el valor crudo del sensor.
            }

            //GPS
            while(GPSserial.available() > 0){
                gps.encode(GPSserial.read());
            }
            //TODO: revisar si es que esta updateadad con .isUpdated() o algo asi, para no leer valores viejos, o si es que esta validos con .isValid(), para no leer valores erroneos. Por ahora lo dejo asi, pero seria bueno mejorar eso.
            float lat_deg = gps.location.isValid() ? gps.location.lat() : 0.0f; // grados, valor por defecto en caso de que no se pueda leer el GPS
            float lon_deg = gps.location.isValid() ? gps.location.lng() : 0.0f; // grados, valor por defecto en caso de que no se pueda leer el GPS
            float alt_m_gps = gps.altitude.isValid() ? (float)gps.altitude.meters() : 0.0f;
            float hvel_ms = gps.speed.isValid()     ? (float)gps.speed.mps() : 0.0f; // m/s, velocaidad horizontal calculada a partir de la velocidad sobre el suelo (ground speed) que da el GPS.
            
            //Time
            float time_s = millis() / 1000.0f;
            
            
            //MPU6050 (giroscopio y acelerometro)
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            float incx_rad = 0.0f; // rad 
            float incy_rad = 0.0f; // rad
            float gyrox    = g.gyro.x;  // rad/s
            float gyroy    = g.gyro.y;  // rad/s
            float gyroz    = g.gyro.z;  // rad/s
            float acex    = a.acceleration.x;  // rad/s
            float acey    = a.acceleration.y;  // rad/s
            float acez    = a.acceleration.z;  // rad/s

            //POR IMPLEMENTAR:
            // Voltaje (hardcodeado todavia)
            float volt_v = 3.72f;
            float vvel_ms = 0.0f; // Velocidad vertical, se podria calcular a partir de la altitud del GPS, pero el GPS no es tan preciso para eso, por lo que lo dejo en 0 por ahora, o se podria usar el altimetro barometrico para eso, pero tambien tiene ruido, por lo que lo dejo en 0 por ahora. Se podria mejorar eso con un filtro o algo asi.

            // === ARMAR PAQUETE ===
            pkt.TYPE = 3;
            pkt.VOLT = (uint16_t) lroundf(volt_v   * 1000.0f);
            pkt.INCX = (int16_t)  lroundf(incx_rad * 1000.0f);
            pkt.INCY = (int16_t)  lroundf(incy_rad * 1000.0f);
            pkt.LON  = (int32_t)  lroundf(lon_deg  * 10000000.0f);
            pkt.LAT  = (int32_t)  lroundf(lat_deg  * 10000000.0f);
            pkt.TIME = (uint32_t) lroundf(time_s   * 10.0f);
            pkt.VVEL = (int16_t)  lroundf(vvel_ms  * 10.0f); //Aca deberia ser el 
            pkt.PRES = pres_pa;
            pkt.TEMP = (uint16_t) lroundf(temp_k   * 100.0f);
            pkt.ECO2 = eco2_ppm;
            pkt.UV   = (uint16_t) lroundf(uv_index * 100.0f);
            pkt.GYRX = (int16_t)  lroundf(gyrox    * 1000.0f);
            pkt.GYRY = (int16_t)  lroundf(gyroy    * 1000.0f);
            pkt.GYRZ = (int16_t)  lroundf(gyroz    * 1000.0f);
            pkt.ACCX = (int16_t)  lroundf(acex     * 1000.0f);
            pkt.ACCY = (int16_t)  lroundf(acey     * 1000.0f);
            pkt.ACCZ = (int16_t)  lroundf(acez     * 1000.0f);
            pkt.ALT  = (uint16_t) lroundf(((alt_m_bme280 + alt_m_gps)/2)  * 10.0f);   // Uso el promedio de BME280 y gps.

            radio.startTransmit((uint8_t*)&pkt, sizeof(pkt));   // Mando el paquete de datos completos, cada 100 ms, con los datos inventados del principio del loop, para probar que se mande bien el paquete con los datos reales despues.
            transmitFlag = true;                                //MUY IMPORTANTE COLOCAR EL TRANSMIT FLAG EN TRUE, PORQUE SI NO, CUANDO SE MANDE EL PAQUETE, Y SE LEVANTE LA INTERRUPCION DE QUE SE TERMINO DE MANDAR, NO VA A SABER QUE TENIA QUE VOLVER A ESCUCHAR, POR LO QUE SE QUEDARIA SIN HACER NADA DESPUES DE MANDAR EL PRIMER PAQUETE. CON EL FLAG EN TRUE, CUANDO SE LEVANTE LA INTERRUPCION DE QUE SE TERMINO DE MANDAR, VA A PONER POR DEFECTO A ESCUCHAR NUEVAMENTE.
            Serial.print("Mando Datos Completos \n");
        }
    }

    if(generalState == -1){     //ESTADO DEBUG, NOSE XD, POR PARA TESTAR
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


//Esta funcion biene de la libreria de Adafruit para el LTR390, pero la adapte para que devuelva el indice UV en vez del valor crudo del sensor, y ademas le puse el calculo de sensibilidad segun la ganancia y resolucion que tengo configurada, para que devuelva un valor mas realista del indice UV.
// https://github.com/adafruit/Adafruit_CircuitPython_LTR390/blob/main/adafruit_ltr390.py
float readUVI() {
  const float gain = 3.0f;              
  const float resolutionBits = 16.0f;

  float sensitivity = 2300.0f *
                      (gain / 18.0f) *
                      ((float)(1UL << 16) / (float)(1UL << 20));

  float uvs = (float)ltr390.readUVS();
  return uvs / sensitivity;
}
//Funcion de adafruit original, para referencia(en python):
// def uvi(self) -> float:
//     """Read UV count and return calculated UV Index (UVI) value based upon the rated sensitivity
//     of 1 UVI per 2300 counts at 18X gain factor and 20-bit resolution."""
//     return (
//         self.uvs
//         / (
//             (Gain.factor[self.gain] / 18)
//             * (2 ** Resolution.factor[self.resolution])
//             / (2**20)
//             * 2300
//         )
//         * self._window_factor
//     )