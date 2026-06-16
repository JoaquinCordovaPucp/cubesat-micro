#ifndef SENSORS_HPP
#define SENSORS_HPP

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include "Adafruit_LTR390.h"
#include <Adafruit_ICM20948.h>
#include <Wire.h>
#define ENS160_I2C_ADDRESS 0x52


#pragma pack(push, 1)
//Struct del paquete:
struct TelemetryPacket {
    uint8_t  TYPE;
    uint16_t SEQ;
    uint32_t TIME;   // Décimas de segundo
    uint32_t FLAGS;  // Campos válidos

    uint16_t VOLT;   // mV

    int16_t PITCH;   // Valor transmitido * 1000
    int16_t ROLL;    // Valor transmitido * 1000

    int32_t LON;     // Grados * 1e7
    int32_t LAT;     // Grados * 1e7

    int16_t VVEL;    // m/s * 10

    uint32_t PRES;   // Pa
    uint16_t TEMP;   // K * 100

    uint16_t ECO2;   // ppm
    uint16_t ETOH;   // ppm
    uint8_t  AQI;    // 1-5
    uint16_t UV;     // UV * 100

    int16_t GYRX;    // rad/s * 1000
    int16_t GYRY;    // rad/s * 1000
    int16_t GYRZ;    // rad/s * 1000

    int16_t ACCX;    // m/s² * 1000
    int16_t ACCY;    // m/s² * 1000
    int16_t ACCZ;    // m/s² * 1000

    int16_t ALT;     // m * 10

    uint16_t CHK;    // CRC-16
};



#pragma pack(pop)

static_assert(
    sizeof(TelemetryPacket) == 56,
    "TelemetryPacket debe medir 56 bytes"
);


struct ACSData {
    float incx_rad;
    float incy_rad;
    float gyrox;
    float gyroy;
    float gyroz;
    float acex;
    float acey;
    float acez;
    float roll;
    float pitch;
};


class Sensors {                 //Se crea una clase para manejar todos los sensores, con sus respectivas funciones de inicializacion y lectura, 
public:                         //para que el codigo del cubesat quede mas ordenado. Ademas, si se quiere cambiar algun sensor o agregar uno nuevo, se puede hacer facilmente modificando esta clase sin tener que tocar el codigo del cubesat.
    Adafruit_BME280 bme;        //Se declaran los objetos de cada sensor y podran ser accesados asi: 
    Adafruit_LTR390 ltr390;
    Adafruit_ICM20948 icm;
    Adafruit_AHTX0 aht;
    // Variables para guardar la calibración
    float offsetX= 0.0;
    float offsetY= 0.0;
    float offsetZ= 0.0;
    //Definicion de funciones auxiliares relacionadas a los sensores
    void init(HardwareSerial* serial);
    void startENS160StandardMeasure();
    void save_bmeDATA(struct TelemetryPacket* data);
    void save_ens160DataNATH21(struct TelemetryPacket* data);
    void save_ltr390DATA(struct TelemetryPacket* data);
    void getACSData(struct ACSData* data);
    void save_voltage(struct TelemetryPacket* data);
    void save_acsDATA(struct TelemetryPacket* data);
    void saveTime(struct TelemetryPacket* data);
    float getBaroAltitude();
    // float readUVI();
private:
    float readUVI();
};

#endif // SENSORS_HPP