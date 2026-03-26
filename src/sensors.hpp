#ifndef SENSORS_HPP
#define SENSORS_HPP

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include "SparkFun_ENS160.h"
#include <Adafruit_AHTX0.h>
#include "Adafruit_LTR390.h"
#include <Adafruit_MPU6050.h>

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


class Sensors {                 //Se crea una clase para manejar todos los sensores, con sus respectivas funciones de inicializacion y lectura, 
public:                         //para que el codigo del cubesat quede mas ordenado. Ademas, si se quiere cambiar algun sensor o agregar uno nuevo, se puede hacer facilmente modificando esta clase sin tener que tocar el codigo del cubesat.
    Adafruit_BME280 bme;        //Se declaran los objetos de cada sensor y podran ser accesados asi: 
    SparkFun_ENS160 ens160;     //Sensors dataSensors    (crear un objeto de la clase Sensors, y guardarlo en la variable dataSensors)
    Adafruit_AHTX0 aht;         //dataSensors.mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Adafruit_LTR390 ltr390;
    Adafruit_MPU6050 mpu;
    //Definicion de funciones auxiliares relacionadas a los sensores
    void init(HardwareSerial* serial);
    void save_bmeDATA(struct TelemetryPacket* data);
    // float readUVI();
private:
    // void mostrar_contrasenha();
};

#endif // SENSORS_HPP