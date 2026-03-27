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
    uint16_t SEQ;
    uint32_t TIME;      // s * 10  -> décimas de segundo
    uint32_t FLAGS;     // Bitfield para indicar qué datos son válidos, por ejemplo: bit 0 para indicar si el campo VOLT es válido, bit 1 para INCX, etc. Esto es útil para no enviar datos erróneos o no actualizados, y para ahorrar ancho de banda al no enviar datos que no han cambiado.
    uint16_t VOLT;      // 1. mV  (V * 1000)
    int16_t  PITCH;      // 2. rad * 1000
    int16_t  ROLL;      // 3. rad * 1000
    int32_t  LON;       // 4. deg * 1e7
    int32_t  LAT;       // 5. deg * 1e7
    int16_t  VVEL;      // 6. m/s * 10
    uint32_t PRES;      // 7. Pa
    uint16_t TEMP;      // 8. K * 100
    uint16_t ECO2;      // 9. ppm
    uint16_t ETOH;     // 10. ppm
    uint8_t AQI;        // 11. 1 - Excellent, 2 - Good, 3 - Moderate, 4 - Poor, 5 - Unhealthy
    uint16_t UV;        // 12. UV * 100
    int16_t  GYRX;      // 13. rad/s * 1000
    int16_t  GYRY;      // 14. rad/s * 1000
    int16_t  GYRZ;      // 15. rad/s * 1000
    int16_t  ACCX;      // 16. m/s^2 * 1000
    int16_t  ACCY;      // 17. m/s^2 * 1000
    int16_t  ACCZ;      // 18. m/s^2 * 1000
    uint16_t ALT;    // 19. m * 10
    uint16_t CHK;    // CRC-16
};

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
    SparkFun_ENS160 ens160;     //Sensors dataSensors    (crear un objeto de la clase Sensors, y guardarlo en la variable dataSensors)
    Adafruit_AHTX0 aht;         //dataSensors.mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Adafruit_LTR390 ltr390;
    Adafruit_MPU6050 mpu;
    //Definicion de funciones auxiliares relacionadas a los sensores
    void init(HardwareSerial* serial);
    void save_bmeDATA(struct TelemetryPacket* data);
    void save_ens160DATA(struct TelemetryPacket* data);
    void save_ltr390DATA(struct TelemetryPacket* data);
    void getACSData(struct ACSData* data);
    void save_voltage(struct TelemetryPacket* data);
    void save_acsDATA(struct TelemetryPacket* data);
    void saveTime(struct TelemetryPacket* data);
    // float readUVI();
private:
    float readUVI();
};

#endif // SENSORS_HPP