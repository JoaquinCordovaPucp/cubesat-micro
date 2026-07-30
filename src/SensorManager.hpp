#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_LTR390.h>

#include "Estructuras.hpp"

class SensorManager {
private:
    Adafruit_BME280 bme;
    Adafruit_AHTX0 aht;
    Adafruit_LTR390 ltr390;

    int ultimoEco2;
    int ultimoTvoc;
    int ultimaCalidadAire;

    float ultimoIndiceUV;

    void leerBME(DatosSensores *datos);
    void leerCalidadAire(DatosSensores *datos);
    void leerRadiacionUV(DatosSensores *datos);
    void leerVoltaje(DatosSensores *datos);
    void leerMovimiento(DatosSensores *datos);

    float calcularIndiceUV();

public:
    SensorManager();

    void iniciar(HardwareSerial *serial);
    void leer(DatosSensores *datos);

    float obtenerAltitudBarometrica();
    uint16_t obtenerLecturaVoltajeADC();
    float obtenerVoltajeMilivoltios();

};

#endif
