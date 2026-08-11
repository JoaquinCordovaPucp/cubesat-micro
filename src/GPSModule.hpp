#ifndef GPS_MODULE_HPP
#define GPS_MODULE_HPP

#include <Arduino.h>
#include <TinyGPSPlus.h>

#include "Estructuras.hpp"

class GPSModule {
private:
    TinyGPSPlus gps;
    HardwareSerial gpsSerial;

public:
    GPSModule();

    void iniciar();
    void actualizar();

    void obtenerDatos(DatosGPS *datos);
};

#endif
