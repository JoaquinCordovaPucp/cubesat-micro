#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#include "Estructuras.hpp"

class DataLogger {
private:
    File archivo;

    bool disponible;

    unsigned long ultimoFlushMillis;

    void escribirCabecera();

public:
    DataLogger();

    bool iniciar(int pinCS);

    void guardarPaquete(
        TelemetryPacket *paquete
    );

    void actualizar();

    bool estaDisponible();
};

#endif