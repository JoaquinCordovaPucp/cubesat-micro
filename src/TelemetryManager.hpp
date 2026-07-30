#ifndef TELEMETRY_MANAGER_HPP
#define TELEMETRY_MANAGER_HPP

#include <Arduino.h>
#include "Estructuras.hpp"

class TelemetryManager {
private:
    void limpiarPaquete(TelemetryPacket *paquete);

    void guardarTiempo(TelemetryPacket *paquete);

    void guardarDatosSensores(TelemetryPacket *paquete, DatosSensores *datosSensores);

    void guardarDatosGPS(
        TelemetryPacket *paquete,
        DatosGPS *datosGPS
    );

    void guardarDatosFiltro(
        TelemetryPacket *paquete,
        float altitud,
        float velocidadVertical
    );

public:
    TelemetryManager();

    void crearHeartbeat(
        TelemetryPacket *paquete
    );

    void crearStandBy(
    TelemetryPacket *paquete,
    uint16_t lecturaVoltajeADC
    );
    
    void crearPaqueteBasico(
        TelemetryPacket *paquete
    );

    void crearPaqueteCompleto(
        TelemetryPacket *paquete,
        DatosSensores *datosSensores,
        DatosGPS *datosGPS,
        float altitud,
        float velocidadVertical
    );
};

#endif