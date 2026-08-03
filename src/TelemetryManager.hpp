#ifndef TELEMETRY_MANAGER_HPP
#define TELEMETRY_MANAGER_HPP

#include <Arduino.h>
#include "Estructuras.hpp"

class TelemetryManager {
private:
    uint16_t numeroSecuencia;
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
    void guardarSecuencia(TelemetryPacket *paquete);
    
public:
    TelemetryManager();

    void crearHeartbeat(
        TelemetryPacket *paquete
    );

    void crearStandBy(
    TelemetryPacket *paquete,
    float voltajeMilivoltios);
    
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

    void crearPaquetePostCaida(
    TelemetryPacket *paquete,
    float voltajeMilivoltios,
    DatosGPS *datosGPS
    );
};

#endif