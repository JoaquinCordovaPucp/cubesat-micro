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
    uint32_t construirFlags(
        DatosSensores *datosSensores,
        DatosGPS *datosGPS,
        bool paracaidasHabilitado,
        bool paracaidasArmado,
        bool primeraEtapaActivada,
        bool segundaEtapaActivada,
        bool aterrizajeDetectado
    );
    
public:
    TelemetryManager();
    void crearHeartbeat(
        TelemetryPacket *paquete,
        bool paracaidasHabilitado,
        bool paracaidasArmado,
        bool primeraEtapaActivada,
        bool segundaEtapaActivada,
        bool aterrizajeDetectado
    );
    void crearStandBy(
        TelemetryPacket *paquete,
        float voltajeMilivoltios,
        bool paracaidasHabilitado,
        bool paracaidasArmado,
        bool primeraEtapaActivada,
        bool segundaEtapaActivada,
        bool aterrizajeDetectado
    );

    void crearPaqueteBasico(
        TelemetryPacket *paquete,
        bool paracaidasHabilitado,
        bool paracaidasArmado,
        bool primeraEtapaActivada,
        bool segundaEtapaActivada,
        bool aterrizajeDetectado
    );

    void crearPaqueteCompleto(
        TelemetryPacket *paquete,
        DatosSensores *datosSensores,
        DatosGPS *datosGPS,
        float altitud,
        float velocidadVertical,
        bool paracaidasHabilitado,
        bool paracaidasArmado,
        bool primeraEtapaActivada,
        bool segundaEtapaActivada,
        bool aterrizajeDetectado
    );

    void crearPaquetePostCaida(
        TelemetryPacket *paquete,
        float voltajeMilivoltios,
        DatosGPS *datosGPS,
        bool paracaidasHabilitado,
        bool paracaidasArmado,
        bool primeraEtapaActivada,
        bool segundaEtapaActivada,
        bool aterrizajeDetectado
    );
};

#endif