#ifndef CUBESAT_HPP
#define CUBESAT_HPP

#include <Arduino.h>

#include "ACSController.hpp"
#include "AltitudeFilter.hpp"
#include "EjectionSystem.hpp"
#include "SensorManager.hpp"
#include "GPSModule.hpp"
#include "RadioCommunication.hpp"
#include "TelemetryManager.hpp"
#include "Estructuras.hpp"

class CubeSat {
private:
    ACSController controladorACS;
    AltitudeFilter filtroAltitud;
    EjectionSystem sistemaEyeccion;
    SensorManager sensores;
    GPSModule moduloGPS;
    RadioCommunication radio;
    TelemetryManager telemetria;

    EstadoCubeSat estadoActual;

    DatosSensores datosSensores;
    DatosGPS datosGPS;
    TelemetryPacket paquete;

    float altitudReferenciaBarometro;

    unsigned long ultimoEnvioMillis;
    unsigned long ultimoFiltroMillis;
    unsigned long ultimoFiltroMicros;

    unsigned long inicioPruebaMotoresMillis;
    bool segundoPasoMotoresRealizado;

    void calibrarBarometro();

    void actualizarPruebaMotores();
    void actualizarFiltroAltitud();

    void procesarMensajesRadio();
    void procesarMensaje(String mensaje);

    EstadoCubeSat obtenerEstadoPorComando(
        String comando
    );

    void ejecutarEstadoActual();

    void ejecutarEsperandoACK();
    void ejecutarStandBy();
    void ejecutarTelemetriaBasica();
    void ejecutarTelemetriaCompleta();
    void ejecutarDebug();

    void enviarPaquete();

public:
    CubeSat();

    void iniciar();
    void actualizar();
};

#endif
