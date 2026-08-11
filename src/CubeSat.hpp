#ifndef CUBESAT_HPP
#define CUBESAT_HPP

#include <Arduino.h>

#include "ACSController.hpp"
#include "AltitudeEKF.hpp"
#include "EjectionSystem.hpp"
#include "SensorManager.hpp"
#include "GPSModule.hpp"
#include "RadioCommunication.hpp"
#include "TelemetryManager.hpp"
#include "Estructuras.hpp"
#include "DataLogger.hpp"

class CubeSat {
private:
    ACSController controladorACS;
    AltitudeEKF  filtroAltitud;
    EjectionSystem sistemaParacaidas;
    SensorManager sensores;
    GPSModule moduloGPS;
    RadioCommunication radio;
    TelemetryManager telemetria;

    EstadoCubeSat estadoActual;

    DatosSensores datosSensores;
    DatosGPS datosGPS;
    TelemetryPacket paquete;
    DataLogger registrador;
    
    float altitudReferenciaBarometro;

    unsigned long ultimoEnvioMillis;
    unsigned long ultimoFiltroMillis;
    unsigned long ultimoFiltroMicros;

    unsigned long inicioPruebaMotoresMillis;
    bool segundoPasoMotoresRealizado;

    bool paracaidasHabilitado;
    bool paracaidasArmado;

    bool camaraActivada;

    
    bool primeraEtapaActivada;
    bool segundaEtapaActivada;

    bool vueloIniciado;
    bool aterrizajeDetectado;

    float alturaAnterior;
    float alturaMaximaAlcanzada;

    unsigned long inicioReposoMillis;
    // Guarda cuando comenzo la confirmacion
    // de los 10 segundos de 100 m
    unsigned long inicioConfirmacionArmadoMillis;
    void calibrarBarometro();

    void actualizarPruebaMotores();
    void actualizarFiltroAltitud();

    void procesarMensajesRadio();
    void procesarMensaje(int codigo);


    void ejecutarEstadoActual();

    void ejecutarEsperandoACK();
    void ejecutarStandBy();
    void ejecutarTelemetriaBasica();
    void ejecutarTelemetriaCompleta();
    void ejecutarDebug();
    void enviarPaquete();

    void imprimirPaquete(); // para probar

    void procesarComando(int comando);

    void actualizarParacaidas();

    void actualizarDeteccionAterrizaje();

    void ejecutarPostCaida(); // evaluar en funcion de la altura
        
public:
    CubeSat();

    void iniciar();
    void actualizar();
};

#endif
