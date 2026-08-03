#include "TelemetryManager.hpp"

#include <math.h>

TelemetryManager::TelemetryManager() {
    numeroSecuencia = 0;
}

void TelemetryManager::limpiarPaquete(
    TelemetryPacket *paquete
) {
    TelemetryPacket paqueteVacio = {};

    *paquete = paqueteVacio;
}

void TelemetryManager::guardarTiempo(
    TelemetryPacket *paquete
) {
    paquete->TIME = millis() / 100;
}

void TelemetryManager::guardarDatosSensores(
    TelemetryPacket *paquete,
    DatosSensores *datosSensores
) {
    paquete->VOLT = (uint16_t)lroundf(
        datosSensores->voltajeMilivoltios
    );

    paquete->TEMP = (uint16_t)(
    datosSensores->temperaturaKelvin * 100.0f
    );

    paquete->PRES = (uint32_t)(
        datosSensores->presionPascales
    );

    paquete->ECO2 = (uint16_t)(
        datosSensores->eco2
    );

    paquete->ETOH = (uint16_t)(
        datosSensores->tvoc
    );

    paquete->AQI = (uint8_t)(
        datosSensores->calidadAire
    );

    paquete->UV = (uint16_t)(
    datosSensores->indiceUV * 100.0f
    );

    paquete->GYRX = (int16_t)lroundf(
        datosSensores->giroscopioX * 1000.0f
    );

    paquete->GYRY = (int16_t)lroundf(
        datosSensores->giroscopioY * 1000.0f
    );

    paquete->GYRZ = (int16_t)lroundf(
        datosSensores->giroscopioZ * 1000.0f
    );

    paquete->ACCX = (int16_t)lroundf(
        datosSensores->aceleracionX * 1000.0f
    );

    paquete->ACCY = (int16_t)lroundf(
        datosSensores->aceleracionY * 1000.0f
    );

    paquete->ACCZ = (int16_t)lroundf(
        datosSensores->aceleracionZ * 1000.0f
    );

    paquete->ROLL = (int16_t)lroundf(
        datosSensores->roll * 1000.0f
    );

    paquete->PITCH = (int16_t)lroundf(
        datosSensores->pitch * 1000.0f
    );

    if (datosSensores->calidadAireValida == true) {
        paquete->FLAGS =
            paquete->FLAGS |
            (1UL << 9);

        paquete->FLAGS =
            paquete->FLAGS |
            (1UL << 10);

        paquete->FLAGS =
            paquete->FLAGS |
            (1UL << 11);
    }

    if (datosSensores->radiacionUVValida == true) {
        paquete->FLAGS =
            paquete->FLAGS |
            (1UL << 12);
    }
}

void TelemetryManager::guardarDatosGPS(
    TelemetryPacket *paquete,
    DatosGPS *datosGPS
) {
    paquete->LAT = (int32_t)lroundf(
        datosGPS->latitud * 10000000.0f
    );

    paquete->LON = (int32_t)lroundf(
        datosGPS->longitud * 10000000.0f
    );

}

void TelemetryManager::guardarDatosFiltro(
    TelemetryPacket *paquete,
    float altitud,
    float velocidadVertical
) {
    paquete->ALT = (int16_t)lroundf(
        altitud * 10.0f
    );

    paquete->VVEL = (int16_t)lroundf(
        velocidadVertical * 10.0f
    );
}

void TelemetryManager::crearHeartbeat(
    TelemetryPacket *paquete
) {
    limpiarPaquete(paquete);
    guardarSecuencia(paquete);
    guardarTiempo(paquete);
    paquete->TYPE = 0;
}

void TelemetryManager::crearStandBy(
    TelemetryPacket *paquete,
    float voltajeMilivoltios
) {
    limpiarPaquete(paquete);
    guardarSecuencia(paquete);
    guardarTiempo(paquete);

    paquete->TYPE = 1;

    paquete->VOLT =
        (uint16_t)lroundf(
            voltajeMilivoltios
        );
}

void TelemetryManager::crearPaqueteBasico(
    TelemetryPacket *paquete
) {
    limpiarPaquete(paquete);
    guardarSecuencia(paquete);
    guardarTiempo(paquete);

    paquete->TYPE = 2;
}

void TelemetryManager::crearPaqueteCompleto(
    TelemetryPacket *paquete,
    DatosSensores *datosSensores,
    DatosGPS *datosGPS,
    float altitud,
    float velocidadVertical
) {
    limpiarPaquete(paquete);
    guardarSecuencia(paquete);

    paquete->TYPE = 3;

    guardarTiempo(paquete);

    guardarDatosSensores(
        paquete,
        datosSensores
    );

    guardarDatosGPS(
        paquete,
        datosGPS
    );

    guardarDatosFiltro(
        paquete,
        altitud,
        velocidadVertical
    );
}

void TelemetryManager::guardarSecuencia(
    TelemetryPacket *paquete
) {
    paquete->SEQ = numeroSecuencia;

    numeroSecuencia++;
}

void TelemetryManager::crearPaquetePostCaida(
    TelemetryPacket *paquete,
    float voltajeMilivoltios,
    DatosGPS *datosGPS
) {
    limpiarPaquete(paquete);

    guardarSecuencia(paquete);
    guardarTiempo(paquete);

    paquete->TYPE = 4;

    paquete->VOLT =
        (uint16_t)lroundf(
            voltajeMilivoltios
        );

    guardarDatosGPS(
        paquete,
        datosGPS
    );
}