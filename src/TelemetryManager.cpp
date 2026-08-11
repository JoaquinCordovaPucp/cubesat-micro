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
    TelemetryPacket *paquete,
    bool paracaidasHabilitado,
    bool paracaidasArmado,
    bool primeraEtapaActivada,
    bool segundaEtapaActivada,
    bool aterrizajeDetectado
) {
    limpiarPaquete(paquete);
    guardarSecuencia(paquete);
    guardarTiempo(paquete);
    paquete->TYPE = 0;

    paquete->FLAGS = construirFlags(
        nullptr,
        nullptr,
        paracaidasHabilitado,
        paracaidasArmado,
        primeraEtapaActivada,
        segundaEtapaActivada,
        aterrizajeDetectado
    );
}

void TelemetryManager::crearStandBy(
    TelemetryPacket *paquete,
    float voltajeMilivoltios,
    bool paracaidasHabilitado,
    bool paracaidasArmado,
    bool primeraEtapaActivada,
    bool segundaEtapaActivada,
    bool aterrizajeDetectado
) {
    limpiarPaquete(paquete);
    guardarSecuencia(paquete);
    guardarTiempo(paquete);

    paquete->TYPE = 1;

    paquete->VOLT =
        (uint16_t)lroundf(
            voltajeMilivoltios
        );

    paquete->FLAGS = construirFlags(
        nullptr,
        nullptr,
        paracaidasHabilitado,
        paracaidasArmado,
        primeraEtapaActivada,
        segundaEtapaActivada,
        aterrizajeDetectado
    );
}

void TelemetryManager::crearPaqueteBasico(
    TelemetryPacket *paquete,
    bool paracaidasHabilitado,
    bool paracaidasArmado,
    bool primeraEtapaActivada,
    bool segundaEtapaActivada,
    bool aterrizajeDetectado
) {
    limpiarPaquete(paquete);
    guardarSecuencia(paquete);
    guardarTiempo(paquete);

    paquete->TYPE = 2;

    paquete->FLAGS = construirFlags(
        nullptr,
        nullptr,
        paracaidasHabilitado,
        paracaidasArmado,
        primeraEtapaActivada,
        segundaEtapaActivada,
        aterrizajeDetectado
    );
}

void TelemetryManager::crearPaqueteCompleto(
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

    paquete->FLAGS = construirFlags(
        datosSensores,
        datosGPS,
        paracaidasHabilitado,
        paracaidasArmado,
        primeraEtapaActivada,
        segundaEtapaActivada,
        aterrizajeDetectado
    );
}

void TelemetryManager::guardarSecuencia(
    TelemetryPacket *paquete
) {
    paquete->SEQ = numeroSecuencia;

    numeroSecuencia++;
}

uint32_t TelemetryManager::construirFlags(
    DatosSensores *datosSensores,
    DatosGPS *datosGPS,
    bool paracaidasHabilitado,
    bool paracaidasArmado,
    bool primeraEtapaActivada,
    bool segundaEtapaActivada,
    bool aterrizajeDetectado
) {
    uint32_t flags;

    flags = 0;

    if (datosSensores != nullptr) {
        if (datosSensores->calidadAireValida == true) {
            flags = flags | FLAG_CALIDAD_AIRE_VALIDA;
        }

        if (datosSensores->radiacionUVValida == true) {
            flags = flags | FLAG_RADIACION_UV_VALIDA;
        }

        if (datosSensores->movimientoValido == true) {
            flags = flags | FLAG_MOVIMIENTO_VALIDO;
        }
    }

    if (datosGPS != nullptr) {
        if (datosGPS->ubicacionValida == true) {
            flags = flags | FLAG_GPS_UBICACION_VALIDA;
        }

        if (datosGPS->velocidadValida == true) {
            flags = flags | FLAG_GPS_VELOCIDAD_VALIDA;
        }
    }

    if (paracaidasHabilitado == true) {
        flags = flags | FLAG_PARACAIDAS_HABILITADO;
    }

    if (paracaidasArmado == true) {
        flags = flags | FLAG_PARACAIDAS_ARMADO;
    }

    if (primeraEtapaActivada == true) {
        flags = flags | FLAG_PARACAIDAS_PRIMERA_ETAPA;
    }

    if (segundaEtapaActivada == true) {
        flags = flags | FLAG_PARACAIDAS_SEGUNDA_ETAPA;
    }

    if (aterrizajeDetectado == true) {
        flags = flags | FLAG_ATERRIZAJE_DETECTADO;
    }

    return flags;
}

void TelemetryManager::crearPaquetePostCaida(
    TelemetryPacket *paquete,
    float voltajeMilivoltios,
    DatosGPS *datosGPS,
    bool paracaidasHabilitado,
    bool paracaidasArmado,
    bool primeraEtapaActivada,
    bool segundaEtapaActivada,
    bool aterrizajeDetectado
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

    paquete->FLAGS = construirFlags(
        nullptr,
        datosGPS,
        paracaidasHabilitado,
        paracaidasArmado,
        primeraEtapaActivada,
        segundaEtapaActivada,
        aterrizajeDetectado
    );
}