#include "AltitudeEKF.hpp"

AltitudeEKF::AltitudeEKF() {
    memset(estado, 0, sizeof(estado));
    memset(covarianza, 0, sizeof(covarianza));
    memset(
        covarianzaPredicha,
        0,
        sizeof(covarianzaPredicha)
    );
    memset(
        ruidoProceso,
        0,
        sizeof(ruidoProceso)
    );

    // 50 Hz por defecto
    dt = 0.02f;

    // Ruido inicial del barometro
    ruidoMedicion = 0.25f;

    // Incertidumbre inicial
    covarianza[0][0] = 0.5f;
    covarianza[1][1] = 0.5f;

    // Ruido inicial del modelo
    ruidoProceso[0] = 0.01f;
    ruidoProceso[1] = 0.10f;
}

void AltitudeEKF::setFrecuenciaMuestreo(
    float frecuenciaHz
) {
    if (frecuenciaHz > 0.0f) {
        dt = 1.0f / frecuenciaHz;
    }
}

void AltitudeEKF::setDeltaTiempo(
    float deltaTiempo
) {
    if (
        deltaTiempo > 0.0f &&
        deltaTiempo <= 0.5f
    ) {
        dt = deltaTiempo;
    }
}

void AltitudeEKF::setEstadoInicial(
    float altitudInicial,
    float velocidadInicial
) {
    estado[0] = altitudInicial;
    estado[1] = velocidadInicial;
}

void AltitudeEKF::setCovarianzaInicial(
    float varianzaAltitud,
    float varianzaVelocidad
) {
    covarianza[0][0] = varianzaAltitud;
    covarianza[0][1] = 0.0f;
    covarianza[1][0] = 0.0f;
    covarianza[1][1] = varianzaVelocidad;
}

void AltitudeEKF::setRuidoProceso(
    float ruidoAltitud,
    float ruidoVelocidad
) {
    ruidoProceso[0] = ruidoAltitud;
    ruidoProceso[1] = ruidoVelocidad;
}

void AltitudeEKF::setRuidoMedicion(
    float ruidoBarometro
) {
    ruidoMedicion = ruidoBarometro;
}

void AltitudeEKF::predecir(
    float aceleracionVertical
) {
    estado[0] =
        estado[0] +
        estado[1] * dt +
        0.5f *
        aceleracionVertical *
        dt *
        dt;

    estado[1] =
        estado[1] +
        aceleracionVertical * dt;

    covarianzaPredicha[0][0] =
        covarianza[0][0] +
        dt * covarianza[1][0] +
        dt * covarianza[0][1] +
        dt * dt * covarianza[1][1] +
        ruidoProceso[0];

    covarianzaPredicha[0][1] =
        covarianza[0][1] +
        dt * covarianza[1][1];

    covarianzaPredicha[1][0] =
        covarianza[1][0] +
        dt * covarianza[1][1];

    covarianzaPredicha[1][1] =
        covarianza[1][1] +
        ruidoProceso[1];
}

void AltitudeEKF::corregir(
    float altitudBarometrica
) {
    float errorAltitud;
    float covarianzaMedicion;
    float gananciaAltitud;
    float gananciaVelocidad;

    errorAltitud =
        altitudBarometrica -
        estado[0];

    covarianzaMedicion =
        covarianzaPredicha[0][0] +
        ruidoMedicion;

    if (
        fabs(covarianzaMedicion) <
        0.000001f
    ) {
        covarianzaMedicion =
            0.000001f;
    }

    gananciaAltitud =
        covarianzaPredicha[0][0] /
        covarianzaMedicion;

    gananciaVelocidad =
        covarianzaPredicha[1][0] /
        covarianzaMedicion;

    estado[0] =
        estado[0] +
        gananciaAltitud *
        errorAltitud;

    estado[1] =
        estado[1] +
        gananciaVelocidad *
        errorAltitud;

    covarianza[0][0] =
        (1.0f - gananciaAltitud) *
        covarianzaPredicha[0][0];

    covarianza[0][1] =
        (1.0f - gananciaAltitud) *
        covarianzaPredicha[0][1];

    covarianza[1][0] =
        covarianzaPredicha[1][0] -
        gananciaVelocidad *
        covarianzaPredicha[0][0];

    covarianza[1][1] =
        covarianzaPredicha[1][1] -
        gananciaVelocidad *
        covarianzaPredicha[0][1];
}

void AltitudeEKF::actualizar(
    float aceleracionVertical,
    float altitudBarometrica
) {
    predecir(aceleracionVertical);
    corregir(altitudBarometrica);
}

void AltitudeEKF::actualizar(
    float aceleracionVertical,
    float altitudBarometrica,
    float deltaTiempo
) {
    setDeltaTiempo(deltaTiempo);

    actualizar(
        aceleracionVertical,
        altitudBarometrica
    );
}

float AltitudeEKF::getAltitud() const {
    return estado[0];
}

float AltitudeEKF::getVelocidadVertical() const {
    return estado[1];
}
