//
// Created by Francis on 17/05/2026.
//

#include "AltitudeEKF.hpp"
AltitudeEKF::AltitudeEKF() {
    memset(estado, 0, sizeof(estado));
    memset(covarianza, 0, sizeof(covarianza));
    memset(covarianzaPredicha, 0, sizeof(covarianzaPredicha));
    memset(ruidoProceso, 0, sizeof(ruidoProceso));

    // Valores iniciales seguros
    dt = 0.02f;              // 50 Hz por defecto
    ruidoMedicion = 0.25f;   // ruido inicial del barometro

    // Incertidumbre inicial del filtro
    covarianza[0][0] = 0.5f; // altitud
    covarianza[1][1] = 0.5f; // velocidad

    // Ruido inicial del modelo
    ruidoProceso[0] = 0.01f;
    ruidoProceso[1] = 0.10f;
}

void AltitudeEKF::setFrecuenciaMuestreo(float frecuenciaHz) {
    if (frecuenciaHz > 0.0f) {
        dt = 1.0f / frecuenciaHz;
    }
}

void AltitudeEKF::setDeltaTiempo(float deltaTiempo) {
    // Proteccion: evita integrar con tiempos negativos, cero o saltos grandes.
    // En el CubeSat normalmente sera cercano a 0.02 s porque el filtro corre a 50 Hz.
    if (deltaTiempo > 0.0f && deltaTiempo <= 0.5f) {
        dt = deltaTiempo;
    }
}

void AltitudeEKF::setEstadoInicial(float altitudInicial, float velocidadInicial) {
    estado[0] = altitudInicial;
    estado[1] = velocidadInicial;
}

void AltitudeEKF::setCovarianzaInicial(float varianzaAltitud, float varianzaVelocidad) {
    covarianza[0][0] = varianzaAltitud;
    covarianza[0][1] = 0.0f;
    covarianza[1][0] = 0.0f;
    covarianza[1][1] = varianzaVelocidad;
}

void AltitudeEKF::setRuidoProceso(float ruidoAltitud, float ruidoVelocidad) {
    ruidoProceso[0] = ruidoAltitud;
    ruidoProceso[1] = ruidoVelocidad;
}

void AltitudeEKF::setRuidoMedicion(float ruidoBarometro) {
    ruidoMedicion = ruidoBarometro;
}

void AltitudeEKF::predecir(float aceleracionVertical) {
    /*
      MODELO DE MOVIMIENTO:

      altitud = altitud + velocidad * dt + 0.5 * aceleracion * dt^2
      velocidad = velocidad + aceleracion * dt

      IMPORTANTE:
      aceleracionVertical debe venir sin gravedad.
      Es decir, si el sensor esta quieto, debe estar cerca de 0.
    */

    estado[0] = estado[0] + estado[1] * dt + 0.5f * aceleracionVertical * dt * dt;
    estado[1] = estado[1] + aceleracionVertical * dt;

    /*
      Prediccion de covarianza:

      F = [1  dt]
          [0   1]

      P_predicha = F * P * F^T + Q
    */

    covarianzaPredicha[0][0] =
        covarianza[0][0]
        + dt * covarianza[1][0]
        + dt * covarianza[0][1]
        + dt * dt * covarianza[1][1]
        + ruidoProceso[0];

    covarianzaPredicha[0][1] =
        covarianza[0][1]
        + dt * covarianza[1][1];

    covarianzaPredicha[1][0] =
        covarianza[1][0]
        + dt * covarianza[1][1];

    covarianzaPredicha[1][1] =
        covarianza[1][1]
        + ruidoProceso[1];
}

void AltitudeEKF::corregir(float altitudBarometrica) {
    /*
      El barometro mide directamente la altitud.

      z = altitudBarometrica
      H = [1  0]
    */

    float errorAltitud = altitudBarometrica - estado[0];

    // S = H * P_predicha * H^T + R
    float S = covarianzaPredicha[0][0] + ruidoMedicion;

    // Proteccion para evitar division entre cero
    if (fabs(S) < 0.000001f) {
        S = 0.000001f;
    }

    // Ganancia de Kalman
    float gananciaAltitud = covarianzaPredicha[0][0] / S;
    float gananciaVelocidad = covarianzaPredicha[1][0] / S;

    // Correccion del estado
    estado[0] = estado[0] + gananciaAltitud * errorAltitud;
    estado[1] = estado[1] + gananciaVelocidad * errorAltitud;

    // Correccion de la covarianza
    covarianza[0][0] = (1.0f - gananciaAltitud) * covarianzaPredicha[0][0];
    covarianza[0][1] = (1.0f - gananciaAltitud) * covarianzaPredicha[0][1];

    covarianza[1][0] = covarianzaPredicha[1][0] - gananciaVelocidad * covarianzaPredicha[0][0];
    covarianza[1][1] = covarianzaPredicha[1][1] - gananciaVelocidad * covarianzaPredicha[0][1];
}

void AltitudeEKF::actualizar(float aceleracionVertical, float altitudBarometrica) {
    predecir(aceleracionVertical);
    corregir(altitudBarometrica);
}

void AltitudeEKF::actualizar(float aceleracionVertical, float altitudBarometrica, float deltaTiempo) {
    setDeltaTiempo(deltaTiempo);
    actualizar(aceleracionVertical, altitudBarometrica);
}

float AltitudeEKF::getAltitud() const {
    return estado[0];
}

float AltitudeEKF::getVelocidadVertical() const {
    return estado[1];
}