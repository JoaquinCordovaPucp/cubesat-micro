#ifndef ALTITUDE_EKF_HPP
#define ALTITUDE_EKF_HPP

#include <math.h>
#include <string.h>

class AltitudeEKF {
private:
    // estado[0] = altitud estimada
    // estado[1] = velocidad vertical estimada
    float estado[2];

    // Matriz de covarianza actual
    float covarianza[2][2];

    // Matriz de covarianza predicha
    float covarianzaPredicha[2][2];

    // Ruido del proceso
    float ruidoProceso[2];

    // Ruido de medicion del barometro
    float ruidoMedicion;

    // Tiempo de muestreo
    float dt;

public:
    AltitudeEKF();

    void setFrecuenciaMuestreo(float frecuenciaHz);
    void setDeltaTiempo(float deltaTiempo);

    void setEstadoInicial(
        float altitudInicial,
        float velocidadInicial
    );

    void setCovarianzaInicial(
        float varianzaAltitud,
        float varianzaVelocidad
    );

    void setRuidoProceso(
        float ruidoAltitud,
        float ruidoVelocidad
    );

    void setRuidoMedicion(float ruidoBarometro);

    void predecir(float aceleracionVertical);
    void corregir(float altitudBarometrica);

    void actualizar(
        float aceleracionVertical,
        float altitudBarometrica
    );

    void actualizar(
        float aceleracionVertical,
        float altitudBarometrica,
        float deltaTiempo
    );

    float getAltitud() const;
    float getVelocidadVertical() const;
};

#endif
