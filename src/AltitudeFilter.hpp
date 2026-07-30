#ifndef ALTITUDE_FILTER_HPP
#define ALTITUDE_FILTER_HPP

#include <Arduino.h>
#include "Configuracion.hpp"

class AltitudeFilter {
private:
    float baroWindow[TAMANO_MEDIANA_BAROMETRO];
    int baroIndex;
    bool baroWindowFull;

    float restWindow[TAMANO_VENTANA_REPOSO];
    int restIndex;

    float calculateBarometerMedian(float newSample);
    bool isResting();

public:
    float estimatedAltitude;
    float estimatedVelocity;

    float positionGain;
    float velocityGain;
    float accelerationThreshold;

    AltitudeFilter();

    void begin(
        float positionGainValue,
        float velocityGainValue,
        float accelerationThresholdValue
    );

    void estimate(
        float verticalAcceleration,
        float barometerAltitude,
        float deltaTime
    );
};

#endif
