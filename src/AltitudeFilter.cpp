#include "AltitudeFilter.hpp"

AltitudeFilter::AltitudeFilter() {
    estimatedAltitude = 0.0f;
    estimatedVelocity = 0.0f;

    positionGain = 0.12f;
    velocityGain = 0.003f;
    accelerationThreshold = 0.15f;

    baroIndex = 0;
    baroWindowFull = false;

    for (int i = 0; i < TAMANO_MEDIANA_BAROMETRO; i++) {
        baroWindow[i] = 0.0f;
    }

    restIndex = 0;

    for (int i = 0; i < TAMANO_VENTANA_REPOSO; i++) {
        restWindow[i] = 0.0f;
    }
}

void AltitudeFilter::begin(
    float positionGainValue,
    float velocityGainValue,
    float accelerationThresholdValue
) {
    positionGain = positionGainValue;
    velocityGain = velocityGainValue;
    accelerationThreshold = accelerationThresholdValue;
}

float AltitudeFilter::calculateBarometerMedian(float newSample) {
    baroWindow[baroIndex] = newSample;

    baroIndex++;

    if (baroIndex >= TAMANO_MEDIANA_BAROMETRO) {
        baroIndex = 0;
        baroWindowFull = true;
    }

    if (baroWindowFull == false) {
        return newSample;
    }

    float sortedSamples[TAMANO_MEDIANA_BAROMETRO];

    for (int i = 0; i < TAMANO_MEDIANA_BAROMETRO; i++) {
        sortedSamples[i] = baroWindow[i];
    }

    for (int i = 0; i < TAMANO_MEDIANA_BAROMETRO - 1; i++) {

        for (
            int j = 0;
            j < TAMANO_MEDIANA_BAROMETRO - 1 - i;
            j++
        ) {
            if (sortedSamples[j] > sortedSamples[j + 1]) {
                float auxiliary = sortedSamples[j];

                sortedSamples[j] = sortedSamples[j + 1];
                sortedSamples[j + 1] = auxiliary;
            }
        }
    }

    int middlePosition = TAMANO_MEDIANA_BAROMETRO / 2;

    return sortedSamples[middlePosition];
}

bool AltitudeFilter::isResting() {
    for (int i = 0; i < TAMANO_VENTANA_REPOSO; i++) {

        if (
            restWindow[i] > accelerationThreshold ||
            restWindow[i] < -accelerationThreshold
        ) {
            return false;
        }
    }

    return true;
}

void AltitudeFilter::estimate(
    float verticalAcceleration,
    float barometerAltitude,
    float deltaTime
) {
    restWindow[restIndex] = verticalAcceleration;

    restIndex++;

    if (restIndex >= TAMANO_VENTANA_REPOSO) {
        restIndex = 0;
    }

    if (deltaTime > 0.2f) {
        deltaTime = 0.2f;
    }

    if (deltaTime <= 0.0f) {
        return;
    }

    float filteredBarometerAltitude;

    filteredBarometerAltitude =
        calculateBarometerMedian(barometerAltitude);

    if (
        verticalAcceleration < 0.05f &&
        verticalAcceleration > -0.05f
    ) {
        verticalAcceleration = 0.0f;
    }

    estimatedAltitude =
        estimatedAltitude +
        estimatedVelocity * deltaTime +
        0.5f *
        verticalAcceleration *
        deltaTime *
        deltaTime;

    estimatedVelocity =
        estimatedVelocity +
        verticalAcceleration * deltaTime;

    float altitudeError;

    altitudeError =
        filteredBarometerAltitude -
        estimatedAltitude;

    estimatedAltitude =
        estimatedAltitude +
        altitudeError * positionGain;

    estimatedVelocity =
        estimatedVelocity +
        altitudeError * velocityGain;

    if (isResting() == true) {
        estimatedVelocity = 0.0f;
    }
}
