#include "complementary.hpp"

// Al empezar, ponemos todo en cero
AltitudeFilter::AltitudeFilter() {
    estimatedAltitude = 0.0f;
    estimatedVelocity = 0.0f;
}

// Aquí configuramos las ganancias que calculamos antes
void AltitudeFilter::begin(float k_p, float k_i) {
    Kp = k_p;
    Ki = k_i;
}

// Esta es la función que hace el cálculo pesado
void AltitudeFilter::estimate(float accZ, float baroAlt, float dt) {
    
    // 1 Prediccion
    // Calculamos dónde "creemos" que estamos basándonos solo en la aceleración
    // d = v*t + 0.5*a*t^2
    estimatedAltitude = estimatedAltitude + (estimatedVelocity * dt) + (0.5f * accZ * dt * dt);
    estimatedVelocity = estimatedVelocity + (accZ * dt);

    // Calculo del error
    // Comparamos nuestra predicción con lo que dice el Barómetro (la realidad)
    float error = baroAlt - estimatedAltitude;

    // Las correciones
    // Usamos el error para corregir los datos y que no se pierdan en el tiempo
    estimatedAltitude = estimatedAltitude + (error * Kp);
    estimatedVelocity = estimatedVelocity + (error * Ki); // Esta línea quita la "deriva"
}