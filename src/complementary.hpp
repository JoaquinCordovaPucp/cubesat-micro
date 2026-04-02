#ifndef COMPLEMENTARY_HPP
#define COMPLEMENTARY_HPP

#include <Arduino.h>

class AltitudeFilter {
public:
    // Variables donde guardaremos la altura y velocidad calculada
    float estimatedAltitude;
    float estimatedVelocity;

    // Ganancias del filtro (los "votos de confianza")
    float Kp; // Confianza en la posición
    float Ki; // Confianza para corregir la deriva

    // Funciones
    AltitudeFilter(); // Constructor (para iniciar variables)
    void begin(float k_p, float k_i); 
    void estimate(float accZ, float baroAlt, float dt);
};

#endif
