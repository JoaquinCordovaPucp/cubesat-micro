#ifndef COMPLEMENTARY_HPP
#define COMPLEMENTARY_HPP

#include <Arduino.h>
#include <math.h>
class AltitudeFilter{
    public:
    // variables donde guardamos la altura y velocidad calculada
    float estimatedAltidude;
    float estimatedVelocity;
    // Ganancias del filtro 
    float kp; // de la posicion
    float ki; // para corregir la derivada

    // funciones
    AltitudeFilter(); // constructor (para iniciar variables)
    void begin(float k_p, float k_i);
    void estimate(float accZ, float baroAlt, float dt);
};
#endif // COMPLEMENTARY_HPP
