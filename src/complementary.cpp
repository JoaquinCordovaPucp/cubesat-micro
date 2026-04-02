#include "complementary.hpp"
// Al empezar, ponemos todo en cero
AltitudeFilter::AltitudeFilter(){
    estimatedAltidude =0.0f;
    estimatedVelocity= 0.0f;
}
// configuracion las ganancias que calculamos antes
void AltitudeFilter::begin(float k_p,float k_i){
    kp = k_p;
    ki = k_i;
}
// aqui vienen los calculos
void AltitudeFilter::estimate(float accZ, float baroAlt, float dt){
    // 1: La predicción
    estimatedAltidude = estimatedAltidude + (estimatedVelocity*dt)+ (0.5*accZ*dt*dt);
    estimatedVelocity =estimatedVelocity + (accZ*dt);
    // 2: calculo del error
    float error = baroAlt - estimatedAltidude;
    // COrrecion
    estimatedAltidude = estimatedAltidude +(error*kp);
    estimatedVelocity = estimatedVelocity + (error*ki);
}
