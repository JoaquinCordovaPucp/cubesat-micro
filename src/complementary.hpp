#ifndef COMPLEMENTARY_HPP
#define COMPLEMENTARY_HPP

#include <Arduino.h>
#include <math.h>
#define ZUPT_SIZE 12
class ComplementaryFilter{
    public:
 
    // ── Inicializar el filtro ─────────────────────────────
    // sigmaAccel     : desviación estándar del acelerómetro [m/s²]
    //                  MPU6050 típico: 0.0005
    // sigmaBaro      : desviación estándar del barómetro [m]
    //                  BME280 típico: 0.018
    // accelThreshold : umbral para detectar reposo [m/s²]
    //                  Si az < threshold durante 12 ciclos → vel = 0
    void begin(float sigmaAccel, float sigmaBaro, float accelThreshold);
 
    // ── Ejecutar el filtro (llamar en cada ciclo) ─────────
    // az       : aceleración vertical inercial [m/s²]
    //            (ya con gravedad descontada)
    // baroAlt  : altitud del BME280 [m]
    //            (ya relativa al punto de lanzamiento)
    // dt       : tiempo desde el ciclo anterior [s]
    void estimate(float az, float baroAlt, float dt);
 
    // ── Leer resultados ───────────────────────────────────
    float getAltitude();   // altitud estimada [m]
    float getVelocity();   // velocidad vertical estimada [m/s]
 
private:
 
    // Ganancias del filtro (se calculan en begin())
    float k1;
    float k2;
 
    // Umbral para el ZUPT
    float accelThreshold;
 
    // Estado del filtro (se actualiza en cada estimate())
    float pastAltitude;
    float pastVelocity;
 
    // Resultados del último ciclo
    float estimatedAltitude;
    float estimatedVelocity;
 
    // Ventana ZUPT — guarda las últimas ZUPT_SIZE aceleraciones
    float ZUPT[ZUPT_SIZE];
    uint8_t ZUPTIdx;
 
    // Función interna: aplica ZUPT a la velocidad
    // Si hay reposo → devuelve 0, si no → devuelve vel sin cambio
    float applyZUPT(float az, float vel);
};
#endif // COMPLEMENTARY_HPP
