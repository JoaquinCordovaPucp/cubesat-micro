#ifndef COMPLEMENTARY_HPP
#define COMPLEMENTARY_HPP

#include <Arduino.h>
// Tamaño de la mediana para el barómetro
// esto es para eliminar picos esporádicos de ±0.5 m del BME280 antes de que entren al filtro. N=5 es suficiente sin añadir lag notable.
#define BARO_MEDIAN_SIZE 5
// Tamaño de la ventana de reposo (ZUPT)
// Si las últimas N aceleraciones netas son menores al umbral
// declaramos reposo y forzamos velocidad = 0.
// Con el filtro corriendo en el loop general (~5 ms/ciclo) → 20
// muestras ≈ 100 ms de ventana.
#define ZUPT_SIZE 20 

class AltitudeFilter {
public:
    // Variables donde guardaremos la altura y velocidad calculada
    float estimatedAltitude;
    float estimatedVelocity;

    // Ganancias del filtro (los "votos de confianza")
    float Kp; // Confianza en la posición
    float Ki; // Confianza para corregir la deriva

    // Umbral de de aceñeración para detectar reposo [m/s²]
    float accelThreshold;

    // Funciones
    AltitudeFilter(); // Constructor (para iniciar variables)
    // Inicializa el filtro con las ganancias y el umbral de aceleración
    // Si la altitud sigue oscilando → subir Kp (ej: 0.20)
    // Si la velocidad deriva mucho  → subir Ki (ej: 0.005)
    void begin(float k_p, float k_i,float accel_threshold=0.15f); 

    // Ejecutar un ciclo del filtro
    // accZ  : aceleración vertical del MPU6050 YA sin gravedad [m/s²]
    // baroAlt : altitud del BME280 YA relativa al punto de lanzamiento [m]
    // dt    : tiempo desde el ciclo anterior [s]
    void estimate(float accZ, float baroAlt, float dt);
private:
    // Ventana de mediana del barómetro
    float baroWindow[BARO_MEDIAN_SIZE]; // historial donde el codigo va a gauardar las ultimas 5 muestras del barómetro 
    uint8_t baroIdx; // Es un apuntador, le dice al codigo en que cajon vacio debe guardar la nueva lectura que acaba de llegar
    bool baroFull; // para saber si ya se lleno los 5 cajones del arreglo
    float baroMedian(float newSample);

    // Ventana ZUPT
    float zuptWindow[ZUPT_SIZE];
    uint8_t zuptIdx;
    bool isResting();
};

#endif
