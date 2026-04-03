#include "complementary.hpp"

// Al empezar, ponemos todo en cero
AltitudeFilter::AltitudeFilter() {
    estimatedAltitude = 0.0f;
    estimatedVelocity = 0.0f;
    Kp = 0.12f; // Ganancia para la corrección de posición
    Ki = 0.003f; // Ganancia para la corrección de velocidad (deriva)
    accelThreshold = 0.15f; // Umbral para detectar reposo

    baroIdx = 0;
    baroFull = false;
    for (uint8_t i = 0; i < BARO_MEDIAN_SIZE; i++) {
        baroWindow[i] = 0.0f;
    }
    zuptIdx = 0;
    for (uint8_t i = 0; i < ZUPT_SIZE; i++) {
        zuptWindow[i] = 0.0f;
    }
}

// Aquí configuramos las ganancias que calculamos antes y el umbral ZUPT
void AltitudeFilter::begin(float k_p, float k_i,float accel_threshold) {
    Kp = k_p;
    Ki = k_i;
    accelThreshold = accel_threshold;
}

// Esta es la función que hace el cálculo pesado
// baroMedian() — mediana de 5 muestras sobre el barómetro
//
// El BME280 tiene picos esporádicos de ±0.5 m que se propagan
// directamente al estimador como saltos de altitud/velocidad.
// La mediana los elimina por completo sin el lag de un promedio.
float AltitudeFilter::baroMedian(float newSample) {
    baroWindow[baroIdx] = newSample; // cuando llega el dato, lo guardamos en el cajon
    baroIdx = (baroIdx + 1) % BARO_MEDIAN_SIZE; // para que el indice vuelve a empezar y borra el dato mas viejo para ponerle el nuevo
    if (baroIdx == 0) baroFull = true;
    // Mientras la ventana no está llena, devolver muestra directa
    if (!baroFull) return newSample;
    // Copiar y ordenar (burbuja — rápido para N=5)
    float s[BARO_MEDIAN_SIZE];
    for (uint8_t i = 0; i < BARO_MEDIAN_SIZE; i++) s[i] = baroWindow[i]; // copiar el arreglo para no modificar el original
    // arreglar con el algoritmo de burbuja
    for (uint8_t i = 0; i < BARO_MEDIAN_SIZE - 1; i++) {
        for (uint8_t j = 0; j < BARO_MEDIAN_SIZE - 1 - i; j++) {
            if (s[j] > s[j+1]) { float t = s[j]; s[j] = s[j+1]; s[j+1] = t; }
        }
    }
    return s[BARO_MEDIAN_SIZE / 2]; // Devolver la mediana

}

// Función para detectar reposo
// Si TODAS las aceleraciones nestas de la ventana estan por debajo de accelThreshold 
// el cubesat esta quieto, forzar vel = 0.
// Esto evita que la integracion acumule velocidad falsa en reposo. 
bool AltitudeFilter::isResting() {
    for(uint8_t i = 0; i < ZUPT_SIZE; i++) {
        if (zuptWindow[i] > accelThreshold || zuptWindow[i] < -accelThreshold)
            return false; // Si alguna muestra supera el umbral, no estamos en reposo
    }
    return true; // Si todas las muestras están dentro del umbral, estamos en reposo
}
// estimate() — predictor-corrector mejorado
//
// Arquitectura original preservada (Kp/Ki):
//   Predicción:  h += v*dt + 0.5*a*dt²
//                v += a*dt
//   Corrección:  error = baro_filtrado - h
//                h += error * Kp
//                v += error * Ki
//
// Mejoras aplicadas:
//   1. Mediana de 5 muestras sobre baroAlt  → elimina picos ±0.5 m
//   2. ZUPT de 20 muestras con umbral 0.15  → velocidad = 0 en reposo
//   3. Umbral de vibración sube a 0.15 m/s² → menos cortes falsos
//      durante movimiento suave
//   4. Validación de dt                     → evita saltos tras gaps
//
// Parámetros:
//   accZ    : aceleración vertical YA sin gravedad [m/s²]
//   baroAlt : altitud BME280 YA relativa al lanzamiento [m]
//   dt      : segundos desde el ciclo anterior
void AltitudeFilter::estimate(float accZ, float baroAlt, float dt) {
    // Guardar en ventana ZUPT antes de cualquier recorte 
    zuptWindow[zuptIdx] = accZ;
    zuptIdx = (zuptIdx + 1) % ZUPT_SIZE;
    // Valirdar dt -si hay un gap grande, limitar para no integrar mal
    if (dt > 0.1f) dt = 0.1f; // Limitar a 100 ms para evitar saltos grandes
    if(dt <= 0.0f) return;

    float baroFiltered = baroMedian(baroAlt); // Filtrar la altitud del barómetro con la mediana
    // ZUPT: umbral para vibración/ruido del acelerómetro
    // Original: < 0.3  → muy agresivo, cortaba movimientos reales lentos
    // Ajustado: ZUPT detecta reposo por ventana; aquí solo suprimimos ruido puro
    if(fabs(accZ)<0.05f){
        accZ=0.0f;
    }
    // 1 Prediccion
    // Calculamos dónde "creemos" que estamos basándonos solo en la aceleración
    // d = v*t + 0.5*a*t^2
    estimatedAltitude = estimatedAltitude + (estimatedVelocity * dt) + (0.5f * accZ * dt * dt);
    estimatedVelocity = estimatedVelocity + (accZ * dt);

    // Calculo del error
    // Comparamos nuestra predicción con lo que dice el Barómetro (la realidad)
   // ── Corrección con el barómetro filtrado ───────────────────────
    float error = baroFiltered - estimatedAltitude;
    estimatedAltitude += error * Kp;
    estimatedVelocity += error * Ki;
    
    // ── ZUPT: si llevamos 20 ciclos quietos, forzar velocidad = 0 ──
    if (isResting()) {
        estimatedVelocity = 0.0f;
    }
}