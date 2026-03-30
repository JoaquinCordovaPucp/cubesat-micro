#include "complementary.hpp"
// ─────────────────────────────────────────────────────────────────
// begin() — inicializa el filtro y calcula las ganancias k1 y k2
//
// Las ganancias se derivan matemáticamente de los errores
// de los sensores (ver librería AltitudeEstimation de referencia):
//
//   k1 = sqrt(2 * sigmaAccel / sigmaBaro)
//   k2 = sigmaAccel / sigmaBaro
//
// Si sigmaAccel es pequeño (acelerómetro preciso) →
//   k1 y k2 pequeños → confías más en la física
// Si sigmaBaro es pequeño (barómetro preciso) →
//   k1 y k2 grandes → corriges más agresivamente con el barómetro
// ─────────────────────────────────────────────────────────────────
void ComplementaryFilter::begin(float sigmaAccel, float sigmaBaro,
                                float accelThreshold) {
    // Calcular ganancias
    k1 = sqrt(2.0f * sigmaAccel / sigmaBaro);
    k2 = sigmaAccel / sigmaBaro;
 
    // Guardar umbral ZUPT
    this->accelThreshold = accelThreshold;
 
    // Inicializar estado en cero
    // (altitud relativa al punto de lanzamiento)
    pastAltitude      = 0.0f;
    pastVelocity      = 0.0f;
    estimatedAltitude = 0.0f;
    estimatedVelocity = 0.0f;
 
    // Inicializar ventana ZUPT en cero
    ZUPTIdx = 0;
    for (uint8_t i = 0; i < ZUPT_SIZE; i++) {
        ZUPT[i] = 0.0f;
    }
}
 
// ─────────────────────────────────────────────────────────────────
// estimate() — ejecuta el filtro complementario
//
// Basado en ComplementaryFilter::estimate() de AltitudeEstimation:
// https://github.com/simondlevy/AltitudeEstimation
//
// Ecuación de ALTITUD:
//   h = h_ant + dt * (v_ant + (k1 + k2*dt/2) * (baro - h_ant))
//       + az * dt² / 2
//
//   Desglose:
//   ┌─ h_ant                     → altitud anterior (memoria)
//   ├─ dt * v_ant                → cuánto subí/bajé por velocidad
//   ├─ dt*(k1+k2*dt/2)*(baro-h) → corrección del barómetro
//   └─ az * dt² / 2              → efecto de la aceleración
//
// Ecuación de VELOCIDAD:
//   v = v_ant + dt * (k2 * (baro - h_ant) + az)
//
//   Desglose:
//   ┌─ v_ant                     → velocidad anterior (memoria)
//   ├─ dt * k2 * (baro - h_ant)  → corrección del barómetro
//   └─ dt * az                   → integración de la aceleración
//
// Parámetros:
//   az      : aceleración vertical inercial [m/s²]
//             YA con gravedad descontada (az_raw - 9.81)
//   baroAlt : altitud del BME280 [m]
//             YA relativa al punto de lanzamiento
//   dt      : tiempo desde el ciclo anterior [s]
// ─────────────────────────────────────────────────────────────────
void ComplementaryFilter::estimate(float az, float baroAlt, float dt) {
 
    // Innovación: diferencia entre barómetro y estimación anterior
    // Si es positiva → el barómetro dice que estamos más alto
    // Si es negativa → el barómetro dice que estamos más bajo
    float innovation = baroAlt - pastAltitude;
 
    // ── Ecuación de altitud ───────────────────────────────────────
    estimatedAltitude = pastAltitude
                      + dt * (pastVelocity
                              + (k1 + k2 * dt / 2.0f) * innovation)
                      + az * dt * dt / 2.0f;
 
    // ── Ecuación de velocidad ─────────────────────────────────────
    estimatedVelocity = pastVelocity
                      + dt * (k2 * innovation + az);
 
    // ── ZUPT: Zero Velocity Update ────────────────────────────────
    // Si el CubeSat lleva 12 ciclos con aceleración casi nula
    // (está quieto antes del lanzamiento o después de aterrizar)
    // forzamos velocidad = 0 para evitar que derive
    estimatedVelocity = applyZUPT(az, estimatedVelocity);
 
    // ── Actualizar memoria para el próximo ciclo ──────────────────
    pastAltitude = estimatedAltitude;
    pastVelocity = estimatedVelocity;
}
 
// ─────────────────────────────────────────────────────────────────
// applyZUPT() — Zero Velocity Update
//
// Mantiene una ventana circular de las últimas ZUPT_SIZE (12)
// aceleraciones. Si TODAS son menores que accelThreshold,
// significa que el sistema está en reposo → fuerza velocidad = 0
//
// Esto evita que pequeños errores del acelerómetro acumulen
// velocidad falsa cuando el CubeSat está quieto.
// ─────────────────────────────────────────────────────────────────
float ComplementaryFilter::applyZUPT(float az, float vel) {
 
    // Guardar la aceleración actual en la ventana circular
    ZUPT[ZUPTIdx] = az;
 
    // Avanzar el índice de forma circular (0,1,2,...,11,0,1,...)
    ZUPTIdx = (ZUPTIdx + 1) % ZUPT_SIZE;
 
    // Revisar si TODAS las muestras son menores al umbral
    for (uint8_t k = 0; k < ZUPT_SIZE; k++) {
        if (ZUPT[k] > accelThreshold || ZUPT[k] < -accelThreshold) {
            // Hay movimiento → devolver velocidad sin cambio
            return vel;
        }
    }
 
    // Todas las muestras son pequeñas → reposo detectado
    return 0.0f;
}
 
// ─────────────────────────────────────────────────────────────────
// getAltitude() — devuelve la altitud estimada [m]
// ─────────────────────────────────────────────────────────────────
float ComplementaryFilter::getAltitude() {
    return estimatedAltitude;
}
 
// ─────────────────────────────────────────────────────────────────
// getVelocity() — devuelve la velocidad vertical estimada [m/s]
// Positivo = subiendo, Negativo = bajando
// ─────────────────────────────────────────────────────────────────
float ComplementaryFilter::getVelocity() {
    return estimatedVelocity;
}