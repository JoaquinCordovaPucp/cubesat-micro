
# TODO - Plataforma Telemetría ESP32

## Validación y Flags
- [] Implementar bitmasking para validez de datos en TelemetryPacket (FLAGS).
	- Ejemplo: 11011 → solo el dato 3 (CO2) no es válido.

## Sensores
- [ ] Unificar y validar integración de ENS160 (ScioSense, SparkFun, Adafruit).
	- Usar solo una librería, con begin(&Wire, dirección) y manejo correcto de isNewData/hasNewData.
	- Revisar validez de datos y flags.
- [ ] Mejorar manejo e integración de GPS (TinyGPSPlus).
	- Agregar función para leer y guardar datos GPS en el paquete de telemetría.
	- Revisar robustez y manejo de errores.
- [ ] Agregar y validar librería para datos derivados de IMU y barómetro (altura, complementary filter).

## Lógica de Vuelo y Estados
- [ ] Implementar cambio de estado automático según altura y condiciones (ej: <80m y flag de armado).
- [ ] Mejorar lógica de armado/desarmado para evitar activaciones accidentales.

## Control ACS (Actitud)
- [ ] Clampear/mapping de output pitch y roll del ACS (ej: -1 a 1 → 1500 a 2000 us para servos).
- [ ] Implementar dithering en el control PID del ACS.
- [ ] Implementar anti-windup en el PID del ACS.

## Telemetría y Logs
- [ ] Implementar sistema de logs para guardar datos de vuelo en memoria externa (SD, SPIFFS, etc).

---

**Notas rápidas:**
- El error de compilación por ENS160 ya fue resuelto (ver README).
- Revisar comentarios en src/sensors.hpp y src/sensors.cpp para detalles de integración y flags.