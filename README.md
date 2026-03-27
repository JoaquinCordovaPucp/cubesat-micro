# Test01

Proyecto de telemetria para `ESP32` con `PlatformIO` y `Arduino`, orientado a envio por `LoRa` y lectura de sensores.

## Importante

- La carpeta real del firmware es `src/`.
- `pruebitas/` es solo una carpeta de pruebas y testeo basico.

## Estructura Del Proyecto

- `src/main.cpp`: logica principal, maquina de estados, LoRa TX/RX y armado de paquetes.
- `src/sensors.hpp`: definicion de `TelemetryPacket`, `ACSData` y clase `Sensors`.
- `src/sensors.cpp`: inicializacion y lectura de sensores.
- `pruebitas/`: ejemplos y pruebas aisladas, no es la base del firmware de vuelo.
- `platformio.ini`: entorno `esp32dev`, dependencias y configuracion de compilacion/monitor.

## Hardware Y Sensores

- MCU: ESP32 (`board = esp32dev`)
- LoRa: SX1276 (RadioLib)
- Sensores:
- BME280
- ENS160
- AHTX0
- LTR390
- MPU6050
- GPS (TinyGPSPlus por `HardwareSerial`)

## Pines Relevantes (segun codigo actual)

- I2C: `SDA=21`, `SCL=26` (`Wire.begin(21, 26)`)
- GPS UART2: `RX=16`, `TX=17` (`GPSserial.begin(..., 16, 17)`)
- LoRa Module: `new Module(5, 4, 22, 3)`
- Voltaje bateria: `GPIO34` (`voltagePin`)

## Maquina De Estados (`generalState`)

- `0`: heartbeat inicial y espera de `ack`
- `1`: standby (paquete tipo 1 cada 1s)
- `2`: datos basicos (paquete tipo 2 cada 100ms)
- `3`: datos completos (paquete tipo 3 cada 100ms)
- `-1`: debug (bloqueante)

## Flujo General

1. Inicializa I2C, serial, sensores y LoRa en `setup()`.
2. Mantiene radio en recepcion por defecto.
3. Cuando termina TX/RX (`setFlag`), vuelve a `startReceive()`.
4. En cada estado arma y transmite un `TelemetryPacket` binario.

## Dependencias Principales

Definidas en `platformio.ini`:

- `jgromes/RadioLib`
- `adafruit/Adafruit Unified Sensor`
- `adafruit/Adafruit BME280 Library`
- `adafruit/ENS160 - Adafruit Fork`
- `adafruit/Adafruit AHTX0`
- `sparkfun/SparkFun Indoor Air Quality Sensor - ENS160`
- `adafruit/Adafruit LTR390 Library`
- `adafruit/Adafruit MPU6050`
- `mikalhart/TinyGPSPlus`

## Compilar, Subir Y Monitor

Desde la raiz del proyecto:

```bash
pio run
pio run -t upload --upload-port /dev/cu.usbserial-0001
pio device monitor -b 115200
```

## Perfilado Basico (Modo 3)

Muestra de log capturada:

```text
[M3 us] BME=3384 ENS=531 LTR=1110 GPS=22 MPU=2588 PACK=23 TXcall=1344 TOTAL=9010
```

Promedios medidos (`n=8`, en microsegundos):

- `BME`: 3382.75
- `MPU`: 2587.88
- `TXcall`: 1345.38
- `LTR`: 1111.00
- `ENS`: 532.13
- `PACK`: 23.00
- `GPS`: 21.75
- `TOTAL`: 9009.13

Nota: `TXcall` mide solo el costo de llamar `startTransmit()` (no bloqueante), no el tiempo en aire de la trama LoRa.

Test rapido de latencia de telemetria (medido desde `startTransmit` hasta `operationDone`):

- Promedio observado: `35.240 ms`

## Estado Actual / TODO Rapido

- Ajustar conversion real de `VOLT` desde ADC.
- Revisar variables ACS (roll/pitch y conversiones rad/grados) para consistencia.
- Afinar PID y validarlo con `dt` estable.
- Completar `CHK` (CRC-16) del `TelemetryPacket`.
