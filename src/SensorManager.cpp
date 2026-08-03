#include "SensorManager.hpp"

#include <Wire.h>
#include <ScioSense_ENS16x.h>
#include <math.h>

#include "Configuracion.hpp"

// Este objeto solo sera utilizado dentro de este archivo.
static ENS160 ens160;

SensorManager::SensorManager() {
    ultimoEco2 = 0;
    ultimoTvoc = 0;
    ultimaCalidadAire = 0;

    ultimoIndiceUV = 0.0f;

    offsetAceleracionX = 0.0f;
    offsetAceleracionY = 0.0f;
    offsetAceleracionZ = 0.0f;
}

void SensorManager::iniciar(HardwareSerial *serial) {
    serial->begin(VELOCIDAD_SERIAL);

    Wire.begin(PIN_SDA, PIN_SCL);

    pinMode(PIN_VOLTAJE, INPUT);

    if (bme.begin(DIRECCION_BME280) == false) {
        serial->println(
            "No se pudo encontrar el sensor BME280."
        );

        while (true) {
            delay(10);
        }
    }

    ens160.enableDebugging(Serial);
    ens160.begin(&Wire, DIRECCION_ENS160);

    serial->println("Iniciando ENS160.");

    while (ens160.init() != true) {
        serial->print(".");
        delay(1000);
    }

    if (aht.begin() == false) {
        serial->println(
            "No se pudo encontrar el sensor AHT21."
        );

        while (true) {
            delay(10);
        }
    }

    if (ltr390.begin() == false) {
        serial->println(
            "No se pudo encontrar el sensor LTR390."
        );

        while (true) {
            delay(10);
        }
    }

    iniciarICM20948(serial);

    ltr390.setMode(LTR390_MODE_UVS);
    ltr390.setGain(LTR390_GAIN_3);
    ltr390.setResolution(LTR390_RESOLUTION_16BIT);

    ens160.startStandardMeasure();

    serial->println("Sensores inicializados.");
}

void SensorManager::iniciarICM20948(
    HardwareSerial *serial
) {
    serial->println(
        "Inicializando ICM-20948 en direccion 0x68."
    );

    if (
        icm20948.begin_I2C(
            DIRECCION_ICM20948,
            &Wire
        ) == false
    ) {
        serial->println(
            "No se pudo encontrar el ICM-20948."
        );

        while (true) {
            delay(10);
        }
    }

    icm20948.setAccelRange(
        ICM20948_ACCEL_RANGE_8_G
    );

    icm20948.setGyroRange(
        ICM20948_GYRO_RANGE_500_DPS
    );

    icm20948.enableAccelDLPF(
        true,
        ICM20X_ACCEL_FREQ_23_9_HZ
    );

    icm20948.enableGyrolDLPF(
        true,
        ICM20X_GYRO_FREQ_23_9_HZ
    );

    serial->println(
        "ICM-20948 inicializado correctamente."
    );

    calibrarICM20948(serial);
}

void SensorManager::calibrarICM20948(
    HardwareSerial *serial
) {
    float sumaX;
    float sumaY;
    float sumaZ;

    sumaX = 0.0f;
    sumaY = 0.0f;
    sumaZ = 0.0f;

    serial->println(
        "Calibrando ICM-20948. Mantener el CubeSat quieto."
    );

    delay(1000);

    for (
        int i = 0;
        i < CANTIDAD_MUESTRAS_CALIBRACION_ICM;
        i++
    ) {
        sensors_event_t acelerometro;
        sensors_event_t giroscopio;
        sensors_event_t temperatura;
        sensors_event_t magnetometro;

        icm20948.getEvent(
            &acelerometro,
            &giroscopio,
            &temperatura,
            &magnetometro
        );

        sumaX += acelerometro.acceleration.x;
        sumaY += acelerometro.acceleration.y;

        sumaZ +=
            acelerometro.acceleration.z -
            ACELERACION_GRAVEDAD;

        delay(RETARDO_CALIBRACION_ICM);
    }

    offsetAceleracionX =
        sumaX /
        CANTIDAD_MUESTRAS_CALIBRACION_ICM;

    offsetAceleracionY =
        sumaY /
        CANTIDAD_MUESTRAS_CALIBRACION_ICM;

    offsetAceleracionZ =
        sumaZ /
        CANTIDAD_MUESTRAS_CALIBRACION_ICM;

    serial->println(
        "Calibracion del ICM-20948 completa."
    );
}

void SensorManager::leer(DatosSensores *datos) {
    datos->calidadAireValida = false;
    datos->radiacionUVValida = false;
    datos->movimientoValido = false;

    leerBME(datos);
    leerCalidadAire(datos);
    leerRadiacionUV(datos);
    leerVoltaje(datos);
    leerMovimiento(datos);
}

void SensorManager::leerBME(DatosSensores *datos) {
    float temperaturaCelsius;

    temperaturaCelsius = bme.readTemperature();

    datos->temperaturaKelvin =
        temperaturaCelsius + 273.15f;

    datos->presionPascales =
        bme.readPressure();

    datos->altitudBarometrica =
        obtenerAltitudBarometrica();
}

void SensorManager::leerCalidadAire(
    DatosSensores *datos
) {
    sensors_event_t humedad;
    sensors_event_t temperatura;

    aht.getEvent(&humedad, &temperatura);

    uint16_t temperaturaCompensacion;
    uint16_t humedadCompensacion;

    temperaturaCompensacion =
        Ens16x_CalcTempInFromCelsius(
            temperatura.temperature
        );

    humedadCompensacion =
        Ens16x_CalcRhIn(
            humedad.relative_humidity
        );

    ens160.writeCompensation(
        temperaturaCompensacion,
        humedadCompensacion
    );

    Result resultado;

    resultado = ens160.update();

    if (resultado == RESULT_OK) {
        ultimoEco2 = ens160.getEco2();
        ultimoTvoc = ens160.getTvoc();

        ultimaCalidadAire =
            ens160.getAirQualityIndex_UBA();

        datos->calidadAireValida = true;
    }

    datos->eco2 = ultimoEco2;
    datos->tvoc = ultimoTvoc;
    datos->calidadAire = ultimaCalidadAire;
}

void SensorManager::leerRadiacionUV(
    DatosSensores *datos
) {
    if (ltr390.newDataAvailable() == true) {
        ultimoIndiceUV = calcularIndiceUV();

        datos->radiacionUVValida = true;
    }

    datos->indiceUV = ultimoIndiceUV;
}

void SensorManager::leerVoltaje(
    DatosSensores *datos
) {
    datos->voltajeMilivoltios =
        obtenerVoltajeMilivoltios();
}

void SensorManager::leerMovimiento(
    DatosSensores *datos
) {
    sensors_event_t acelerometro;
    sensors_event_t giroscopio;
    sensors_event_t temperatura;
    sensors_event_t magnetometro;

    bool lecturaCorrecta;

    lecturaCorrecta = icm20948.getEvent(
        &acelerometro,
        &giroscopio,
        &temperatura,
        &magnetometro
    );

    if (lecturaCorrecta == false) {
        datos->giroscopioX = 0.0f;
        datos->giroscopioY = 0.0f;
        datos->giroscopioZ = 0.0f;

        datos->aceleracionX = 0.0f;
        datos->aceleracionY = 0.0f;
        datos->aceleracionZ = 0.0f;

        datos->roll = 0.0f;
        datos->pitch = 0.0f;

        return;
    }

    datos->aceleracionX =
        acelerometro.acceleration.x -
        offsetAceleracionX;

    datos->aceleracionY =
        acelerometro.acceleration.y -
        offsetAceleracionY;

    datos->aceleracionZ =
        acelerometro.acceleration.z -
        offsetAceleracionZ;

    datos->giroscopioX =
        giroscopio.gyro.x;

    datos->giroscopioY =
        giroscopio.gyro.y;

    datos->giroscopioZ =
        giroscopio.gyro.z;

    datos->roll = atan2f(
        datos->aceleracionY,
        sqrtf(
            datos->aceleracionX *
            datos->aceleracionX +
            datos->aceleracionZ *
            datos->aceleracionZ
        )
    );

    datos->pitch = atan2f(
        -datos->aceleracionX,
        sqrtf(
            datos->aceleracionY *
            datos->aceleracionY +
            datos->aceleracionZ *
            datos->aceleracionZ
        )
    );

    datos->movimientoValido = true;
}

float SensorManager::calcularIndiceUV() {
    float ganancia;
    float sensibilidad;
    float valorUV;

    ganancia = 3.0f;

    sensibilidad =
        2300.0f *
        (ganancia / 18.0f) *
        (
            (float)(1UL << 16) /
            (float)(1UL << 20)
        );

    valorUV = ltr390.readUVS();

    return valorUV / sensibilidad;
}

float SensorManager::obtenerAltitudBarometrica() {
    return bme.readAltitude(PRESION_NIVEL_MAR);
}

uint16_t SensorManager::obtenerLecturaVoltajeADC() {
    return (uint16_t)analogRead(PIN_VOLTAJE);
}

float SensorManager::obtenerVoltajeMilivoltios() {
    uint32_t voltajePinMilivoltios;

    voltajePinMilivoltios =
        analogReadMilliVolts(
            PIN_VOLTAJE
        );

    return (float)voltajePinMilivoltios;
}

float SensorManager::obtenerAceleracionVertical() {
    sensors_event_t acelerometro;
    sensors_event_t giroscopio;
    sensors_event_t temperatura;
    sensors_event_t magnetometro;

    bool lecturaCorrecta;

    lecturaCorrecta = icm20948.getEvent(
        &acelerometro,
        &giroscopio,
        &temperatura,
        &magnetometro
    );

    if (lecturaCorrecta == false) {
        return 0.0f;
    }

    return
        acelerometro.acceleration.z -
        offsetAceleracionZ -
        ACELERACION_GRAVEDAD;
}
