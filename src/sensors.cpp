#define voltagePin 34 //TODO: Elegir el pin
#include "sensors.hpp"
#include <ScioSense_ENS16x.h>
#include <Wire.h>

// Mantiene la implementacion ENS160 en esta unidad de compilacion para evitar
// definiciones duplicadas provenientes de headers .inl de la libreria.
static ENS160 g_ens160;

void Sensors::init(HardwareSerial* serial) {    //Notese que se recibe un puntero a HardwareSerial

    serial->begin(115200);  // OJO: Como es un puntero, se accede a sus funciones con "->" en vez de "."

    pinMode(voltagePin, INPUT); // Iniciar el pin del voltaje como entrada, para medir el voltaje de la bateria

    //SENSORES INIT
    // BME280
    if (!bme.begin(0x76)) {
        serial->println("No se pudo encontrar el sensor BME280, revisar conexiones!");
        while (1);
    }

    g_ens160.enableDebugging(Serial);

    Wire.begin();
    g_ens160.begin(&Wire, ENS160_I2C_ADDRESS);
    serial->println("begin ens160..");
    while (g_ens160.init() != true) {
        serial->print(".");
        delay(1000);
    }
    //AHT21
    if (!aht.begin()) {
    Serial.println("Failed to find AHT10/AHT20 chip");
    while (1) {
      delay(10);
    }
    }

    // LTR390
    if (!ltr390.begin()) {
        serial->println("No se pudo encontrar el sensor LTR390, revisar conexiones!");
        while (1);
    }
    
    //MPU6050
    if (!mpu.begin()) {
        serial->println("No se pudo encontrar el sensor MPU6050, revisar conexiones!");
        while (1);
    }
    // Calibracion del MPU6050
    serial->println("Calibrando MPU6050...");
    delay(1000); // Esperar un momento para que el sensor se estabilice
    float sumaX=0, sumaY=0,sumaZ=0;
    int num_lecturas = 200;
    for(int i=0; i<num_lecturas; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sumaX += a.acceleration.x;
        sumaY += a.acceleration.y;
        sumaZ += a.acceleration.z -9.81f;
        delay(10); // Pequeña pausa entre lecturas
    }
    // guardar el error promedio en las variables de offset
    offsetX = sumaX / num_lecturas;
    offsetY = sumaY / num_lecturas;
    offsetZ = sumaZ / num_lecturas;
    serial->println("Calibracion MPU6050 completa!");

}

void Sensors::startENS160StandardMeasure() {
    g_ens160.startStandardMeasure();
}

void Sensors::save_bmeDATA(struct TelemetryPacket* data) {      //Data tambien es un puntero, por lo que se accede a sus campos con "->" en vez de "."
    //NOTA: Aca puedo acceder a bme.readTemp(), xq el objeto bme es un atributo de la clase Sensors, por lo que puedo acceder a sus funciones y atributos. Lo mismo para los otros sensores.
    float temp_k  = bme.readTemperature() + 273.15f;        // Kelvin
    // float alt_m_bme280 = bme.readAltitude();   // m 
    uint32_t pres_pa = bme.readPressure();                  // Pa

    //Guardar los datos en el puntero del struct TelemetryPacket    
    data->TEMP = (uint16_t)(temp_k * 100);
    // data->ALT = (uint16_t)(alt_m_bme280 * 10);
    data->PRES = pres_pa;
}

void Sensors::save_ens160DataNATH21(struct TelemetryPacket* data) {
    const uint32_t ECO2_VALID_BIT = (1UL << 9);
    const uint32_t ETOH_VALID_BIT = (1UL << 10);
    const uint32_t AQI_VALID_BIT = (1UL << 11);
    static uint16_t eco2_last = 0;
    static uint16_t etoh_last = 0;
    static uint8_t aqi_last = 0;

    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    (void)humidity;
    (void)temp;

    // ENS16x espera compensacion en formato fijo interno (no float directo).
    uint16_t tempIn = Ens16x_CalcTempInFromCelsius(temp.temperature);
    uint16_t rhIn = Ens16x_CalcRhIn(humidity.relative_humidity);
    g_ens160.writeCompensation(tempIn, rhIn);

    // Refresca el buffer interno; sin update() getEco2/getTvoc leen valores viejos.
    Result updateResult = g_ens160.update();

    // Limpia solo los bits que maneja esta funcion.
    data->FLAGS &= ~(ECO2_VALID_BIT | ETOH_VALID_BIT | AQI_VALID_BIT);

    if (updateResult == RESULT_OK) {
        uint16_t eco2_ppm = g_ens160.getEco2();   // ppm
        uint16_t tvoc_ppb = g_ens160.getTvoc();   // ppb
        uint8_t aqi = (uint8_t)g_ens160.getAirQualityIndex_UBA(); // 1..5

        Serial.print("ECO2: ");
        Serial.print(eco2_ppm);
        Serial.print(" ppm, TVOC: ");
        Serial.print(tvoc_ppb);
        Serial.print(" ppb, AQI: ");
        Serial.println(aqi);

        eco2_last = eco2_ppm;
        etoh_last = tvoc_ppb; // Reuso temporal del campo ETOH para TVOC.
        aqi_last = aqi;

        data->ECO2 = eco2_ppm;
        data->ETOH = tvoc_ppb;
        data->AQI = aqi;
        data->FLAGS |= (ECO2_VALID_BIT | ETOH_VALID_BIT | AQI_VALID_BIT);
        return;
    }

    // Sin dato nuevo o en calentamiento: conserva ultimo valor y no marca valido.
    data->ECO2 = eco2_last;
    data->ETOH = etoh_last;
    data->AQI = aqi_last;
}



void Sensors::save_ltr390DATA(struct TelemetryPacket* data) {
    static uint16_t uvi_last = 0;
    data->FLAGS &= ~(1UL << 12); // Limpia el bit de UV valido (12)
    if(ltr390.newDataAvailable()) {
        uint16_t uvi = (uint16_t)(readUVI() * 100);
        uvi_last = uvi;
        data->UV = uvi;
        data->FLAGS |= (1UL << 12); // Setea el bit de UV valido (12)
        return;
    } else {
        data->UV = uvi_last; // Valor anterior, aunque no es valido. Esto es para evitar que quede en 0, lo cual podria ser confuso.
    }
}

float Sensors::readUVI() {
  const float gain = 3.0f;              
  const float resolutionBits = 16.0f; // 16 bitn Conversion time 25 ms, 20 bit 400ms

  float sensitivity = 2300.0f *
                      (gain / 18.0f) *
                      ((float)(1UL << 16) / (float)(1UL << 20));

  float uvs = (float)ltr390.readUVS();
  return uvs / sensitivity;
}

void Sensors::save_voltage(struct TelemetryPacket* data) {
    int raw_adc = analogRead(voltagePin);
    data->VOLT = (uint16_t) lroundf(raw_adc * 3.3f / 1023.0f * 1000.0f); // Convert to mV
}

void Sensors::save_acsDATA(struct TelemetryPacket* data) {
    struct ACSData acsData;
    getACSData(&acsData);
    data->GYRX = (int16_t) lroundf(acsData.gyrox * 1000.0f); // Convert to rad/s * 1000
    data->GYRY = (int16_t) lroundf(acsData.gyroy * 1000.0f);
    data->GYRZ = (int16_t) lroundf(acsData.gyroz * 1000.0f);
    data->ACCX = (int16_t) lroundf(acsData.acex * 1000.0f); // Convert to m/s^2 * 1000
    data->ACCY = (int16_t) lroundf(acsData.acey * 1000.0f);
    data->ACCZ = (int16_t) lroundf(acsData.acez * 1000.0f);
    data->PITCH = (int16_t) lroundf(acsData.pitch * 1000.0f); // Convert to rad * 1000
    data->ROLL = (int16_t) lroundf(acsData.roll * 1000.0f);
}

void Sensors::saveTime(struct TelemetryPacket* data) {
    data->TIME = millis() / 100; // Convertir a décimas de segundo
}

//Esta funcion biene de la libreria de Adafruit para el LTR390, pero la adapte para que devuelva el indice UV en vez del valor crudo del sensor, y ademas le puse el calculo de sensibilidad segun la ganancia y resolucion que tengo configurada, para que devuelva un valor mas realista del indice UV.
// https://github.com/adafruit/Adafruit_CircuitPython_LTR390/blob/main/adafruit_ltr390.py
//Funcion de adafruit original, para referencia(en python):
// def uvi(self) -> float:
//     """Read UV count and return calculated UV Index (UVI) value based upon the rated sensitivity
//     of 1 UVI per 2300 counts at 18X gain factor and 20-bit resolution."""
//     return (
//         self.uvs
//         / (
//             (Gain.factor[self.gain] / 18)
//             * (2 ** Resolution.factor[self.resolution])
//             / (2**20)
//             * 2300
//         )
//         * self._window_factor
//     )

void Sensors::getACSData(struct ACSData* data) {
    //MPU6050 (giroscopio y acelerometro)
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    data->acex = a.acceleration.x - offsetX; // Aplicar la compensación calculada en init()
    data->acey = a.acceleration.y - offsetY;
    data->acez = a.acceleration.z - offsetZ;
    data->gyrox = g.gyro.x;
    data->gyroy = g.gyro.y;
    data->gyroz = g.gyro.z;
    data->roll = (a.acceleration.y/sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.z*a.acceleration.z)) * (180.0/PI);
    data->pitch = -a.acceleration.x/sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z) * (180.0/PI);
}   

// getBaroAltitude() — lee la altitud del BME280 en metros
//
// Usa la fórmula barométrica estándar con presión de referencia
// 1013.25 hPa (nivel del mar estándar ISA).
//
// IMPORTANTE: esta función devuelve altitud ABSOLUTA sobre el
// nivel del mar. En main.cpp se le restará la altitud inicial
// (medida en setup()) para obtener la altitud RELATIVA al punto
// de lanzamiento, que es lo que necesita el filtro complementario.
//
// Retorna: altitud en metros [m]

float Sensors::getBaroAltitude() {
    return bme.readAltitude(1013.25f);  // metros sobre el nivel del mar
}