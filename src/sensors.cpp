#define voltagePin 34 //TODO: Elegir el pin
#include "sensors.hpp"

void Sensors::init(HardwareSerial* serial) {    //Notese que se recibe un puntero a HardwareSerial

    serial->begin(115200);  // OJO: Como es un puntero, se accede a sus funciones con "->" en vez de "."

    pinMode(voltagePin, INPUT); // Iniciar el pin del voltaje como entrada, para medir el voltaje de la bateria

    //SENSORES INIT
    // BME280
    if (!bme.begin(0x76)) {
        serial->println("No se pudo encontrar el sensor BME280, revisar conexiones!");
        while (1);
    }

    // ENS160
    if (ens160.begin() != 0) {
        serial->println("No se pudo encontrar el sensor ENS160, revisar conexiones!");
        while (1);
    }

    // AHTX0
    if (!aht.begin()) {
        serial->println("No se pudo encontrar el sensor AHTX0, revisar conexiones!");
        while (1);
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
}

void Sensors::save_bmeDATA(struct TelemetryPacket* data) {      //Data tambien es un puntero, por lo que se accede a sus campos con "->" en vez de "."
    //NOTA: Aca puedo acceder a bme.readTemp(), xq el objeto bme es un atributo de la clase Sensors, por lo que puedo acceder a sus funciones y atributos. Lo mismo para los otros sensores.
    float temp_k  = bme.readTemperature() + 273.15f;        // Kelvin
    float alt_m_bme280 = bme.readAltitude(1013.25f);        // m 
    uint32_t pres_pa = bme.readPressure();                  // Pa

    //Guardar los datos en el puntero del struct TelemetryPacket    
    data->TEMP = (uint16_t)(temp_k * 100);
    data->ALT = (uint16_t)(alt_m_bme280 * 10);
    data->PRES = pres_pa;
}

void Sensors::save_ens160DATA(struct TelemetryPacket* data) {
    const uint32_t ECO2_VALID_BIT = (1UL << 9);
    const uint32_t ETOH_VALID_BIT = (1UL << 10);
    const uint32_t AQI_VALID_BIT = (1UL << 11);
    static uint16_t eco2_last = 0;
    static uint16_t etoh_last = 0;
    static uint8_t aqi_last = 0;

    // Limpia solo los bits que maneja esta funcion.
    data->FLAGS &= ~(ECO2_VALID_BIT | ETOH_VALID_BIT | AQI_VALID_BIT);

    if (ens160.checkDataStatus() && (ens160.getFlags() == 0)) { // Flag 0 = Standard, por lo que el sensor esta listo para medidas utiles. Si no es 0, esta en warmup o startup, y las medidas no son confiables.
        uint16_t eco2_ppm = ens160.getECO2();   // ppm
        uint16_t etoh_ppm = ens160.getETOH();   // ppm
        uint8_t aqi = ens160.getAQI(); // 1 - Excellent, 2 - Good, 3 - Moderate, 4 - Poor, 5 - Unhealthy.
        eco2_last = eco2_ppm;
        etoh_last = etoh_ppm;
        aqi_last = aqi;
        data->ETOH = etoh_ppm;
        data->AQI = aqi;
        data->ECO2 = eco2_ppm;
        data->FLAGS |= (ECO2_VALID_BIT | ETOH_VALID_BIT | AQI_VALID_BIT); // Setea los bits de validos
        return;
    } else {
        data->ECO2 = eco2_last; // Valor anterior, aunque no es valido. Esto es para evitar que quede en 0, lo cual podria ser confuso.
        data->ETOH = etoh_last;
        data->AQI = aqi_last;
    }
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
    data->acex = a.acceleration.x;
    data->acey = a.acceleration.y;
    data->acez = a.acceleration.z;
    data->gyrox = g.gyro.x;
    data->gyroy = g.gyro.y;
    data->gyroz = g.gyro.z;
    data->roll = (a.acceleration.y/sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.z*a.acceleration.z)) * (180.0/PI);
    data->pitch = -a.acceleration.x/sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z) * (180.0/PI);
}   