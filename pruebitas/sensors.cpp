#include "sensors.hpp"


void Sensors::init(HardwareSerial* serial) {    //Notese que se recibe un puntero a HardwareSerial

    serial->begin(115200);  // OJO: Como es un puntero, se accede a sus funciones con "->" en vez de "."

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
    // Aquí puedes implementar la lógica para guardar los datos del BME280
    float temp_k  = bme.readTemperature() + 273.15f;        // Kelvin
    float alt_m_bme280 = bme.readAltitude(1013.25f);        // m 
    uint32_t pres_pa = bme.readPressure();                  // Pa

    //Guardar los datos en el puntero del struct TelemetryPacket    
    data->TEMP = (uint16_t)(temp_k * 100);
    data->ALT = (uint16_t)(alt_m_bme280 * 10);
    data->PRES = pres_pa;
}