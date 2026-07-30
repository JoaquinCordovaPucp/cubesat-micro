#ifndef ESTRUCTURAS_HPP
#define ESTRUCTURAS_HPP

#include <stdint.h>

enum EstadoCubeSat {
    DEBUG = -1,
    ESPERANDO_ACK = 0,
    EN_ESPERA = 1,
    TELEMETRIA_BASICA = 2,
    TELEMETRIA_COMPLETA = 3
};

struct DatosSensores {
    float temperaturaKelvin;
    float presionPascales;
    float altitudBarometrica;

    int eco2;
    int tvoc;
    int calidadAire;

    float indiceUV;
    float voltajeMilivoltios;

    float giroscopioX;
    float giroscopioY;
    float giroscopioZ;

    float aceleracionX;
    float aceleracionY;
    float aceleracionZ;

    float roll;
    float pitch;

    bool calidadAireValida;
    bool radiacionUVValida;
};

struct DatosGPS {
    float latitud;
    float longitud;
    float velocidadHorizontal;

    bool ubicacionValida;
    bool velocidadValida;
};

struct TelemetryPacket {
    uint8_t TYPE;
    uint16_t SEQ;
    uint32_t TIME;
    uint32_t FLAGS;

    uint16_t VOLT;

    int16_t PITCH;
    int16_t ROLL;

    int32_t LON;
    int32_t LAT;

    int16_t VVEL;

    uint32_t PRES;
    uint16_t TEMP;

    uint16_t ECO2;
    uint16_t ETOH;
    uint8_t AQI;

    uint16_t UV;

    int16_t GYRX;
    int16_t GYRY;
    int16_t GYRZ;

    int16_t ACCX;
    int16_t ACCY;
    int16_t ACCZ;

    int16_t ALT;

    uint16_t CHK;
};
#endif
