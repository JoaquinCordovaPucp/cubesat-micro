#ifndef ESTRUCTURAS_HPP
#define ESTRUCTURAS_HPP

#include <stdint.h>

enum EstadoCubeSat {
    DEBUG = -1,
    ESPERANDO_ACK = 0,
    EN_ESPERA = 1,
    TELEMETRIA_BASICA = 2,
    TELEMETRIA_COMPLETA = 3,
    POST_CAIDA = 4
};

// Códigos binarios para comandos recibidos por radio.
// Usar 1 byte en el aire evita enviar strings completos.
enum ComandoRadio : uint8_t {
    COMANDO_ACK = 0,
    COMANDO_STANDBY = 1,
    COMANDO_TELEMETRIA_BASICA = 2,
    COMANDO_TELEMETRIA_COMPLETA = 3,
    COMANDO_HABILITAR_PARACAIDAS = 4,
    COMANDO_ACTIVAR_CAMARA = 5
};

// Bits de validez del campo FLAGS en TelemetryPacket.
// Se usan para marcar qué grupos de datos del paquete son confiables.
enum TelemetryFlags : uint32_t {
    FLAG_CALIDAD_AIRE_VALIDA = 1UL << 0,
    FLAG_RADIACION_UV_VALIDA = 1UL << 1,
    FLAG_MOVIMIENTO_VALIDO = 1UL << 2,
    FLAG_GPS_UBICACION_VALIDA = 1UL << 3,
    FLAG_GPS_VELOCIDAD_VALIDA = 1UL << 4,
    FLAG_PARACAIDAS_HABILITADO = 1UL << 5,
    FLAG_PARACAIDAS_ARMADO = 1UL << 6,
    FLAG_PARACAIDAS_PRIMERA_ETAPA = 1UL << 7,
    FLAG_PARACAIDAS_SEGUNDA_ETAPA = 1UL << 8,
    FLAG_ATERRIZAJE_DETECTADO = 1UL << 9
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
    bool movimientoValido;
};

struct DatosGPS {
    float latitud;
    float longitud;
    float velocidadHorizontal;

    bool ubicacionValida;
    bool velocidadValida;
};

#pragma pack(push, 1)

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

#pragma pack(pop)

static_assert(
    sizeof(TelemetryPacket) == 56,
    "TelemetryPacket debe medir 56 bytes"
);
#endif
