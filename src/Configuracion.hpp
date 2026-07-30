#ifndef CONFIGURACION_HPP
#define CONFIGURACION_HPP
// Pines del bus I2C
const int PIN_SDA = 21;
const int PIN_SCL = 26;

// Pines del GPS
const int PIN_GPS_RX = 16;
const int PIN_GPS_TX = 17;

// Pines del módulo LoRa
const int PIN_LORA_NSS = 5;
const int PIN_LORA_DIO0 = 4;
const int PIN_LORA_RESET = 22;
const int PIN_LORA_DIO1 = 3;

// Pines del sistema ACS
const int PIN_ROLL = 25;
const int PIN_PITCH = 27;

// Configuracion de los motores
const int PULSO_MINIMO_ACS = 1000;
const int PULSO_MAXIMO_ACS = 2000;

// Valores usados durante la prueba
const int PULSO_PRUEBA_INICIAL= 1000;
const int PULSO_PRUEBA_FINAL = 1100;
const unsigned long TIEMPO_PRUEBA_MOTOR = 3000;

// Sistema de eyección
const int PIN_MOTOR_EYECCION = 13;

// Lectura de batería
const int PIN_VOLTAJE = 34;

// Comunicación serial
const int VELOCIDAD_SERIAL = 115200;
const int VELOCIDAD_GPS = 9600;

// Intervalos de ejecución
const unsigned long INTERVALO_HEARTBEAT = 1000;
const unsigned long INTERVALO_TELEMETRIA = 100;

// Configuración del filtro de altitud
const int TAMANO_MEDIANA_BAROMETRO = 5;
const int TAMANO_VENTANA_REPOSO = 20;

const unsigned long INTERVALO_FILTRO_ALTITUD = 20;

const float GANANCIA_POSICION_FILTRO = 0.05f;
const float GANANCIA_VELOCIDAD_FILTRO = 0.003f;
const float UMBRAL_REPOSO = 0.15f;

const int CANTIDAD_MUESTRAS_CALIBRACION = 50;
const int RETARDO_CALIBRACION = 10;


// Configuración de los sensores
const int DIRECCION_BME280 = 0x76;
const int DIRECCION_ENS160 = 0x52;

const float PRESION_NIVEL_MAR = 1013.25f;

// Configuración del GPS
const int PUERTO_SERIAL_GPS = 2;

// Configuración del módulo LoRa
const float FRECUENCIA_LORA = 915.0f;
const int FACTOR_PROPAGACION_LORA = 7;
const float ANCHO_BANDA_LORA = 500.0f;
#endif
