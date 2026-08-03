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

// Sistema de paracaidas
const int PIN_PARACAIDAS = 13;

// Control de la ESP32-CAM
const int PIN_CAMARA = 12;

// Etapas del paracaidas
const float ALTURA_PRIMERA_ETAPA = 80.0f;
const float ALTURA_SEGUNDA_ETAPA = 10.0f;

// Tiempo que GPIO13 permanece en HIGH
const unsigned long DURACION_PRIMERA_ETAPA = 3000;
const unsigned long DURACION_SEGUNDA_ETAPA = 5000;

// Deteccion del descenso
const float VELOCIDAD_MINIMA_DESCENSO = -0.5f;

// Confirmacion de que el vuelo comenzo
const float ALTURA_MINIMA_INICIO_VUELO = 20.0f;

// Deteccion de aterrizaje
// valores iniciales para las pruebas
const float ALTURA_CERCA_DEL_PISO = 3.0f;
const float VELOCIDAD_MAXIMA_REPOSO = 0.5f;

const unsigned long TIEMPO_CONFIRMACION_ATERRIZAJE = 40000;

// Envio durante la post-caida
const unsigned long INTERVALO_POST_CAIDA = 1000;

// Lectura de batería
const int PIN_VOLTAJE = 36;

// Comunicación serial
const int VELOCIDAD_SERIAL = 115200;
const int VELOCIDAD_GPS = 9600;

// Intervalos de ejecución
const unsigned long INTERVALO_HEARTBEAT = 1000;
const unsigned long INTERVALO_TELEMETRIA = 100;

// Configuracion del filtro de altitud
const unsigned long INTERVALO_FILTRO_ALTITUD = 20;

const float VARIANZA_INICIAL_ALTITUD = 0.5f;
const float VARIANZA_INICIAL_VELOCIDAD = 0.5f;

const float RUIDO_PROCESO_ALTITUD = 0.01f;
const float RUIDO_PROCESO_VELOCIDAD = 0.10f;

const float RUIDO_MEDICION_BAROMETRO = 0.25f;

// Calibracion inicial del barometro
const int CANTIDAD_MUESTRAS_CALIBRACION = 50;
const int RETARDO_CALIBRACION = 10;


// Configuración de los sensores
const int DIRECCION_BME280 = 0x76;
const int DIRECCION_ENS160 = 0x52;

// Configuracion del ICM-20948
const int DIRECCION_ICM20948 = 0x68;
const int CANTIDAD_MUESTRAS_CALIBRACION_ICM = 200;
const int RETARDO_CALIBRACION_ICM = 10;
const float ACELERACION_GRAVEDAD = 9.80665f;

const float PRESION_NIVEL_MAR = 1013.25f;

// Configuración del GPS
const int PUERTO_SERIAL_GPS = 2;

// Configuración del módulo LoRa
const float FRECUENCIA_LORA = 915.0f;
const int FACTOR_PROPAGACION_LORA = 7;
const float ANCHO_BANDA_LORA = 500.0f;

// validar fisicamente el funcionamiento
// de CS_sD conectado a pin 34
// Configuracion de la MicroSD
const int PIN_SD_SCK = 18;
const int PIN_SD_MISO = 19;
const int PIN_SD_MOSI = 23;
const int PIN_SD_CS = 34;

// Tiempo entre escrituras completas en la MicroSD
const unsigned long INTERVALO_FLUSH_SD = 1000;

#endif
