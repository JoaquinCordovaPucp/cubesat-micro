/*
  ESP32-CAM AI-Thinker
  - Trigger HIGH: toma imágenes, guarda en microSD y habilita /stream y /frame.jpg.
  - Trigger LOW: deja de capturar, guardar y transmitir; apaga el Wi-Fi.
  - Resolución QQVGA (160x120) y 2 FPS.
  - No borra archivos. Cada activación crea una carpeta nueva.

  IMPORTANTE:
  - El pin recomendado es GPIO3 (U0R/RX0), porque GPIO13 pertenece a la microSD.
  - Después de cargar el programa, desconecta el cable TX del adaptador USB-TTL
    que llega al pin U0R/GPIO3.
  - La señal de entrada debe ser de 3.3 V. Nunca conectes 5 V al GPIO.
  - Alimenta la ESP32-CAM por 5 V con una fuente estable de al menos 1 A.
*/

#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "FS.h"
#include "SD_MMC.h"
#include <Preferences.h>

// ============================================================
// WIFI
// ============================================================

const char *WIFI_SSID = "iPhoneGG";
const char *WIFI_PASSWORD = "553618253@";

// IP fija de la ESP32-CAM.
// Debe pertenecer a la misma red que el router y no estar ocupada.
IPAddress IP_LOCAL(192, 168, 1, 80);
IPAddress PUERTA_ENLACE(192, 168, 1, 1);
IPAddress MASCARA_RED(255, 255, 255, 0);
IPAddress DNS_PRIMARIO(8, 8, 8, 8);
IPAddress DNS_SECUNDARIO(1, 1, 1, 1);

const char *NOMBRE_HOST = "esp32cam-cubesat";

// Endpoints directos para usar desde otro HTML.
const char *ENDPOINT_STREAM = "/stream";
const char *ENDPOINT_FRAME = "/frame.jpg";

const unsigned long TIEMPO_MAXIMO_CONEXION_WIFI_MS = 20000;
const unsigned long INTERVALO_REINTENTO_WIFI_MS = 10000;

// ============================================================
// DISPARO
// ============================================================

// GPIO3 = U0R/RX0.
// Desconecta el TX del programador USB-TTL después de cargar el programa.
const int PIN_ACTIVACION = 3;

// La entrada debe estar estable este tiempo para aceptar el cambio.
const unsigned long TIEMPO_ANTIRREBOTE_MS = 50;

// ============================================================
// CAPTURA
// ============================================================

// 500 ms = 2 fotogramas por segundo.
const unsigned long INTERVALO_CAPTURA_MS = 500;

// Tamaño reservado para la última imagen mostrada por la página.
const size_t MAXIMO_TAMANO_FRAME = 60 * 1024;

// ============================================================
// PINES DE CÁMARA: AI-THINKER ESP32-CAM
// ============================================================

#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_Y9      35
#define CAM_PIN_Y8      34
#define CAM_PIN_Y7      39
#define CAM_PIN_Y6      36
#define CAM_PIN_Y5      21
#define CAM_PIN_Y4      19
#define CAM_PIN_Y3      18
#define CAM_PIN_Y2       5

#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// ============================================================
// OBJETOS Y ESTADO
// ============================================================

WebServer servidor(80);
Preferences preferencias;

bool camaraLista = false;
bool sdLista = false;
bool servidorActivo = false;
bool grabacionActiva = false;

bool lecturaBrutaAnterior = false;
bool estadoPinConfirmado = false;

unsigned long inicioCambioPinMs = 0;
unsigned long ultimoFrameMs = 0;
unsigned long ultimoIntentoWifiMs = 0;

uint32_t numeroSesion = 0;
uint32_t numeroFrame = 0;

char carpetaSesion[40] = {0};

uint8_t *ultimoFrame = nullptr;
size_t ultimoFrameLongitud = 0;

// ============================================================
// PÁGINA WEB
// ============================================================

const char PAGINA_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32-CAM CubeSat</title>
</head>
<body>
  <h2>ESP32-CAM CubeSat</h2>
  <p>Stream MJPEG de baja resolución.</p>
  <img src="/stream" alt="Camara ESP32-CAM">
</body>
</html>
)rawliteral";

// ============================================================
// CÁMARA
// ============================================================

bool iniciarCamara() {
    camera_config_t config = {};

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;

    config.pin_d0 = CAM_PIN_Y2;
    config.pin_d1 = CAM_PIN_Y3;
    config.pin_d2 = CAM_PIN_Y4;
    config.pin_d3 = CAM_PIN_Y5;
    config.pin_d4 = CAM_PIN_Y6;
    config.pin_d5 = CAM_PIN_Y7;
    config.pin_d6 = CAM_PIN_Y8;
    config.pin_d7 = CAM_PIN_Y9;

    config.pin_xclk = CAM_PIN_XCLK;
    config.pin_pclk = CAM_PIN_PCLK;
    config.pin_vsync = CAM_PIN_VSYNC;
    config.pin_href = CAM_PIN_HREF;

    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;

    config.pin_pwdn = CAM_PIN_PWDN;
    config.pin_reset = CAM_PIN_RESET;

    config.xclk_freq_hz = 20000000;

    // JPEG evita conversiones pesadas.
    config.pixel_format = PIXFORMAT_JPEG;

    // Resolución muy baja: 160 x 120.
    config.frame_size = FRAMESIZE_QQVGA;

    // En JPEG, un número mayor significa menor calidad y menor tamaño.
    config.jpeg_quality = 30;

    if (psramFound()) {
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
    }
    else {
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.fb_count = 1;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    esp_err_t resultado = esp_camera_init(&config);

    if (resultado != ESP_OK) {
        Serial.printf(
            "ERROR: no se pudo iniciar la camara. Codigo 0x%X\n",
            resultado
        );

        return false;
    }

    sensor_t *sensor = esp_camera_sensor_get();

    if (sensor != nullptr) {
        sensor->set_framesize(
            sensor,
            FRAMESIZE_QQVGA
        );

        sensor->set_quality(
            sensor,
            30
        );
    }

    Serial.println("Camara iniciada.");
    return true;
}

// ============================================================
// MICROSD
// ============================================================

bool iniciarMicroSD() {
    /*
      true = modo SD_MMC de 1 bit.

      Esto usa:
      GPIO14 = CLK
      GPIO15 = CMD
      GPIO2  = D0

      No formatea la tarjeta y no borra archivos.
    */
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println(
            "ERROR: no se pudo montar la microSD."
        );

        return false;
    }

    if (SD_MMC.cardType() == CARD_NONE) {
        Serial.println(
            "ERROR: no se detecto una microSD."
        );

        return false;
    }

    Serial.print("MicroSD iniciada. Capacidad MB: ");
    Serial.println(
        SD_MMC.cardSize() /
        (1024ULL * 1024ULL)
    );

    return true;
}

// ============================================================
// CREAR UNA CARPETA NUEVA PARA CADA ACTIVACIÓN
// ============================================================

bool crearNuevaSesion() {
    preferencias.begin(
        "camara",
        false
    );

    numeroSesion =
        preferencias.getUInt(
            "sesion",
            0
        ) + 1;

    // Evita reutilizar una carpeta aunque la NVS haya quedado atrasada.
    while (true) {
        snprintf(
            carpetaSesion,
            sizeof(carpetaSesion),
            "/video_%06lu",
            static_cast<unsigned long>(
                numeroSesion
            )
        );

        if (!SD_MMC.exists(carpetaSesion)) {
            break;
        }

        numeroSesion++;
    }

    bool creada =
        SD_MMC.mkdir(carpetaSesion);

    if (creada) {
        preferencias.putUInt(
            "sesion",
            numeroSesion
        );
    }

    preferencias.end();

    if (!creada) {
        Serial.print(
            "ERROR: no se pudo crear "
        );

        Serial.println(
            carpetaSesion
        );

        return false;
    }

    numeroFrame = 0;

    Serial.print(
        "Nueva grabacion: "
    );

    Serial.println(
        carpetaSesion
    );

    return true;
}

// ============================================================
// WIFI
// ============================================================

bool conectarWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    Serial.print(
        "Conectando a WiFi: "
    );

    Serial.println(
        WIFI_SSID
    );

    WiFi.mode(WIFI_STA);

    // Mantiene el Wi-Fi activo para reducir cortes en el stream.
    WiFi.setSleep(false);

    // Configura la IP fija antes de iniciar la conexión.
    bool ipConfigurada =
        WiFi.config(
            IP_LOCAL,
            PUERTA_ENLACE,
            MASCARA_RED,
            DNS_PRIMARIO,
            DNS_SECUNDARIO
        );

    if (!ipConfigurada) {
        Serial.println(
            "ERROR: no se pudo configurar la IP fija."
        );

        WiFi.mode(WIFI_OFF);
        return false;
    }

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );

    // Potencia máxima disponible en el core Arduino ESP32.
    WiFi.setTxPower(
        WIFI_POWER_19_5dBm
    );

    unsigned long inicio =
        millis();

    while (
        WiFi.status() != WL_CONNECTED &&
        millis() - inicio <
        TIEMPO_MAXIMO_CONEXION_WIFI_MS
    ) {
        delay(250);
        Serial.print(".");
    }

    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(
            "No se pudo conectar al WiFi."
        );

        WiFi.disconnect(
            true,
            false
        );

        WiFi.mode(WIFI_OFF);

        return false;
    }

    WiFi.setAutoReconnect(true);

    Serial.println(
        "WiFi conectado."
    );

    Serial.print(
        "IP fija: "
    );

    Serial.println(
        WiFi.localIP()
    );

    Serial.print(
        "RSSI: "
    );

    Serial.print(
        WiFi.RSSI()
    );

    Serial.println(
        " dBm"
    );

    Serial.print(
        "Endpoint de video: http://"
    );

    Serial.print(
        WiFi.localIP()
    );

    Serial.println(
        ENDPOINT_STREAM
    );

    Serial.print(
        "Endpoint de imagen: http://"
    );

    Serial.print(
        WiFi.localIP()
    );

    Serial.println(
        ENDPOINT_FRAME
    );

    return true;
}

void apagarWiFi() {
    if (servidorActivo) {
        servidor.stop();
        servidorActivo = false;
    }

    MDNS.end();

    WiFi.disconnect(
        true,
        false
    );

    WiFi.mode(WIFI_OFF);

    Serial.println(
        "WiFi apagado."
    );
}

// ============================================================
// SERVIDOR WEB
// ============================================================

void manejarPaginaPrincipal() {
    if (!grabacionActiva) {
        servidor.send(
            503,
            "text/plain",
            "La captura esta desactivada."
        );

        return;
    }

    servidor.send_P(
        200,
        "text/html",
        PAGINA_HTML
    );
}

void manejarUltimoFrame() {
    if (
        !grabacionActiva ||
        ultimoFrame == nullptr ||
        ultimoFrameLongitud == 0
    ) {
        servidor.send(
            503,
            "text/plain",
            "Todavia no hay una imagen disponible."
        );

        return;
    }

    WiFiClient cliente =
        servidor.client();

    cliente.print(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: image/jpeg\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-store, no-cache, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "Content-Length: "
    );

    cliente.print(
        ultimoFrameLongitud
    );

    cliente.print(
        "\r\n\r\n"
    );

    cliente.write(
        ultimoFrame,
        ultimoFrameLongitud
    );
}

// ============================================================
// STREAM MJPEG
//
// Uso desde cualquier página HTML:
// <img src="http://192.168.1.80/stream">
// ============================================================

void manejarStream() {
    if (
        !grabacionActiva ||
        !camaraLista ||
        !sdLista
    ) {
        servidor.send(
            503,
            "text/plain",
            "La captura esta desactivada."
        );

        return;
    }

    WiFiClient cliente =
        servidor.client();

    cliente.print(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-store, no-cache, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n"
    );

    unsigned long ultimoEnvioStreamMs = 0;

    while (
        cliente.connected() &&
        digitalRead(PIN_ACTIVACION) == HIGH
    ) {
        unsigned long ahora =
            millis();

        // Mientras el cliente mira el stream, este mismo handler
        // mantiene la captura y el guardado en la microSD.
        if (
            ahora -
            ultimoFrameMs >=
            INTERVALO_CAPTURA_MS
        ) {
            ultimoFrameMs = ahora;

            capturarYGuardarFrame();
        }

        if (
            ultimoFrame != nullptr &&
            ultimoFrameLongitud > 0 &&
            ahora -
            ultimoEnvioStreamMs >=
            INTERVALO_CAPTURA_MS
        ) {
            ultimoEnvioStreamMs = ahora;

            cliente.print(
                "--frame\r\n"
                "Content-Type: image/jpeg\r\n"
                "Content-Length: "
            );

            cliente.print(
                ultimoFrameLongitud
            );

            cliente.print(
                "\r\n\r\n"
            );

            size_t enviados =
                cliente.write(
                    ultimoFrame,
                    ultimoFrameLongitud
                );

            cliente.print(
                "\r\n"
            );

            if (
                enviados !=
                ultimoFrameLongitud
            ) {
                break;
            }
        }

        delay(5);
    }

    cliente.stop();
}

void configurarServidor() {
    servidor.on(
        "/",
        HTTP_GET,
        manejarPaginaPrincipal
    );

    servidor.on(
        ENDPOINT_STREAM,
        HTTP_GET,
        manejarStream
    );

    servidor.on(
        ENDPOINT_FRAME,
        HTTP_GET,
        manejarUltimoFrame
    );

    servidor.onNotFound(
        []() {
            servidor.send(
                404,
                "text/plain",
                "Ruta no encontrada."
            );
        }
    );
}

void iniciarServidor() {
    if (
        servidorActivo ||
        WiFi.status() != WL_CONNECTED
    ) {
        return;
    }

    servidor.begin();
    servidorActivo = true;

    if (MDNS.begin(NOMBRE_HOST)) {
        Serial.print(
            "Nombre local: http://"
        );

        Serial.print(
            NOMBRE_HOST
        );

        Serial.println(
            ".local"
        );
    }

    Serial.println(
        "Servidor de camara iniciado."
    );

    Serial.print(
        "Usa en HTML: <img src=\"http://"
    );

    Serial.print(
        WiFi.localIP()
    );

    Serial.println(
        "/stream\">"
    );
}

// ============================================================
// CAPTURAR, GUARDAR Y ACTUALIZAR LA VISTA WEB
// ============================================================

void capturarYGuardarFrame() {
    if (
        !grabacionActiva ||
        !camaraLista ||
        !sdLista
    ) {
        return;
    }

    camera_fb_t *frame =
        esp_camera_fb_get();

    if (frame == nullptr) {
        Serial.println(
            "ERROR: no se pudo obtener un frame."
        );

        return;
    }

    if (
        frame->format !=
        PIXFORMAT_JPEG
    ) {
        Serial.println(
            "ERROR: el frame no esta en JPEG."
        );

        esp_camera_fb_return(frame);
        return;
    }

    numeroFrame++;

    char ruta[96];

    snprintf(
        ruta,
        sizeof(ruta),
        "%s/frame_%08lu.jpg",
        carpetaSesion,
        static_cast<unsigned long>(
            numeroFrame
        )
    );

    File archivo =
        SD_MMC.open(
            ruta,
            FILE_WRITE
        );

    if (!archivo) {
        Serial.print(
            "ERROR: no se pudo crear "
        );

        Serial.println(
            ruta
        );
    }
    else {
        size_t escritos =
            archivo.write(
                frame->buf,
                frame->len
            );

        archivo.close();

        if (escritos != frame->len) {
            Serial.print(
                "ADVERTENCIA: frame incompleto en "
            );

            Serial.println(
                ruta
            );
        }
    }

    // Conserva una copia del último frame para la página.
    if (
        ultimoFrame != nullptr &&
        frame->len <= MAXIMO_TAMANO_FRAME
    ) {
        memcpy(
            ultimoFrame,
            frame->buf,
            frame->len
        );

        ultimoFrameLongitud =
            frame->len;
    }

    esp_camera_fb_return(frame);
}

// ============================================================
// ACTIVAR Y DESACTIVAR EL SISTEMA
// ============================================================

void activarSistema() {
    if (grabacionActiva) {
        return;
    }

    if (
        !camaraLista ||
        !sdLista
    ) {
        Serial.println(
            "ERROR: camara o microSD no disponibles."
        );

        return;
    }

    if (!crearNuevaSesion()) {
        return;
    }

    grabacionActiva = true;
    ultimoFrameLongitud = 0;
    ultimoFrameMs = 0;
    ultimoIntentoWifiMs = millis();

    Serial.println(
        "PIN HIGH: captura y guardado activados."
    );

    if (conectarWiFi()) {
        iniciarServidor();
    }
}

void desactivarSistema() {
    if (!grabacionActiva) {
        return;
    }

    grabacionActiva = false;
    ultimoFrameLongitud = 0;

    apagarWiFi();

    Serial.println(
        "PIN LOW: captura, guardado y transmision detenidos."
    );
}

// ============================================================
// LEER PIN CON ANTIRREBOTE
// ============================================================

void actualizarEstadoPin() {
    bool lecturaActual =
        digitalRead(
            PIN_ACTIVACION
        ) == HIGH;

    if (
        lecturaActual !=
        lecturaBrutaAnterior
    ) {
        lecturaBrutaAnterior =
            lecturaActual;

        inicioCambioPinMs =
            millis();
    }

    if (
        millis() -
        inicioCambioPinMs <
        TIEMPO_ANTIRREBOTE_MS
    ) {
        return;
    }

    if (
        estadoPinConfirmado ==
        lecturaBrutaAnterior
    ) {
        return;
    }

    estadoPinConfirmado =
        lecturaBrutaAnterior;

    if (estadoPinConfirmado) {
        activarSistema();
    }
    else {
        desactivarSistema();
    }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println(
        "ESP32-CAM iniciando."
    );

    /*
      GPIO3 se convierte en entrada.
      El TX del puerto Serial (GPIO1) continúa disponible para imprimir.
    */
    pinMode(
        PIN_ACTIVACION,
        INPUT_PULLDOWN
    );

    // Debe establecerse antes de iniciar Wi-Fi.
    WiFi.mode(WIFI_MODE_NULL);
    WiFi.setHostname(
        NOMBRE_HOST
    );

    camaraLista =
        iniciarCamara();

    sdLista =
        iniciarMicroSD();

    configurarServidor();

    if (psramFound()) {
        ultimoFrame =
            static_cast<uint8_t *>(
                ps_malloc(
                    MAXIMO_TAMANO_FRAME
                )
            );
    }
    else {
        ultimoFrame =
            static_cast<uint8_t *>(
                malloc(
                    MAXIMO_TAMANO_FRAME
                )
            );
    }

    if (ultimoFrame == nullptr) {
        Serial.println(
            "ADVERTENCIA: no hay memoria para la vista web."
        );
    }

    lecturaBrutaAnterior =
        digitalRead(
            PIN_ACTIVACION
        ) == HIGH;

    estadoPinConfirmado = false;
    inicioCambioPinMs = millis();

    Serial.println(
        "Listo. Esperando HIGH en GPIO3."
    );
}

// ============================================================
// LOOP
// ============================================================

void loop() {
    actualizarEstadoPin();

    if (!grabacionActiva) {
        delay(10);
        return;
    }

    // La grabación continúa aunque temporalmente falle el Wi-Fi.
    unsigned long ahora =
        millis();

    if (
        ahora -
        ultimoFrameMs >=
        INTERVALO_CAPTURA_MS
    ) {
        ultimoFrameMs = ahora;

        capturarYGuardarFrame();
    }

    if (WiFi.status() == WL_CONNECTED) {
        if (!servidorActivo) {
            iniciarServidor();
        }

        servidor.handleClient();
    }
    else {
        if (
            ahora -
            ultimoIntentoWifiMs >=
            INTERVALO_REINTENTO_WIFI_MS
        ) {
            ultimoIntentoWifiMs = ahora;

            if (conectarWiFi()) {
                iniciarServidor();
            }
        }
    }

    delay(2);
}
