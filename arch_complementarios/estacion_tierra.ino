#include <Arduino.h>
#include <RadioLib.h>

// ============================================================
// PAQUETE DE TELEMETRÍA
// Debe coincidir exactamente con el CubeSat.
// ============================================================

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


// ============================================================
// COMANDOS BINARIOS
//
// La estación transmite exactamente 1 byte por comando.
// Estos valores deben coincidir con el enum del CubeSat.
// ============================================================

enum ComandoRadio : uint8_t {
    COMANDO_ACK = 0,
    COMANDO_STANDBY = 1,
    COMANDO_TELEMETRIA_BASICA = 2,
    COMANDO_TELEMETRIA_COMPLETA = 3,
    COMANDO_HABILITAR_PARACAIDAS = 4,
    COMANDO_ACTIVAR_CAMARA = 5
};

static_assert(
    sizeof(ComandoRadio) == 1,
    "ComandoRadio debe ocupar exactamente 1 byte"
);


// ============================================================
// CONFIGURACIÓN LORA
// ============================================================

const int PIN_LORA_NSS = 5;
const int PIN_LORA_DIO0 = 4;
const int PIN_LORA_RESET = 22;
const int PIN_LORA_DIO1 = 3;

const float FRECUENCIA_LORA = 915.0f;
const int FACTOR_PROPAGACION_LORA = 7;
const float ANCHO_BANDA_LORA = 500.0f;

SX1276 radio = new Module(
    PIN_LORA_NSS,
    PIN_LORA_DIO0,
    PIN_LORA_RESET,
    PIN_LORA_DIO1
);


// ============================================================
// CONFIGURACIÓN DE BOTONES
//
// Conexión de cada botón:
//
// GPIO -------- BOTÓN -------- GND
//
// INPUT_PULLUP:
// Botón libre      -> HIGH
// Botón presionado -> LOW
// ============================================================

const int PIN_BOTON_STANDBY = 21;
const int PIN_BOTON_TELEMETRIA = 16;
const int PIN_BOTON_PARACAIDAS = 17;
const int PIN_BOTON_CAMARA = 15;

const unsigned long TIEMPO_REBOTE_BOTON = 300;


// ============================================================
// BANDERAS DE LOS BOTONES
// ============================================================

volatile bool eventoBotonStandby = false;
volatile bool eventoBotonTelemetria = false;
volatile bool eventoBotonParacaidas = false;
volatile bool eventoBotonCamara = false;


// ============================================================
// TIEMPOS DEL ANTIRREBOTE
// ============================================================

unsigned long ultimoStandbyMillis = 0;
unsigned long ultimaTelemetriaMillis = 0;
unsigned long ultimoParacaidasMillis = 0;
unsigned long ultimaCamaraMillis = 0;


// ============================================================
// VARIABLES DE CONTROL DEL RADIO
// ============================================================

volatile bool operacionTerminada = false;

// DIO0 se usa tanto para TxDone como para RxDone.
// Esta variable permite distinguir ambos casos.
bool transmitiendo = false;


// ============================================================
// VARIABLES PARA COMANDOS
// ============================================================

// El usuario todavía puede escribir nombres por Serial.
// Esos nombres se convierten a un ComandoRadio de 1 byte.
String entradaSerial = "";

// Comando esperando una oportunidad para transmitirse.
ComandoRadio comandoPendiente = COMANDO_ACK;
bool hayComandoPendiente = false;

// Byte estable utilizado por la transmisión no bloqueante.
uint8_t byteComandoEnTransmision =
    static_cast<uint8_t>(COMANDO_ACK);


// ============================================================
// INTERRUPCIÓN DEL RADIO
// ============================================================

void cambiarBanderaRadio() {
    operacionTerminada = true;
}


// ============================================================
// INTERRUPCIONES DE LOS BOTONES
// ============================================================

void IRAM_ATTR interrupcionBotonStandby() {
    eventoBotonStandby = true;
}

void IRAM_ATTR interrupcionBotonTelemetria() {
    eventoBotonTelemetria = true;
}

void IRAM_ATTR interrupcionBotonParacaidas() {
    eventoBotonParacaidas = true;
}

void IRAM_ATTR interrupcionBotonCamara() {
    eventoBotonCamara = true;
}


// ============================================================
// NOMBRES PARA DEPURACIÓN
// ============================================================

const char *obtenerNombreComando(
    ComandoRadio comando
) {
    switch (comando) {
        case COMANDO_ACK:
            return "ACK";

        case COMANDO_STANDBY:
            return "STANDBY";

        case COMANDO_TELEMETRIA_BASICA:
            return "TELEMETRIA_BASICA";

        case COMANDO_TELEMETRIA_COMPLETA:
            return "TELEMETRIA_COMPLETA";

        case COMANDO_HABILITAR_PARACAIDAS:
            return "HABILITAR_PARACAIDAS";

        case COMANDO_ACTIVAR_CAMARA:
            return "ACTIVAR_CAMARA";

        default:
            return "DESCONOCIDO";
    }
}


const char *obtenerNombreTipoPaquete(
    uint8_t tipo
) {
    switch (tipo) {
        case 0:
            return "HEARTBEAT";

        case 1:
            return "STANDBY";

        case 2:
            return "TELEMETRIA_BASICA";

        case 3:
            return "TELEMETRIA_COMPLETA";

        case 4:
            return "POST_CAIDA";

        default:
            return "DESCONOCIDO";
    }
}


// ============================================================
// VOLVER A RECEPCIÓN LORA
// ============================================================

void volverARecepcion() {
    int estado;

    estado = radio.startReceive();

    if (estado != RADIOLIB_ERR_NONE) {
        Serial.print("# ERROR_RECEPCION,");
        Serial.println(estado);
    }
}


// ============================================================
// TRANSMITIR UN COMANDO BINARIO
//
// Se transmite exactamente 1 byte.
// No se transmite String ni texto ASCII.
// ============================================================

bool iniciarTransmisionComandoBinario(
    ComandoRadio comando,
    const char *motivo
) {
    if (transmitiendo == true) {
        return false;
    }

    byteComandoEnTransmision =
        static_cast<uint8_t>(comando);

    int estado;

    estado = radio.startTransmit(
        &byteComandoEnTransmision,
        sizeof(byteComandoEnTransmision)
    );

    if (estado != RADIOLIB_ERR_NONE) {
        Serial.print(
            "# ERROR_ENVIO_COMANDO,"
        );

        Serial.print(
            static_cast<unsigned int>(
                byteComandoEnTransmision
            )
        );

        Serial.print(",");

        Serial.print(
            obtenerNombreComando(comando)
        );

        Serial.print(",");

        Serial.print(motivo);

        Serial.print(",");

        Serial.println(estado);

        return false;
    }

    transmitiendo = true;

    Serial.print(
        "# COMANDO_BINARIO_TRANSMITIENDO,ID="
    );

    Serial.print(
        static_cast<unsigned int>(
            byteComandoEnTransmision
        )
    );

    Serial.print(",NOMBRE=");

    Serial.print(
        obtenerNombreComando(comando)
    );

    Serial.print(",ORIGEN=");

    Serial.println(motivo);

    return true;
}


// ============================================================
// ENCOLAR UN COMANDO
// ============================================================

bool encolarComando(
    ComandoRadio comando,
    const char *origen
) {
    if (hayComandoPendiente == true) {
        Serial.print(
            "# COMANDO_IGNORADO,"
        );

        Serial.print(origen);

        Serial.println(
            ",YA_EXISTE_COMANDO_PENDIENTE"
        );

        return false;
    }

    comandoPendiente = comando;
    hayComandoPendiente = true;

    Serial.print(
        "# COMANDO_ENCOLADO,ID="
    );

    Serial.print(
        static_cast<unsigned int>(
            comandoPendiente
        )
    );

    Serial.print(",NOMBRE=");

    Serial.print(
        obtenerNombreComando(
            comandoPendiente
        )
    );

    Serial.print(",ORIGEN=");

    Serial.println(origen);

    return true;
}


// ============================================================
// CONVERTIR TEXTO DEL MONITOR SERIAL A COMANDO BINARIO
//
// También acepta directamente los números 0 a 5.
// ============================================================

bool convertirComando(
    String entrada,
    ComandoRadio *comandoLoRa
) {
    entrada.trim();
    entrada.toLowerCase();

    if (
        entrada == "ack" ||
        entrada == "0"
    ) {
        *comandoLoRa = COMANDO_ACK;
        return true;
    }

    if (
        entrada == "stand by" ||
        entrada == "standby" ||
        entrada == "1"
    ) {
        *comandoLoRa = COMANDO_STANDBY;
        return true;
    }

    if (
        entrada == "tomardatosbasicos" ||
        entrada == "basica" ||
        entrada == "basicos" ||
        entrada == "2"
    ) {
        *comandoLoRa =
            COMANDO_TELEMETRIA_BASICA;

        return true;
    }

    if (
        entrada == "tomardatostotales" ||
        entrada == "completa" ||
        entrada == "totales" ||
        entrada == "3"
    ) {
        *comandoLoRa =
            COMANDO_TELEMETRIA_COMPLETA;

        return true;
    }

    if (
        entrada == "habilitar paracaidas" ||
        entrada == "habilitarparacaidas" ||
        entrada == "paracaidas" ||
        entrada == "4"
    ) {
        *comandoLoRa =
            COMANDO_HABILITAR_PARACAIDAS;

        return true;
    }

    if (
        entrada == "activar camara" ||
        entrada == "activarcamara" ||
        entrada == "camara" ||
        entrada == "5"
    ) {
        *comandoLoRa =
            COMANDO_ACTIVAR_CAMARA;

        return true;
    }

    return false;
}


// ============================================================
// PROCESAR UNA LÍNEA DEL MONITOR SERIAL
// ============================================================

void procesarLineaSerial(
    String linea
) {
    ComandoRadio comando;

    linea.trim();

    if (linea.length() == 0) {
        return;
    }

    if (
        convertirComando(
            linea,
            &comando
        ) == false
    ) {
        Serial.println(
            "# COMANDO_NO_RECONOCIDO"
        );

        Serial.println(
            "# Use: standby, basica, completa, "
            "paracaidas, camara o IDs 1-5"
        );

        return;
    }

    encolarComando(
        comando,
        "SERIAL"
    );
}


// ============================================================
// LEER COMANDOS DESDE LA COMPUTADORA
// ============================================================

void leerSerial() {
    while (Serial.available() > 0) {
        char caracter;

        caracter =
            static_cast<char>(
                Serial.read()
            );

        if (caracter == '\n' || caracter == '\r') {
            if (entradaSerial.length() > 0) {
                procesarLineaSerial(
                    entradaSerial
                );

                entradaSerial = "";
            }
        }
        else {
            if (entradaSerial.length() < 80) {
                entradaSerial += caracter;
            }
        }
    }
}


// ============================================================
// EXTRAER UNA BANDERA DE INTERRUPCIÓN
// ============================================================

bool obtenerEventoBoton(
    volatile bool &evento
) {
    bool resultado;

    noInterrupts();

    resultado = evento;
    evento = false;

    interrupts();

    return resultado;
}


// ============================================================
// PROCESAR LOS CUATRO BOTONES
// ============================================================

void procesarBotones() {
    unsigned long tiempoActual;

    tiempoActual = millis();

    // Botón 1: COMANDO_STANDBY = 1.
    if (
        obtenerEventoBoton(
            eventoBotonStandby
        )
    ) {
        if (
            tiempoActual -
            ultimoStandbyMillis >=
            TIEMPO_REBOTE_BOTON
        ) {
            ultimoStandbyMillis =
                tiempoActual;

            encolarComando(
                COMANDO_STANDBY,
                "BOTON_STANDBY"
            );
        }
    }

    // Botón 2: COMANDO_TELEMETRIA_COMPLETA = 3.
    if (
        obtenerEventoBoton(
            eventoBotonTelemetria
        )
    ) {
        if (
            tiempoActual -
            ultimaTelemetriaMillis >=
            TIEMPO_REBOTE_BOTON
        ) {
            ultimaTelemetriaMillis =
                tiempoActual;

            encolarComando(
                COMANDO_TELEMETRIA_COMPLETA,
                "BOTON_TELEMETRIA"
            );
        }
    }

    // Botón 3: COMANDO_HABILITAR_PARACAIDAS = 4.
    if (
        obtenerEventoBoton(
            eventoBotonParacaidas
        )
    ) {
        if (
            tiempoActual -
            ultimoParacaidasMillis >=
            TIEMPO_REBOTE_BOTON
        ) {
            ultimoParacaidasMillis =
                tiempoActual;

            encolarComando(
                COMANDO_HABILITAR_PARACAIDAS,
                "BOTON_PARACAIDAS"
            );
        }
    }

    // Botón 4: COMANDO_ACTIVAR_CAMARA = 5.
    if (
        obtenerEventoBoton(
            eventoBotonCamara
        )
    ) {
        if (
            tiempoActual -
            ultimaCamaraMillis >=
            TIEMPO_REBOTE_BOTON
        ) {
            ultimaCamaraMillis =
                tiempoActual;

            encolarComando(
                COMANDO_ACTIVAR_CAMARA,
                "BOTON_CAMARA"
            );
        }
    }
}


// ============================================================
// IMPRIMIR TELEMETRÍA CON PREFIJOS Y CONVERSIONES
// ============================================================

void imprimirPaqueteCSV(
    const TelemetryPacket *paquete
) {
    float tiempoSegundos =
        paquete->TIME / 10.0f;

    float voltajeVoltios =
        paquete->VOLT / 1000.0f;

    float pitchRadianes =
        paquete->PITCH / 1000.0f;

    float rollRadianes =
        paquete->ROLL / 1000.0f;

    float pitchGrados =
        pitchRadianes *
        180.0f /
        PI;

    float rollGrados =
        rollRadianes *
        180.0f /
        PI;

    double longitudGrados =
        paquete->LON /
        10000000.0;

    double latitudGrados =
        paquete->LAT /
        10000000.0;

    float velocidadVertical =
        paquete->VVEL /
        10.0f;

    float presionHpa =
        paquete->PRES /
        100.0f;

    float temperaturaKelvin =
        paquete->TEMP /
        100.0f;

    float temperaturaCelsius =
        temperaturaKelvin -
        273.15f;

    float indiceUV =
        paquete->UV /
        100.0f;

    float giroX =
        paquete->GYRX /
        1000.0f;

    float giroY =
        paquete->GYRY /
        1000.0f;

    float giroZ =
        paquete->GYRZ /
        1000.0f;

    float aceleracionX =
        paquete->ACCX /
        1000.0f;

    float aceleracionY =
        paquete->ACCY /
        1000.0f;

    float aceleracionZ =
        paquete->ACCZ /
        1000.0f;

    float altitudMetros =
        paquete->ALT /
        10.0f;

    
    Serial.print(paquete->TYPE);

    /*Serial.print(",");
    Serial.print(
        obtenerNombreTipoPaquete(
            paquete->TYPE
        )
    );
    */

    Serial.print(",");
    Serial.print(paquete->SEQ);

    Serial.print(",");
    Serial.print(tiempoSegundos, 1);

    Serial.print(",");
    Serial.print(paquete->FLAGS);

    Serial.print(",");
    Serial.print(voltajeVoltios, 3);

    Serial.print(",");
    Serial.print(pitchGrados, 2);

    Serial.print(",");
    Serial.print(rollGrados, 2);

    Serial.print(",");
    Serial.print(longitudGrados, 7);

    Serial.print(",");
    Serial.print(latitudGrados, 7);

    Serial.print(",");
    Serial.print(velocidadVertical, 1);

    Serial.print(",");
    Serial.print(presionHpa, 2);

    Serial.print(",");
    Serial.print(temperaturaCelsius, 2);

    Serial.print(",");
    Serial.print(paquete->ECO2);

    Serial.print(",");
    Serial.print(paquete->ETOH);

    Serial.print(",");
    Serial.print(
        static_cast<unsigned int>(
            paquete->AQI
        )
    );

    Serial.print(",");
    Serial.print(indiceUV, 2);

    Serial.print(",");
    Serial.print(giroX, 3);

    Serial.print(",");
    Serial.print(giroY, 3);

    Serial.print(",");
    Serial.print(giroZ, 3);

    Serial.print(",");
    Serial.print(aceleracionX, 3);

    Serial.print(",");
    Serial.print(aceleracionY, 3);

    Serial.print(",");
    Serial.print(aceleracionZ, 3);

    Serial.print(",");
    Serial.print(altitudMetros, 1);

    Serial.print(",");
    Serial.println(paquete->CHK);
}


// ============================================================
// TRANSMITIR EL COMANDO PENDIENTE EN LA VENTANA POST-PAQUETE
// ============================================================

bool transmitirComandoEnVentana() {
    if (
        hayComandoPendiente == false ||
        transmitiendo == true
    ) {
        return false;
    }

    // Da tiempo al CubeSat para cambiar de TX a RX.
    delayMicroseconds(3000);

    ComandoRadio comandoAEnviar =
        comandoPendiente;

    bool iniciado =
        iniciarTransmisionComandoBinario(
            comandoAEnviar,
            "VENTANA_POST_PAQUETE"
        );

    if (iniciado == true) {
        hayComandoPendiente = false;
        comandoPendiente = COMANDO_ACK;
    }

    return iniciado;
}


// ============================================================
// PROCESAR PAQUETE RECIBIDO
// ============================================================

void procesarPaqueteRecibido() {
    uint8_t buffer[
        sizeof(TelemetryPacket)
    ];

    size_t longitudPaquete;
    int estado;

    longitudPaquete =
        radio.getPacketLength();

    if (
        longitudPaquete !=
        sizeof(TelemetryPacket)
    ) {
        Serial.print(
            "# TAMANO_INCORRECTO,"
        );

        Serial.println(
            longitudPaquete
        );

        uint8_t descarte[255];

        radio.readData(
            descarte,
            sizeof(descarte)
        );

        volverARecepcion();

        return;
    }

    estado = radio.readData(
        buffer,
        sizeof(buffer)
    );

    if (
        estado !=
        RADIOLIB_ERR_NONE
    ) {
        Serial.print(
            "# ERROR_LECTURA,"
        );

        Serial.println(estado);

        volverARecepcion();

        return;
    }

    TelemetryPacket *paquete;

    paquete =
        reinterpret_cast<
            TelemetryPacket *
        >(
            buffer
        );

    // TYPE 0: el CubeSat espera COMANDO_ACK = 0.
    if (paquete->TYPE == 0) {
        Serial.println(
            "# HEARTBEAT_RECIBIDO"
        );

        delayMicroseconds(3000);

        bool ackIniciado =
            iniciarTransmisionComandoBinario(
                COMANDO_ACK,
                "RESPUESTA_HEARTBEAT"
            );

        if (ackIniciado == false) {
            volverARecepcion();
        }

        return;
    }

    if (
        paquete->TYPE == 1 ||
        paquete->TYPE == 2 ||
        paquete->TYPE == 3 ||
        paquete->TYPE == 4
    ) {
        // El comando pendiente se inicia antes de imprimir.
        bool comandoIniciado =
            transmitirComandoEnVentana();

        imprimirPaqueteCSV(paquete);

        if (comandoIniciado == false) {
            volverARecepcion();
        }

        return;
    }

    Serial.print(
        "# TYPE_DESCONOCIDO,"
    );

    Serial.println(
        paquete->TYPE
    );

    volverARecepcion();
}


// ============================================================
// CONFIGURAR BOTONES
// ============================================================

void configurarBotones() {
    pinMode(
        PIN_BOTON_STANDBY,
        INPUT_PULLUP
    );

    pinMode(
        PIN_BOTON_TELEMETRIA,
        INPUT_PULLUP
    );

    pinMode(
        PIN_BOTON_PARACAIDAS,
        INPUT_PULLUP
    );

    pinMode(
        PIN_BOTON_CAMARA,
        INPUT_PULLUP
    );

    attachInterrupt(
        digitalPinToInterrupt(
            PIN_BOTON_STANDBY
        ),
        interrupcionBotonStandby,
        FALLING
    );

    attachInterrupt(
        digitalPinToInterrupt(
            PIN_BOTON_TELEMETRIA
        ),
        interrupcionBotonTelemetria,
        FALLING
    );

    attachInterrupt(
        digitalPinToInterrupt(
            PIN_BOTON_PARACAIDAS
        ),
        interrupcionBotonParacaidas,
        FALLING
    );

    attachInterrupt(
        digitalPinToInterrupt(
            PIN_BOTON_CAMARA
        ),
        interrupcionBotonCamara,
        FALLING
    );
}


// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(115200);

    configurarBotones();

    Serial.println(
        "# ESTACION_TIERRA_INICIANDO"
    );

    int estado;

    estado = radio.begin(
        FRECUENCIA_LORA
    );

    if (
        estado !=
        RADIOLIB_ERR_NONE
    ) {
        Serial.print(
            "# ERROR_LORA,"
        );

        Serial.println(estado);

        while (true) {
            delay(10);
        }
    }

    estado =
        radio.setSpreadingFactor(
            FACTOR_PROPAGACION_LORA
        );

    if (
        estado !=
        RADIOLIB_ERR_NONE
    ) {
        Serial.print(
            "# ERROR_SPREADING_FACTOR,"
        );

        Serial.println(estado);
    }

    estado =
        radio.setBandwidth(
            ANCHO_BANDA_LORA
        );

    if (
        estado !=
        RADIOLIB_ERR_NONE
    ) {
        Serial.print(
            "# ERROR_ANCHO_BANDA,"
        );

        Serial.println(estado);
    }

    radio.setDio0Action(
        cambiarBanderaRadio,
        RISING
    );

    volverARecepcion();

    Serial.println(
        "# ESTACION_TIERRA_LISTA"
    );

    Serial.println(
        "# BOTON GPIO21: ID 1 - STANDBY"
    );

    Serial.println(
        "# BOTON GPIO16: ID 3 - TELEMETRIA_COMPLETA"
    );

    Serial.println(
        "# BOTON GPIO17: ID 4 - HABILITAR_PARACAIDAS"
    );

    Serial.println(
        "# BOTON GPIO15: ID 5 - ACTIVAR_CAMARA"
    );

    Serial.println(
        "# Serial: standby, basica, completa, "
        "paracaidas, camara o IDs 1-5"
    );
}


// ============================================================
// LOOP
// ============================================================

void loop() {
    leerSerial();

    procesarBotones();

    if (operacionTerminada == true) {
        operacionTerminada = false;

        if (transmitiendo == true) {
            // Terminó de transmitirse el ACK o un comando.
            transmitiendo = false;

            volverARecepcion();
        }
        else {
            // Terminó de recibirse un paquete.
            procesarPaqueteRecibido();
        }
    }
}