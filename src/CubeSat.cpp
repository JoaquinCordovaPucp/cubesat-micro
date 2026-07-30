#include "CubeSat.hpp"

#include <Configuracion.hpp>

CubeSat::CubeSat() {
    estadoActual = ESPERANDO_ACK;

    altitudReferenciaBarometro = 0.0f;

    ultimoEnvioMillis = 0;
    ultimoFiltroMillis = 0;
    ultimoFiltroMicros = 0;

    inicioPruebaMotoresMillis = 0;
    segundoPasoMotoresRealizado = false;

    datosSensores = {};
    datosGPS = {};
    paquete = {};
}

void CubeSat::iniciar() {
    sensores.iniciar(&Serial);

    controladorACS.begin(
        PIN_ROLL,
        PIN_PITCH,
        PULSO_MINIMO_ACS,
        PULSO_MAXIMO_ACS
    );

    moduloGPS.iniciar();

    sistemaEyeccion.begin(
        PIN_MOTOR_EYECCION
    );

    radio.iniciar();

    filtroAltitud.begin(
        GANANCIA_POSICION_FILTRO,
        GANANCIA_VELOCIDAD_FILTRO,
        UMBRAL_REPOSO
    );

    calibrarBarometro();

    ultimoFiltroMillis = millis();
    ultimoFiltroMicros = micros();

    inicioPruebaMotoresMillis = millis();

    controladorACS.setRollOutput(
        PULSO_PRUEBA_INICIAL
    );

    controladorACS.setPitchOutput(
        PULSO_PRUEBA_INICIAL
    );

    Serial.println(
        "CubeSat inicializado."
    );

    Serial.println(
        "Prueba de motores iniciada."
    );
}

void CubeSat::actualizar() {
    sistemaEyeccion.activate();

    actualizarPruebaMotores();
    actualizarFiltroAltitud();

    radio.actualizar();
    procesarMensajesRadio();

    moduloGPS.actualizar();

    ejecutarEstadoActual();
}

void CubeSat::calibrarBarometro() {
    float sumaAltitudes;

    sumaAltitudes = 0.0f;

    for (
        int i = 0;
        i < CANTIDAD_MUESTRAS_CALIBRACION;
        i++
    ) {
        sumaAltitudes +=
            sensores.obtenerAltitudBarometrica();

        delay(RETARDO_CALIBRACION);
    }

    altitudReferenciaBarometro =
        sumaAltitudes /
        CANTIDAD_MUESTRAS_CALIBRACION;

    Serial.print(
        "Altitud de referencia del barometro: "
    );

    Serial.print(
        altitudReferenciaBarometro
    );

    Serial.println(" m");
}

void CubeSat::actualizarPruebaMotores() {
    unsigned long tiempoActual;

    tiempoActual = millis();

    if (
        segundoPasoMotoresRealizado == false &&
        tiempoActual -
        inicioPruebaMotoresMillis >=
        TIEMPO_PRUEBA_MOTOR
    ) {
        controladorACS.setRollOutput(
            PULSO_PRUEBA_FINAL
        );

        controladorACS.setPitchOutput(
            PULSO_PRUEBA_FINAL
        );

        segundoPasoMotoresRealizado = true;

        Serial.println(
            "Prueba de motores cambiada al segundo nivel."
        );
    }
}

void CubeSat::actualizarFiltroAltitud() {
    unsigned long tiempoActualMillis;

    tiempoActualMillis = millis();

    if (
        tiempoActualMillis -
        ultimoFiltroMillis <
        INTERVALO_FILTRO_ALTITUD
    ) {
        return;
    }

    unsigned long tiempoActualMicros;
    float tiempoTranscurrido;
    float aceleracionVertical;
    float altitudBarometricaRelativa;

    tiempoActualMicros = micros();

    tiempoTranscurrido =
        (float)(
            tiempoActualMicros -
            ultimoFiltroMicros
        ) /
        1000000.0f;

    ultimoFiltroMillis =
        tiempoActualMillis;

    ultimoFiltroMicros =
        tiempoActualMicros;

    if (
        tiempoTranscurrido <= 0.0f ||
        tiempoTranscurrido > 0.5f
    ) {
        tiempoTranscurrido =
            INTERVALO_FILTRO_ALTITUD /
            1000.0f;
    }

    aceleracionVertical = 0.0f;

    altitudBarometricaRelativa =
        sensores.obtenerAltitudBarometrica() -
        altitudReferenciaBarometro;

    filtroAltitud.estimate(
        aceleracionVertical,
        altitudBarometricaRelativa,
        tiempoTranscurrido
    );
}

void CubeSat::procesarMensajesRadio() {
    if (radio.hayMensaje() == false) {
        return;
    }

    String mensaje;

    mensaje = radio.obtenerMensaje();

    procesarMensaje(mensaje);
}

void CubeSat::procesarMensaje(
    String mensaje
) {
    Serial.println(
        "Se recibio un mensaje:"
    );

    Serial.println(mensaje);

    if (estadoActual == ESPERANDO_ACK) {
        int posicionSeparador;
        String confirmacion;
        String comando;

        posicionSeparador =
            mensaje.indexOf('&');

        if (posicionSeparador != -1) {
            confirmacion =
                mensaje.substring(
                    0,
                    posicionSeparador
                );

            comando =
                mensaje.substring(
                    posicionSeparador + 1
                );
        }
        else {
            confirmacion = mensaje;
            comando = "";
        }

        if (confirmacion == "ack") {
            estadoActual = EN_ESPERA;

            Serial.println(
                "Se recibio ACK de la estacion en tierra."
            );

            if (comando != "") {
                estadoActual =
                    obtenerEstadoPorComando(
                        comando
                    );
            }
        }

        return;
    }

    estadoActual =
        obtenerEstadoPorComando(
            mensaje
        );
}

EstadoCubeSat CubeSat::obtenerEstadoPorComando(String comando) {
    if (comando == "Stand By") {
        return EN_ESPERA;
    }

    if (comando == "TomarDatosBasicos") {
        return TELEMETRIA_BASICA;
    }

    if (comando == "TomarDatosTotales") {
        return TELEMETRIA_COMPLETA;
    }

    return estadoActual;
}

void CubeSat::ejecutarEstadoActual() {
    switch (estadoActual) {
        case ESPERANDO_ACK:
            ejecutarEsperandoACK();
            break;

        case EN_ESPERA:
            ejecutarStandBy();
            break;

        case TELEMETRIA_BASICA:
            ejecutarTelemetriaBasica();
            break;

        case TELEMETRIA_COMPLETA:
            ejecutarTelemetriaCompleta();
            break;

        case DEBUG:
            ejecutarDebug();
            break;
    }
}

void CubeSat::ejecutarEsperandoACK() {
    unsigned long tiempoActual;

    tiempoActual = millis();

    if (
        tiempoActual -
        ultimoEnvioMillis <
        INTERVALO_HEARTBEAT
    ) {
        return;
    }

    ultimoEnvioMillis = tiempoActual;

    telemetria.crearHeartbeat(
        &paquete
    );

    enviarPaquete();

    Serial.println(
        "Heartbeat enviado."
    );
}

void CubeSat::ejecutarStandBy() {
    unsigned long tiempoActual;

    tiempoActual = millis();

    if (
        tiempoActual -
        ultimoEnvioMillis <
        INTERVALO_HEARTBEAT
    ) {
        return;
    }

    ultimoEnvioMillis = tiempoActual;

    uint16_t lecturaVoltajeADC;

    lecturaVoltajeADC =
        sensores.obtenerLecturaVoltajeADC();

    telemetria.crearStandBy(
        &paquete,
        lecturaVoltajeADC
    );

    enviarPaquete();

    Serial.println(
        "Paquete Stand By enviado."
    );
}

void CubeSat::ejecutarTelemetriaBasica() {
    unsigned long tiempoActual;

    tiempoActual = millis();

    if (
        tiempoActual -
        ultimoEnvioMillis <
        INTERVALO_TELEMETRIA
    ) {
        return;
    }

    ultimoEnvioMillis = tiempoActual;

    delay(12);

    telemetria.crearPaqueteBasico(
        &paquete
    );

    enviarPaquete();

    Serial.println(
        "Paquete de datos basicos enviado."
    );
}

void CubeSat::ejecutarTelemetriaCompleta() {
    unsigned long tiempoActual;

    tiempoActual = millis();

    if (tiempoActual -ultimoEnvioMillis < INTERVALO_TELEMETRIA) {
        return;
    }

    ultimoEnvioMillis = tiempoActual;

    sensores.leer(&datosSensores);
    
    moduloGPS.actualizar();
    moduloGPS.obtenerDatos(&datosGPS);

    telemetria.crearPaqueteCompleto(&paquete,&datosSensores,&datosGPS,
        filtroAltitud.estimatedAltitude,
        filtroAltitud.estimatedVelocity
    );

    enviarPaquete();

    Serial.println(
        "Paquete de datos completos enviado."
    );
}

void CubeSat::ejecutarDebug() {
    Serial.println("Morimos");

    while (true) {
        delay(100);
    }
}

void CubeSat::enviarPaquete() {
    radio.enviarPaquete(
        reinterpret_cast<uint8_t *>(&paquete),
        sizeof(paquete)
    );
}