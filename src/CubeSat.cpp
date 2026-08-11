#include "CubeSat.hpp"
#include <SPI.h>
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

    paracaidasHabilitado = true;
    paracaidasArmado = false;
    camaraActivada = false;

    primeraEtapaActivada = false;
    segundaEtapaActivada = false;

    inicioConfirmacionArmadoMillis = 0;
    vueloIniciado = false;
    aterrizajeDetectado = false;


    alturaAnterior = 0.0f;
    alturaMaximaAlcanzada = 0.0f;

    inicioReposoMillis = 0;
}

void CubeSat::iniciar() {
    sensores.iniciar(&Serial);

    SPI.begin(
    PIN_SD_SCK,
    PIN_SD_MISO,
    PIN_SD_MOSI
    );

    pinMode(
        PIN_LORA_NSS,
        OUTPUT
    );

    digitalWrite(
        PIN_LORA_NSS,
        HIGH
    );

    registrador.iniciar(
        PIN_SD_CS
    );

    radio.iniciar();
    controladorACS.begin(
        PIN_ROLL,
        PIN_PITCH,
        PULSO_MINIMO_ACS,
        PULSO_MAXIMO_ACS
    );

    moduloGPS.iniciar();

    sistemaParacaidas.begin(
    PIN_PARACAIDAS);

    pinMode(PIN_CAMARA,
        OUTPUT);

    digitalWrite(PIN_CAMARA,LOW);


    calibrarBarometro();

    filtroAltitud.setEstadoInicial(
        0.0f,
        0.0f
    );

    filtroAltitud.setCovarianzaInicial(
        VARIANZA_INICIAL_ALTITUD,
        VARIANZA_INICIAL_VELOCIDAD
    );

    filtroAltitud.setRuidoProceso(
        RUIDO_PROCESO_ALTITUD,
        RUIDO_PROCESO_VELOCIDAD
    );

    filtroAltitud.setRuidoMedicion(
        RUIDO_MEDICION_BAROMETRO
    );

    ultimoFiltroMillis = millis();
    ultimoFiltroMicros = micros();

    
    Serial.println(
        "CubeSat inicializado."
    );


}

void CubeSat::actualizar() {
    registrador.actualizar();

    sistemaParacaidas.update();


    radio.actualizar();
    procesarMensajesRadio();

    moduloGPS.actualizar();

    if (estadoActual != POST_CAIDA){
        // Actualizar la altura y velocidad
        actualizarFiltroAltitud();

        // luego decide si arma, abre o desacopla
        actualizarParacaidas();

        actualizarDeteccionAterrizaje();
    }


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

    if (tiempoActualMillis -
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

    aceleracionVertical =
        sensores.obtenerAceleracionVertical();

    altitudBarometricaRelativa =
        sensores.obtenerAltitudBarometrica() -
        altitudReferenciaBarometro;

    filtroAltitud.actualizar(
    aceleracionVertical,
    altitudBarometricaRelativa,
    tiempoTranscurrido);
}

void CubeSat::procesarMensajesRadio() {
    if (radio.hayComando() == false) {
        return;
    }

    int comando;

    comando = radio.obtenerComando();

    procesarMensaje(comando);
}

void CubeSat::procesarMensaje(
    int codigo
) {
    Serial.println(
        "Se recibio un comando binario:"
    );

    Serial.println(codigo);

    if (codigo == COMANDO_ACK) {
        if (estadoActual == ESPERANDO_ACK) {
            estadoActual = EN_ESPERA;

            Serial.println(
                "Se recibio ACK de la estacion."
            );
        }

        return;
    }

    procesarComando(codigo);
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
        case POST_CAIDA:
            ejecutarPostCaida();
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
        &paquete,
        paracaidasHabilitado,
        paracaidasArmado,
        primeraEtapaActivada,
        segundaEtapaActivada,
        aterrizajeDetectado
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

    float voltajeMilivoltios;

    voltajeMilivoltios =
        sensores.obtenerVoltajeMilivoltios();

    telemetria.crearStandBy(
        &paquete,
        voltajeMilivoltios,
        paracaidasHabilitado,
        paracaidasArmado,
        primeraEtapaActivada,
        segundaEtapaActivada,
        aterrizajeDetectado
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
        &paquete,
        paracaidasHabilitado,
        paracaidasArmado,
        primeraEtapaActivada,
        segundaEtapaActivada,
        aterrizajeDetectado
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

    telemetria.crearPaqueteCompleto(
    &paquete,&datosSensores,&datosGPS,
    filtroAltitud.getAltitud(),
    filtroAltitud.getVelocidadVertical(),
    paracaidasHabilitado,
    paracaidasArmado,
    primeraEtapaActivada,
    segundaEtapaActivada,
    aterrizajeDetectado);

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
    registrador.guardarPaquete(
        &paquete
    );

    radio.enviarPaquete(
        (uint8_t *)&paquete,
        sizeof(paquete)
    );
}

void CubeSat::imprimirPaquete() {
    Serial.print("TYPE=");
    Serial.print(paquete.TYPE);

    Serial.print(", SEQ=");
    Serial.print(paquete.SEQ);

    Serial.print(", TIME=");
    Serial.print(paquete.TIME);

    Serial.print(", FLAGS=");
    Serial.print(paquete.FLAGS);

    Serial.print(", VOLT=");
    Serial.print(paquete.VOLT);

    Serial.print(", PRES=");
    Serial.print(paquete.PRES);

    Serial.print(", TEMP=");
    Serial.print(paquete.TEMP);

    Serial.print(", ECO2=");
    Serial.print(paquete.ECO2);

    Serial.print(", ETOH=");
    Serial.print(paquete.ETOH);

    Serial.print(", AQI=");
    Serial.print(paquete.AQI);

    Serial.print(", UV=");
    Serial.print(paquete.UV);

    Serial.print(", VVEL=");
    Serial.print(paquete.VVEL);

    Serial.print(", ALT=");
    Serial.println(paquete.ALT);
}

void CubeSat::procesarComando(int comando) {
    if (comando == COMANDO_STANDBY) {
        estadoActual = EN_ESPERA;
    }
    else if (
        comando == COMANDO_TELEMETRIA_BASICA
    ) {
        estadoActual =
            TELEMETRIA_BASICA;
    }
    else if (
        comando == COMANDO_TELEMETRIA_COMPLETA
    ) {
        estadoActual =
            TELEMETRIA_COMPLETA;
    }
    else if (
        comando == COMANDO_HABILITAR_PARACAIDAS
    ) {
        paracaidasHabilitado = true;

        Serial.println(
            "Paracaidas habilitado."
        );
    }
    else if (
        comando == COMANDO_ACTIVAR_CAMARA
    ) {
        camaraActivada = true;

        digitalWrite(
            PIN_CAMARA,
            HIGH
        );

        Serial.println(
            "Camara activada."
        );
    }
    else {
        Serial.println(
            "Comando binario no reconocido."
        );
    }
}

void CubeSat::actualizarParacaidas() {
    float alturaActual;
    float velocidadVertical;

    alturaActual = filtroAltitud.getAltitud();

    velocidadVertical = filtroAltitud.getVelocidadVertical();

    unsigned long tiempoActual = millis();

    // 1. No hacer nada si no esta habilitado
    if (paracaidasHabilitado == false) {
        inicioConfirmacionArmadoMillis = 0;
        alturaAnterior = alturaActual;
        return;
    }
    // despues del aterrizaje ya no se ejecutan acciones
    if(aterrizajeDetectado == true){
        alturaAnterior = alturaActual;
        return;
    }
    // confirmar que llego a 100 metros
    if (paracaidasArmado == false){
        // Todavia no comenzo el conteo
        if (inicioConfirmacionArmadoMillis == 0) {
            // conteo comienza cuando llega a 100 m
            if (alturaActual >= ALTURA_INICIO_ARMADO) {
                inicioConfirmacionArmadoMillis = tiempoActual;
                Serial.println("Altura de 100 m alcanzada. " "Comenzando confirmacion de 10s.");
            }
        }
        else {
            // si baja de 95 m antes de terminar los 10 s, se cancela el conteo
            if(alturaActual < ALTURA_CANCELAR_ARMADO){
                inicioConfirmacionArmadoMillis =0;
                Serial.println("Confirmacion cancelada: altura menor a 95 m.");
            }else if (tiempoActual - inicioConfirmacionArmadoMillis >= TIEMPO_CONFIRMACION_ARMADO){
                paracaidasArmado =true;
                vueloIniciado =true;
                Serial.println("Sistema de paracaidas preparado: altura confirmada durante 10s");
            }
        }
        alturaAnterior = alturaActual;
        return;
    }
    // confirmar que esta descendiendo
    bool estaDescendiendo = velocidadVertical <= VELOCIDAD_MINIMA_DESCENSO;
    
    if (estaDescendiendo == false) {
        alturaAnterior = alturaActual;
        return;
    }
    
    if (primeraEtapaActivada == false && alturaActual <= ALTURA_PRIMERA_ETAPA) {
        sistemaParacaidas.activate(DURACION_PRIMERA_ETAPA);
        primeraEtapaActivada = true;
        Serial.print("Paracaidas activado a ");
        Serial.print(alturaActual);
        Serial.println(" m durante 3 segundos.");

        alturaAnterior = alturaActual;
        
        return;
    }

    // detectar el cruce de 3 metros

    if (primeraEtapaActivada == true && segundaEtapaActivada == false && alturaActual <= ALTURA_SEGUNDA_ETAPA){
        sistemaParacaidas.activate(DURACION_SEGUNDA_ETAPA);
        segundaEtapaActivada = true;
        Serial.print("Desacople activado a ");
        Serial.print(alturaActual);
        Serial.println(" m durante 5 segundos.");
    }
    alturaAnterior = alturaActual;
}

void CubeSat::actualizarDeteccionAterrizaje() {
    if (
        vueloIniciado == false ||
        aterrizajeDetectado == true
    ) {
        return;
    }

    float alturaActual;
    float velocidadVertical;

    alturaActual =
        filtroAltitud.getAltitud();

    velocidadVertical =
        filtroAltitud
            .getVelocidadVertical();

    bool cercaDelPiso;
    bool velocidadCasiCero;

    cercaDelPiso =
        fabs(alturaActual) <=
        ALTURA_CERCA_DEL_PISO;

    velocidadCasiCero =
        fabs(velocidadVertical) <=
        VELOCIDAD_MAXIMA_REPOSO;

    if (
        cercaDelPiso == true &&
        velocidadCasiCero == true
    ) {
        if (inicioReposoMillis == 0) {
            inicioReposoMillis =
                millis();
        }

        if (
            millis() -
            inicioReposoMillis >=
            TIEMPO_CONFIRMACION_ATERRIZAJE
        ) {
            aterrizajeDetectado = true;
            estadoActual = POST_CAIDA;

            sistemaParacaidas.deactivate();

            Serial.println(
                "Aterrizaje confirmado."
            );
        }
    }
    else {
        inicioReposoMillis = 0;
    }
}

void CubeSat::ejecutarPostCaida() {
    unsigned long tiempoActual;

    tiempoActual = millis();

    if (
        tiempoActual -
        ultimoEnvioMillis <
        INTERVALO_POST_CAIDA
    ) {
        return;
    }

    ultimoEnvioMillis =
        tiempoActual;

    moduloGPS.obtenerDatos(
        &datosGPS
    );

    float voltajeMilivoltios;

    voltajeMilivoltios =
        sensores
            .obtenerVoltajeMilivoltios();

    telemetria.crearPaquetePostCaida(
        &paquete,
        voltajeMilivoltios,
        &datosGPS,
        paracaidasHabilitado,
        paracaidasArmado,
        primeraEtapaActivada,
        segundaEtapaActivada,
        aterrizajeDetectado
    );

    enviarPaquete();

    Serial.println(
        "Paquete post-caida enviado."
    );
}