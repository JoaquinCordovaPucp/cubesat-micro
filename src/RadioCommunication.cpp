#include "RadioCommunication.hpp"
#include "Configuracion.hpp"

// Esta variable cambia cuando termina una transmisión
// o cuando llega un paquete.
static volatile bool operacionRadioTerminada = false;


// Esta función es llamada automáticamente por la interrupción.
static void cambiarBanderaRadio() {
    operacionRadioTerminada = true;
}


RadioCommunication::RadioCommunication() :
    radio(
        new Module(
            PIN_LORA_NSS,
            PIN_LORA_DIO0,
            PIN_LORA_RESET,
            PIN_LORA_DIO1
        )
    )
{
    transmitiendo = false;
    comandoDisponible = false;

    ultimoComando = 0;
    ultimoEstado = RADIOLIB_ERR_NONE;
}


void RadioCommunication::iniciar() {
    ultimoEstado = radio.begin(FRECUENCIA_LORA);

    radio.setSpreadingFactor(
        FACTOR_PROPAGACION_LORA
    );

    radio.setBandwidth(
        ANCHO_BANDA_LORA
    );

    if (ultimoEstado == RADIOLIB_ERR_NONE) {
        Serial.println(
            "Modulo LoRa inicializado correctamente."
        );
    }
    else {
        Serial.print(
            "Error al iniciar el modulo LoRa. Codigo: "
        );

        Serial.println(ultimoEstado);

        while (true) {
            delay(10);
        }
    }

    radio.setDio0Action(
        cambiarBanderaRadio,
        RISING
    );
    ultimoEstado =
    radio.startReceive();

    if (
        ultimoEstado !=
        RADIOLIB_ERR_NONE
    ) {
        Serial.print(
            "Error al iniciar recepcion LoRa: "
        );

        Serial.println(
            ultimoEstado
        );
    }
    else {
        Serial.println(
            "LoRa escuchando comandos."
        );
    }
}


void RadioCommunication::actualizar() {
    if (operacionRadioTerminada == false) {
        return;
    }

    operacionRadioTerminada = false;

    if (transmitiendo == true) {
        radio.startReceive();

        transmitiendo = false;

        return;
    }

    uint8_t comandoRecibido;

    ultimoEstado =
        radio.readData(
            &comandoRecibido,
            sizeof(comandoRecibido)
        );

    if (ultimoEstado == RADIOLIB_ERR_NONE) {
        ultimoComando = comandoRecibido;
        comandoDisponible = true;
    }

    radio.startReceive();
}


int RadioCommunication::enviarPaquete(
    uint8_t *datos,
    int cantidadBytes
) {
    ultimoEstado =
        radio.startTransmit(
            datos,
            cantidadBytes
        );
        
    if (ultimoEstado == RADIOLIB_ERR_NONE) {
        transmitiendo = true;
    }

    return ultimoEstado;
}


bool RadioCommunication::hayComando() {
    return comandoDisponible;
}


int RadioCommunication::obtenerComando() {
    int comando;

    comando = ultimoComando;

    ultimoComando = 0;
    comandoDisponible = false;

    return comando;
}


int RadioCommunication::obtenerUltimoEstado() {
    return ultimoEstado;
}
