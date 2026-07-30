#ifndef RADIO_COMMUNICATION_HPP
#define RADIO_COMMUNICATION_HPP

#include <Arduino.h>
#include <RadioLib.h>

class RadioCommunication {
private:
    SX1276 radio;

    bool transmitiendo;
    bool mensajeDisponible;

    String ultimoMensaje;

    int ultimoEstado;

public:
    RadioCommunication();

    void iniciar();
    void actualizar();

    int enviarPaquete(
        uint8_t *datos,
        int cantidadBytes
    );

    bool hayMensaje();
    String obtenerMensaje();

    int obtenerUltimoEstado();
};

#endif
