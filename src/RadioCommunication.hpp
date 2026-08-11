#ifndef RADIO_COMMUNICATION_HPP
#define RADIO_COMMUNICATION_HPP

#include <Arduino.h>
#include <RadioLib.h>

class RadioCommunication {
private:
    SX1276 radio;

    bool transmitiendo;
    bool comandoDisponible;

    uint8_t ultimoComando;

    int ultimoEstado;

public:
    RadioCommunication();

    void iniciar();
    void actualizar();

    int enviarPaquete(
        uint8_t *datos,
        int cantidadBytes
    );

    bool hayComando();
    int obtenerComando();

    int obtenerUltimoEstado();
};

#endif
