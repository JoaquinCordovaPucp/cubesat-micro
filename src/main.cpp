#include <Arduino.h>
#include "CubeSat.hpp"

CubeSat cubeSat;

void setup() {
    cubeSat.iniciar();
}

void loop() {
    cubeSat.actualizar();
}
