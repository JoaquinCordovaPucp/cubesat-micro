#include "GPSModule.hpp"
#include "Configuracion.hpp"

GPSModule::GPSModule() :
    gpsSerial(PUERTO_SERIAL_GPS) {
}

void GPSModule::iniciar() {
    gpsSerial.begin(
        VELOCIDAD_GPS,
        SERIAL_8N1,
        PIN_GPS_RX,
        PIN_GPS_TX
    );
}

void GPSModule::actualizar() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
}

void GPSModule::obtenerDatos(DatosGPS *datos) {
    datos->ubicacionValida =
        gps.location.isValid();

    datos->velocidadValida =
        gps.speed.isValid();

    if (datos->ubicacionValida == true) {
        datos->latitud =
            gps.location.lat();

        datos->longitud =
            gps.location.lng();
    }
    else {
        datos->latitud = 0.0f;
        datos->longitud = 0.0f;
    }

    if (datos->velocidadValida == true) {
        datos->velocidadHorizontal =
            gps.speed.mps();
    }
    else {
        datos->velocidadHorizontal = 0.0f;
    }
}
