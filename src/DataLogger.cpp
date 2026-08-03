#include "DataLogger.hpp"
#include "Configuracion.hpp"

DataLogger::DataLogger() {
    disponible = false;

    ultimoFlushMillis = 0;
}

bool DataLogger::iniciar(int pinCS) {
    Serial.println(
        "Iniciando MicroSD..."
    );

    if (
        SD.begin(
            pinCS,
            SPI
        ) == false
    ) {
        Serial.println(
            "No se pudo iniciar la MicroSD."
        );

        disponible = false;

        return false;
    }

    archivo = SD.open(
        "/telemetria.csv",
        FILE_APPEND
    );

    if (archivo == false) {
        Serial.println(
            "No se pudo abrir telemetria.csv."
        );

        disponible = false;

        return false;
    }

    disponible = true;

    if (archivo.size() == 0) {
        escribirCabecera();
    }

    ultimoFlushMillis = millis();

    Serial.println(
        "MicroSD inicializada."
    );

    return true;
}

void DataLogger::escribirCabecera() {
    archivo.println(
        "TYPE,SEQ,TIME,FLAGS,VOLT,"
        "PITCH,ROLL,LON,LAT,VVEL,"
        "PRES,TEMP,ECO2,ETOH,AQI,UV,"
        "GYRX,GYRY,GYRZ,"
        "ACCX,ACCY,ACCZ,ALT,CHK"
    );

    archivo.flush();
}

void DataLogger::guardarPaquete(
    TelemetryPacket *paquete
) {
    if (disponible == false) {
        return;
    }

    archivo.print(
        (int)paquete->TYPE
    );
    archivo.print(",");

    archivo.print(paquete->SEQ);
    archivo.print(",");

    archivo.print(paquete->TIME);
    archivo.print(",");

    archivo.print(paquete->FLAGS);
    archivo.print(",");

    archivo.print(paquete->VOLT);
    archivo.print(",");

    archivo.print(paquete->PITCH);
    archivo.print(",");

    archivo.print(paquete->ROLL);
    archivo.print(",");

    archivo.print(paquete->LON);
    archivo.print(",");

    archivo.print(paquete->LAT);
    archivo.print(",");

    archivo.print(paquete->VVEL);
    archivo.print(",");

    archivo.print(paquete->PRES);
    archivo.print(",");

    archivo.print(paquete->TEMP);
    archivo.print(",");

    archivo.print(paquete->ECO2);
    archivo.print(",");

    archivo.print(paquete->ETOH);
    archivo.print(",");

    archivo.print(
        (int)paquete->AQI
    );
    archivo.print(",");

    archivo.print(paquete->UV);
    archivo.print(",");

    archivo.print(paquete->GYRX);
    archivo.print(",");

    archivo.print(paquete->GYRY);
    archivo.print(",");

    archivo.print(paquete->GYRZ);
    archivo.print(",");

    archivo.print(paquete->ACCX);
    archivo.print(",");

    archivo.print(paquete->ACCY);
    archivo.print(",");

    archivo.print(paquete->ACCZ);
    archivo.print(",");

    archivo.print(paquete->ALT);
    archivo.print(",");

    archivo.println(paquete->CHK);
}

void DataLogger::actualizar() {
    if (disponible == false) {
        return;
    }

    if (
        millis() -
        ultimoFlushMillis >=
        INTERVALO_FLUSH_SD
    ) {
        archivo.flush();

        ultimoFlushMillis = millis();
    }
}

bool DataLogger::estaDisponible() {
    return disponible;
}