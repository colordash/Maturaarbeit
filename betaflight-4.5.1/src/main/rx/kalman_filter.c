#include "kalman_filter.h"
// !!!!!!! BAROMETER !!!!!!!!!!!!!
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp085.h"
#include "drivers/barometer/barometer_bmp280.h"
#include "drivers/barometer/barometer_bmp388.h"
#include "drivers/barometer/barometer_dps310.h"
#include "drivers/barometer/barometer_qmp6988.h"
#include "drivers/barometer/barometer_virtual.h"
#include "drivers/barometer/barometer_ms5611.h"
#include "drivers/barometer/barometer_lps.h"
#include "drivers/barometer/barometer_2smpb_02b.h"
#include "drivers/barometer/barometer_lps22df.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "telemetry/ibus_shared.h"
#include <math.h>  // Für mathematische Operationen wie Multiplikation
#include "rx/rx.h"
#include "sensors/acceleration.h"

// von acceleration.c-----------------------

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/utils.h"

#include "config/feature.h"

#include "sensors/acceleration_init.h"
#include "sensors/boardalignment.h"


/*
void updateBarometerVariance(float new_baro_measurement, KalmanFilter* kf) {
    static float baro_samples[NUM_BARO_SAMPLES] = {0};  // Proben des Barometers
    static int baro_index = 0;  // Index für die aktuelle Probe

    // Alte Probe entfernen und neuen Wert einfügen
    float old_value = baro_samples[baro_index];
    baro_samples[baro_index] = new_baro_measurement;
    baro_index = (baro_index + 1) % NUM_BARO_SAMPLES;  // Zirkulärer Index

    static float mean = 0.0f;      // Laufender Mittelwert
    static float variance = 0.0f;  // Laufende Varianz

    // Mittelwert aktualisieren (Laufmittelwert)
    float new_mean = mean + (new_baro_measurement - old_value) / NUM_BARO_SAMPLES;

    // Varianz aktualisieren
    variance += ((new_baro_measurement - mean) * (new_baro_measurement - new_mean) - 
                 (old_value - mean) * (old_value - new_mean)) / NUM_BARO_SAMPLES;

    mean = new_mean;  // Mittelwert aktualisieren

    // Begrenze die Varianz (in einem sinnvollen Bereich)
    kf->R = fmaxf(fminf(variance, R_MAX), R_MIN);  // Begrenzung der Varianz auf Min/Max-Bereich
}*/


int16_t getACC_z(uint32_t currentTimeMs)
{
    accUpdate(currentTimeMs); 
    return (int16_t)((acc.accADC[2] * acc.dev.acc_1G_rec) * 1000);
}

void initKalmanFilter(KalmanFilter* kf, float initial_height, unsigned long currentTimeMs) {
    // Initialisierung der Zustandsvariablen
    kf->x[0] = initial_height /100.0f;  // Position (Höhe) cm
    kf->x[1] = 0.0f;           // Geschwindigkeit
    
    // Initiale Unsicherheit
    kf->P[0][0] = 100.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 100.0f;
    
    // Prozessrauschen
    kf->Q[0][0] = 0.1f;
    kf->Q[0][1] = 0.0f;
    kf->Q[1][0] = 0.0f;
    kf->Q[1][1] = 0.1f;
    
    // Messrauschen
    kf->R = 5.0f;
    
    // Initialer Zeitschritt
    kf->dt = (1.0f/UPDATE_INTERVAL_MS);  // Startwert
    kf->lastTimeMs = currentTimeMs;
}

float updateHeight(KalmanFilter* kf, unsigned long currentTimeMs, float dt) {
    // Aktualisiere dt basierend auf aktueller Zeit
    kf->dt = dt; 
    
    // Beispielwerte für Beschleunigung und Barometerhöhe
    int16_t acc_z = getACC_z(currentTimeMs) * 9.81f / 1000.0f;  // m/s²
    float baro_height = getBaroAltitude() / 100.0f;  // Meter
    //updateBarometerVariance(baro_height, kf); 
    
    // Prädiktion
    float x_pred[2];
    x_pred[0] = kf->x[0] + kf->x[1] * kf->dt + 0.5f * acc_z * kf->dt * kf->dt;
    x_pred[1] = kf->x[1] + acc_z * kf->dt;
    
    // Prädiktion der Kovarianzmatrix
    float P_pred[2][2];
    P_pred[0][0] = kf->P[0][0] + kf->dt * (kf->P[1][0] + kf->P[0][1] + kf->dt * kf->P[1][1]) + kf->Q[0][0];
    P_pred[0][1] = kf->P[0][1] + kf->dt * kf->P[1][1] + kf->Q[0][1];
    P_pred[1][0] = kf->P[1][0] + kf->dt * kf->P[1][1] + kf->Q[1][0];
    P_pred[1][1] = kf->P[1][1] + kf->Q[1][1];
    
    // Kalman-Gewinn berechnen
    float K[2];
    K[0] = P_pred[0][0] / (P_pred[0][0] + kf->R);
    K[1] = P_pred[1][0] / (P_pred[0][0] + kf->R);
    
    // Update mit Barometerdaten
    kf->x[0] = x_pred[0] + K[0] * (baro_height - x_pred[0]);
    kf->x[1] = x_pred[1] + K[1] * (baro_height - x_pred[0]);
    
    // Update der Kovarianzmatrix
    kf->P[0][0] = (1 - K[0]) * P_pred[0][0];
    kf->P[0][1] = (1 - K[0]) * P_pred[0][1];
    kf->P[1][0] = -K[1] * P_pred[0][0] + P_pred[1][0];
    kf->P[1][1] = -K[1] * P_pred[0][1] + P_pred[1][1];
    
    // Rückgabe der gefilterten Höhe in cm
    return kf->x[0] * 100.0f;
}
