#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
// Skalierungsfaktor
#define R_SCALE_FACTOR 1.0f
#define R_MIN 0.01f
#define R_MAX 10.0f
#define NUM_BARO_SAMPLES 10

#include <stdint.h>  // Für uint32_t, int16_t
#include <stdbool.h> // Für bool, true, false

// Struktur für den Kalman-Filter
typedef struct {
    float x[2];       // Zustandsvektor: [Position, Geschwindigkeit]
    float P[2][2];    // Kovarianzmatrix
    float Q[2][2];    // Prozessrauschen
    float R;          // Messrauschen
    float dt;         // Zeitschritt
    unsigned long lastTimeMs; // Letzte Zeit in Millisekunden
} KalmanFilter;

// Funktionen des Kalman-Filters
void initKalmanFilter(KalmanFilter* kf, float initial_height, unsigned long currentTimeMs);
float updateHeight(KalmanFilter* kf, unsigned long currentTimeMs, float dt);

#endif // KALMAN_FILTER_H
