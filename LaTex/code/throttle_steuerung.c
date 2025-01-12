static bool isFailsafeActivated = false;
static uint16_t lastCallTime = 0;
static float baroAltitudeStart = 0;  // Starthöhe beim Failsafe-Eintritt in cm !!!
static uint16_t throttle = 1200;     // Startwert für Throttle

// Konfiguration
#define MIN_THROTTLE 1000
#define MAX_THROTTLE 1400
#define DEFAULT_THROTTLE 1200
#define UPDATE_INTERVAL_MS 10
#define THROTTLE_STEP 5        // Wie viel der Throttle pro Schritt geändert wird

// Berechnet den Failsafe Throttle
uint16_t getFailsafeThrottle(uint32_t currentTime) {
    // Erste Aktivierung des Failsafe
    if (!isFailsafeActivated) {
        isFailsafeActivated = true;
        lastCallTime = currentTime;
        baroAltitudeStart = getBaroAltitude();
        throttle = DEFAULT_THROTTLE;
        return throttle;
    }

    // Nur alle 100ms aktualisieren
    if (currentTime - lastCallTime < UPDATE_INTERVAL_MS) {
        return throttle;
    }

    // Höhenänderung berechnen
    float currentAltitude = getBaroAltitude();
    float altitude_change = currentAltitude - baroAltitudeStart;

    // Throttle anpassen basierend auf Höhenänderung
    if (altitude_change > 0) {         // Steigt
        throttle -= THROTTLE_STEP;
    } else if (altitude_change < -2) { // Sinkt zu schnell
        throttle += THROTTLE_STEP;
    }
    // Sonst: Sinkrate ok, Throttle beibehalten

    // Throttle begrenzen
    if (throttle < MIN_THROTTLE) throttle = MIN_THROTTLE;
    if (throttle > MAX_THROTTLE) throttle = MAX_THROTTLE;

    // Zeit und Höhe für nächsten Vergleich speichern
    lastCallTime = currentTime;
    baroAltitudeStart = currentAltitude;

    return throttle;
}