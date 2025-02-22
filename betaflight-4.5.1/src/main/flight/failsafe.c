/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
//ab hier Import von projektspezifischen Header-Dateien
#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/time.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"

#include "io/beeper.h"

#include "rx/rx.h"

#include "flight/pid.h"

#include "sensors/barometer.h"


//baro_t baro;  // barometer access functions
/*
 * Usage:
 *
 * failsafeInit() and failsafeReset() must be called before the other methods are used.
 *
 * failsafeInit() and failsafeReset() can be called in any order.
 * failsafeInit() should only be called once.
 *
 * enable() should be called after system initialisation.
 */
// Definition des globalen Zustands für das Failsafe-System
static failsafeState_t failsafeState;
// Registrierung einer Konfiguration für das Failsafe-System mit Standardwerten
PG_REGISTER_WITH_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 2);

/*
#ifdef USE_RACE_PRO
#define DEFAULT_FAILSAFE_RECOVERY_DELAY 1            // 100ms of valid rx data needed to allow recovery from failsafe and arming block
#else
*/
// Standardverzögerung für die Wiederherstellung der Steuerung nach einem Failsafe 500 ms
#define DEFAULT_FAILSAFE_RECOVERY_DELAY 5            // 500ms of valid rx data needed to allow recovery from failsafe and arming block
//#endif
// Definition von Standardwerten für die Failsafe-Konfiguration
PG_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig,
    // Hier wird die abstiegsgeschwindigkeit definiert
    .failsafe_throttle = 1200,                           // default throttle off. am anfang waren es 1000
    //.target_altitude = 500,    eher kritisch so                           // Zielhöhe in cm (beispielsweise 5 Meter)
    .failsafe_throttle_low_delay = 100,                  // default throttle low delay for "just disarm" on failsafe condition
    .failsafe_delay = 0,                                // 1.5 sec stage 1 period, can regain control on signal recovery, at idle in drop mode
    .failsafe_off_delay = 100,                            // 1 sec in landing phase, if enabled
    .failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE1, // default failsafe switch action is identical to rc link loss
    .failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT,    // default full failsafe procedure is 0: auto-landing
    .failsafe_recovery_delay = DEFAULT_FAILSAFE_RECOVERY_DELAY,
    .failsafe_stick_threshold = 30                       // 30 percent of stick deflection to exit GPS Rescue procedure
);

/*
// Neue Variable definieren
//static float targetAltitude = 0;  // Zielhöhe in Metern

// Funktion, um Zielhöhe alle 3 Sekunden um 0,5 Meter zu verringern
void updateTargetAltitude(void) {
    static uint32_t lastUpdateTime = 0;
    uint32_t currentTime = millis();  // Aktuelle Zeit in Millisekunden
    if (currentTime - lastUpdateTime >= 3000) {  // Alle 3 Sekunden
        targetAltitude -= 0.5;  // 0,5 Meter absinken
        lastUpdateTime = currentTime;
    }
}

// Funktion, um die Drosselklappe basierend auf Barometerdaten zu korrigieren
void correctThrottleBasedOnAltitude(void) {
    float currentAltitude = baroGetAltitude();  // Aktuelle Höhe aus Barometer
    float altitudeError = targetAltitude - currentAltitude;  // Abweichung

    // Anpassung der Drosselklappe basierend auf der Abweichung
    if (altitudeError > 0.1) {  // Zu niedrig
        failsafeConfig()->failsafe_throttle += 10;  // Drosselklappe erhöhen
    } else if (altitudeError < -0.1) {  // Zu hoch
        failsafeConfig()->failsafe_throttle -= 10;  // Drosselklappe verringern
    }
}

// In der Hauptschleife oder einer passenden Stelle aufrufen
void failsafeUpdateState(void) {
    if (!failsafeIsMonitoring()) {
        return;
    }

    // Update der Zielhöhe
    updateTargetAltitude();

    // Korrektur der Drosselklappe basierend auf der Höhe
    correctThrottleBasedOnAltitude();

}
*/

void failsafeCheckAltitude(void) {
    float altitude = getBaroAltitude();
    if (altitude < 0) {
        // Beispielaktion: Warnung bei negativer Höhe
        // ... (Implementierung)
    }
}

/*
erstellt theoretisch ein array aus zeigern. macht aber eigentlich einfach eine liste wie gewohnt in python, mit der länge [Failsafe_PROCECURE_COUNT]
*/
// Namen für die verschiedenen Failsafe-Prozeduren
const char * const failsafeProcedureNames[FAILSAFE_PROCEDURE_COUNT] = {
    "AUTO-LAND", // automatische Landung
    "DROP", // Sofortiger Abwurf löschen
/*
#ifdef USE_GPS_RESCUE
    "GPS-RESCUE",
#endif*/
};

/*
 * Should called when the failsafe config needs to be changed - e.g. a different profile has been selected.
 * Funktion zum Zurücksetzen des Failsafe-Zustands
 */
void failsafeReset(void)
{
    failsafeState.rxDataFailurePeriod = failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND;
    if (failsafeState.rxDataFailurePeriod < PERIOD_RXDATA_RECOVERY){
        // avoid transients and ensure reliable arming for minimum of PERIOD_RXDATA_RECOVERY (100ms)
        failsafeState.rxDataFailurePeriod = PERIOD_RXDATA_RECOVERY;
    }
    failsafeState.rxDataRecoveryPeriod = failsafeConfig()->failsafe_recovery_delay * MILLIS_PER_TENTH_SECOND;
    if (failsafeState.rxDataRecoveryPeriod < PERIOD_RXDATA_RECOVERY) {
        // PERIOD_RXDATA_RECOVERY (100ms) is the minimum allowed RxData recovery time
        failsafeState.rxDataRecoveryPeriod = PERIOD_RXDATA_RECOVERY;
    }
    failsafeState.validRxDataReceivedAt = 0; // Initialisieren der Zeitstempel
    failsafeState.validRxDataFailedAt = 0;
    failsafeState.throttleLowPeriod = 0;
    failsafeState.landingShouldBeFinishedAt = 0;
    failsafeState.receivingRxDataPeriod = 0;
    failsafeState.receivingRxDataPeriodPreset = failsafeState.rxDataRecoveryPeriod;
    failsafeState.phase = FAILSAFE_IDLE; // Standard
    failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN; // Verbindungsstatus: Getrennt
    failsafeState.boxFailsafeSwitchWasOn = false;  // Failsafe-Schalter initial deaktiviert
}

// Initialisiert das Failsafe-System
void failsafeInit(void)
{
    failsafeState.events = 0; // Setzt die Ereigniszähler zurück
    failsafeState.monitoring = false; // Überwachung ist initial deaktiviert

    return;
}
// Gibt die aktuelle Failsafe-Phase zurück
failsafePhase_e failsafePhase(void)
{
    return failsafeState.phase;
}

// Überprüft, ob die Überwachung aktiv ist
bool failsafeIsMonitoring(void)
{
    return failsafeState.monitoring;
}

// Prüft, ob Failsafe derzeit aktiv ist
bool failsafeIsActive(void) // real or BOXFAILSAFE induced stage 2 failsafe is currently active
{
    return failsafeState.active;
}

// Startet die Überwachung der Failsafe-Bedingungen
void failsafeStartMonitoring(void)
{
    failsafeState.monitoring = true;
}

// Prüft, ob die Landung abgeschlossen sein sollte
static bool failsafeShouldHaveCausedLandingByNow(void)
{
    return (millis() > failsafeState.landingShouldBeFinishedAt);
}

// Überprüft, ob Daten empfangen werden
bool failsafeIsReceivingRxData(void)
{
    return (failsafeState.rxLinkState == FAILSAFE_RXLINK_UP);
    // Falsch bei aktivem BOXFAILSAFE-Schalter oder wenn 100 ms lang keine gültigen Pakete empfangen wurden  
    // oder ein beliebiger Flugkanal 300 ms lang ungültig ist.  
    // Wird sofort wahr, wenn der BOXFAILSAFE-Schalter deaktiviert wird oder nach Ablauf der Wiederherstellungszeit,  
    // sobald gültige Pakete empfangen werden.  
    // rxLinkState RXLINK_DOWN (nicht UP) ist der Auslöser für die verschiedenen Stufe-2-Ergebnisse der Failsafe.  
}

// Handhabung von Unterbrechungen beim Empfang
void failsafeOnRxSuspend(uint32_t usSuspendPeriod)
{
    failsafeState.validRxDataReceivedAt += (usSuspendPeriod / 1000);    // / 1000 to convert micros to millis
}

// Handhabung der Wiederaufnahme des Empfangs
void failsafeOnRxResume(void)
{
    failsafeState.validRxDataReceivedAt = millis();                     // prevent RX link down trigger, restart rx link up, zeitstempel auf aktualisieren
    failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;                     // do so while rx link is up, status = verbunden
}

void failsafeOnValidDataReceived(void)
// runs, after prior a signal loss, immediately when packets are received or the BOXFAILSAFE switch is reverted
// rxLinkState will go RXLINK_UP immediately if BOXFAILSAFE goes back ON since receivingRxDataPeriodPreset is set to zero in that case
// otherwise RXLINK_UP is delayed for the recovery period (failsafe_recovery_delay, default 500ms, 1-20, min 0.1s)
{
    failsafeState.validRxDataReceivedAt = millis();

    if (failsafeState.validRxDataFailedAt == 0) {
        // after initialisation, we sometimes only receive valid packets, so validRxDataFailedAt will remain unset (0)
        // in this setting, the time the signal is first valid is also time it was last valid, so
        // initialise validRxDataFailedAt to the time of the first valid data
        failsafeState.validRxDataFailedAt = failsafeState.validRxDataReceivedAt;
        // prevent arming until we have valid data for rxDataRecoveryPeriod after initialisation
        // show RXLOSS in OSD to indicate reason we cannot arm
        setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
    }

    if (cmp32(failsafeState.validRxDataReceivedAt, failsafeState.validRxDataFailedAt) > (int32_t)failsafeState.receivingRxDataPeriodPreset) {
        // receivingRxDataPeriodPreset is rxDataRecoveryPeriod unless set to zero to allow immediate control recovery after switch induced failsafe
        // rxDataRecoveryPeriod defaults to 1.0s with minimum of PERIOD_RXDATA_RECOVERY (200ms)
        // link is not considered 'up', after it has been 'down', until that recovery period has expired
        failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;
        // after the rxDataRecoveryPeriod, typically 1s after receiving valid data, clear RXLOSS in OSD and permit arming
        unsetArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
    }
}

void failsafeOnValidDataFailed(void)
// run from rc.c when packets are lost for more than the signal validation period (100ms), or immediately BOXFAILSAFE switch is active
// after the stage 1 delay has expired, sets the rxLinkState to RXLINK_DOWN, ie not up, causing failsafeIsReceivingRxData to become false
// if failsafe is configured to go direct to stage 2, this is emulated immediately in failsafeUpdateState()
{
    //  set RXLOSS in OSD and block arming after 100ms of signal loss
    setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);

    failsafeState.validRxDataFailedAt = millis();
    if ((cmp32(failsafeState.validRxDataFailedAt, failsafeState.validRxDataReceivedAt) > (int32_t)failsafeState.rxDataFailurePeriod)) {
        // sets rxLinkState = DOWN to initiate stage 2 failsafe after expiry of the Stage 1 period
        failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
        // show RXLOSS and block arming
    }
}

// Überprüft periodisch, ob der Empfang fehlschlägt
void failsafeCheckDataFailurePeriod(void)
// runs directly from scheduler, every 10ms, to validate the link
{
    if (cmp32(millis(), failsafeState.validRxDataReceivedAt) > (int32_t)failsafeState.rxDataFailurePeriod) {
        // sets link DOWN after the stage 1 failsafe period, initiating stage 2
        failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
        // Prevent arming with no RX link
        setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
    }
}

// Gibt die Failsafe-Fehlerperiode zurück
uint32_t failsafeFailurePeriodMs(void)
{
    return failsafeState.rxDataFailurePeriod;
}

// Hauptlogik zur Aktualisierung des Failsafe-Zustands
FAST_CODE_NOINLINE void failsafeUpdateState(void)
// direkt und NUR vom Scheduler ausgelöst, bei 10 ms = PERIOD_RXDATA_FAILURE - Intervalle
{
    if (!failsafeIsMonitoring()) {
        return;
    }

    bool receivingRxData = failsafeIsReceivingRxData();
    // Gibt den Status von FAILSAFE_RXLINK_UP zurück, der  
    // nach der Verzögerung von Stufe 1 auf falsch wechselt, sei es durch Signalverlust oder Aktivierung des !!!BOXFAILSAFE-Schalters!!!.  
    // Wechselt sofort auf wahr, wenn der BOXFAILSAFE-Schalter deaktiviert wird oder nach der Wiederherstellungsverzögerung, sobald das Signal zurückkehrt.  
    // Bedeutet im Wesentlichen 'sollte sich in Failsafe-Stufe 2 befinden'.  


    DEBUG_SET(DEBUG_FAILSAFE, 2, receivingRxData); // from Rx alone, not considering switch

    // armed sagt nun welchen zustand den arming flag hat, ture or false. am besten ture ansonsten fällt sie auf den boden i guess 
    bool armed = ARMING_FLAG(ARMED);
    // beeperMode_e beeperMode = BEEPER_SILENCE;

    if (IS_RC_MODE_ACTIVE(BOXFAILSAFE) && (failsafeConfig()->failsafe_switch_mode == FAILSAFE_SWITCH_MODE_STAGE2)) {
        // Force immediate stage 2 responses if mode is failsafe stage2 to emulate immediate loss of signal without waiting
        receivingRxData = false;
    }

    // Beep RX lost only if we are not seeing data and are armed or have been armed earlier
    /*
    if (!receivingRxData && (armed || ARMING_FLAG(WAS_EVER_ARMED))) {
        beeperMode = BEEPER_RX_LOST;
    }
    */
    bool reprocessState;

    do {
        reprocessState = false;

        switch (failsafeState.phase) {  
            case FAILSAFE_IDLE:     // IDLE bedeutet inaktiv (NORMALBETRIEB), aber das untere versteht man noch nicht ganz
                failsafeState.boxFailsafeSwitchWasOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
                // store and use the switch state as it was at the start of the failsafe
                if (armed) {
                    // Track throttle command below minimum time
                    if (calculateThrottleStatus() != THROTTLE_LOW) {
                        failsafeState.throttleLowPeriod = millis() + failsafeConfig()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                    }
                    if (failsafeState.boxFailsafeSwitchWasOn && (failsafeConfig()->failsafe_switch_mode == FAILSAFE_SWITCH_MODE_KILL)) {
                        // Failsafe switch is configured as KILL switch and is switched ON
                        failsafeState.active = true;
                        failsafeState.events++;
                        ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                        failsafeState.phase = FAILSAFE_LANDED;
                        //  go to landed immediately
                        failsafeState.receivingRxDataPeriodPreset = failsafeState.rxDataRecoveryPeriod;
                        //  allow re-arming 1 second after Rx recovery, customisable
                        reprocessState = true;
                    } else if (!receivingRxData) {
                        if (millis() > failsafeState.throttleLowPeriod
/*#ifdef USE_GPS_RESCUE
                            && failsafeConfig()->failsafe_procedure != FAILSAFE_PROCEDURE_GPS_RESCUE
#endif*/
                            ) {
                            //  JustDisarm if throttle was LOW for at least 'failsafe_throttle_low_delay' before failsafe
                            //  protects against false arming when the Tx is powered up after the quad
                            failsafeState.active = true;
                            failsafeState.events++;
                            ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                            failsafeState.phase = FAILSAFE_LANDED;
                            //  go directly to FAILSAFE_LANDED
                            failsafeState.receivingRxDataPeriodPreset = failsafeState.rxDataRecoveryPeriod;
                            //  allow re-arming 1 second after Rx recovery, customisable
                        } else {
                            failsafeState.phase = FAILSAFE_RX_LOSS_DETECTED;
                        }
                        reprocessState = true;
                    }
                } else {
                    // When NOT armed, enable failsafe mode to show warnings in OSD
                    if (failsafeState.boxFailsafeSwitchWasOn) {
                        ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    } else {
                        DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    }
                    // Throttle low period expired (= low long enough for JustDisarm)
                    failsafeState.throttleLowPeriod = 0;
                }
                break;

            case FAILSAFE_RX_LOSS_DETECTED:
                if (receivingRxData) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                } else {
                    failsafeState.active = true;
                    failsafeState.events++;
                    switch (failsafeConfig()->failsafe_procedure) {
                        case FAILSAFE_PROCEDURE_AUTO_LANDING:
                            //  Enter Stage 2 with settings for landing mode
                            ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                            failsafeState.phase = FAILSAFE_LANDING;
                            failsafeState.landingShouldBeFinishedAt = millis() + failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;
                            break;
/*
                        case FAILSAFE_PROCEDURE_DROP_IT: // löschen
                            ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                            failsafeState.phase = FAILSAFE_LANDED;
                            //  go directly to FAILSAFE_LANDED
                            break; */
/*#ifdef USE_GPS_RESCUE
                        case FAILSAFE_PROCEDURE_GPS_RESCUE: //löschen
                            ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
                            failsafeState.phase = FAILSAFE_GPS_RESCUE;
                            break;
#endif
*/
                    }
                    if (failsafeState.boxFailsafeSwitchWasOn) {
                        failsafeState.receivingRxDataPeriodPreset = 0;
                        // recover immediately if failsafe was triggered by a switch
                    } else {
                        failsafeState.receivingRxDataPeriodPreset = failsafeState.rxDataRecoveryPeriod;
                        // recover from true link loss failsafe 1 second after RC Link recovers
                    }
                }
                reprocessState = true;
                break;

            case FAILSAFE_LANDING:
                if (receivingRxData) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                    reprocessState = true;
                } else {
                    /*
                    if (armed) {
                        beeperMode = BEEPER_RX_LOST_LANDING;
                    }*/
                    
                    if (failsafeShouldHaveCausedLandingByNow() || crashRecoveryModeActive() || !armed) {
                        // to manually disarm while Landing, aux channels must be enabled
                        // note also that disarming via arm box must be possible during failsafe in rc_controls.c
                        // this should be blocked during signal not received periods, to avoid false disarms
                        // but should be allowed otherwise, eg after signal recovers, or during switch initiated failsafe
                        failsafeState.phase = FAILSAFE_LANDED;
                        reprocessState = true;
                    }
                }
                break;
/*#ifdef USE_GPS_RESCUE
            case FAILSAFE_GPS_RESCUE:
                if (receivingRxData) {
                    if (areSticksActive(failsafeConfig()->failsafe_stick_threshold) || failsafeState.boxFailsafeSwitchWasOn) {
                        // exits the rescue immediately if failsafe was initiated by switch, otherwise 
                        // requires stick input to exit the rescue after a true Rx loss failsafe
                        // NB this test requires stick inputs to be received during GPS Rescue see PR #7936 for rationale
                        failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                        reprocessState = true;
                    }
                } else {
                    if (armed) {
                        beeperMode = BEEPER_RX_LOST_LANDING;
                    } else {
                        // to manually disarm while in GPS Rescue, aux channels must be enabled
                        failsafeState.phase = FAILSAFE_LANDED;
                        reprocessState = true;
                    }
                }
                break;
#endif
*/
            case FAILSAFE_LANDED:
                disarm(DISARM_REASON_FAILSAFE);
                setArmingDisabled(ARMING_DISABLED_FAILSAFE);
                //  prevent accidently rearming by an intermittent rx link
                failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset;
                //  customise receivingRxDataPeriod according to type of failsafe
                failsafeState.phase = FAILSAFE_RX_LOSS_MONITORING;
                reprocessState = true;
                break;

            case FAILSAFE_RX_LOSS_MONITORING:
                // receivingRxData is true when we get valid Rx Data and the recovery period has expired
                // for switch initiated failsafes, the recovery period is zero
                if (receivingRxData) {
                    if (millis() > failsafeState.receivingRxDataPeriod) {
                        // rx link is good now
                        failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                        reprocessState = true;
                    }
                } else {
                    failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset;
                }
                break;

            case FAILSAFE_RX_LOSS_RECOVERED:
                // Entering IDLE, terminating failsafe, reset throttle low timer
                failsafeState.throttleLowPeriod = millis() + failsafeConfig()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                failsafeState.phase = FAILSAFE_IDLE;
                failsafeState.active = false;
/*#ifdef USE_GPS_RESCUE
                DISABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
#endif
*/
                DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                unsetArmingDisabled(ARMING_DISABLED_FAILSAFE);
                reprocessState = true;
                break;

            default:
                break;
        }

    DEBUG_SET(DEBUG_FAILSAFE, 0, failsafeState.boxFailsafeSwitchWasOn);
    DEBUG_SET(DEBUG_FAILSAFE, 3, failsafeState.phase);

    } while (reprocessState);
 
    /*
    if (beeperMode != BEEPER_SILENCE) {
        beeper(beeperMode);
    }
    */
}
