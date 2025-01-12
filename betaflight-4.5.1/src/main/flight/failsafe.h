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
 * You should have received a copy of the GNU General Public Licegit nse
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "pg/pg.h"

#define FAILSAFE_POWER_ON_DELAY_US (1000 * 1000 * 5)
#define MILLIS_PER_TENTH_SECOND      100
#define MILLIS_PER_SECOND           1000
#define PERIOD_OF_1_SECONDS            1 * MILLIS_PER_SECOND
#define PERIOD_OF_3_SECONDS            3 * MILLIS_PER_SECOND
#define PERIOD_OF_30_SECONDS          30 * MILLIS_PER_SECOND
#define PERIOD_RXDATA_FAILURE        10     // millis
#define PERIOD_RXDATA_RECOVERY       100    // millis

/*macht eine struktur mit den wichtigsten variabeln, zwar noch ohne inhalt aber den datei typen. 
unit16_t = unsigned int bis 2^16; die struktur wird aufgerufen unter failsafeConfig_t
*/

typedef struct failsafeConfig_s {
    uint16_t failsafe_throttle;             // Gashebelstufe für die Landung - Wert zwischen 1000..2000 angeben (PWM-Pulsbreite für leicht unterhalb des Schwebeflugs). Mittelstellung Gashebel = 1500.
    uint16_t failsafe_throttle_low_delay;   // Zeit, die der Gashebel unter 'min_check' gewesen sein muss, um "NurEntsperren" anstelle des "vollständigen Failsafe-Verfahrens" auszuführen.
    uint8_t failsafe_delay;                 // Schutzzeit für die Failsafe-Aktivierung nach Signalverlust. 1 Schritt = 0,1 Sek. - 1 Sek. im Beispiel (10).
    uint8_t failsafe_off_delay;             // Zeit für die Landung, bevor die Motoren in 0,1 Sek. stoppen. 1 Schritt = 0,1 Sek. - 20 Sek. im Beispiel (200).
    uint8_t failsafe_switch_mode;           // Failsafe-Schalteraktion ist 0: Stufe 1, 1: Schaltet sofort ab, 2: Stufe 2.
    uint8_t failsafe_procedure;             // Ausgewähltes vollständiges Failsafe-Verfahren ist 0: Automatische Landung, 1: Fallen lassen.
    uint16_t failsafe_recovery_delay;       // Zeit (in 0,1 Sek.) mit gültigen RX-Daten (mindestens 100ms PERIOD_RXDATA_RECOVERY), um das Wiederherstellen aus dem Failsafe-Verfahren zu ermöglichen.
    uint8_t failsafe_stick_threshold;       // Steuerknüppelabweichung in Prozent, um das GPS-Rettungsverfahren zu beenden
} failsafeConfig_t;

PG_DECLARE(failsafeConfig_t, failsafeConfig);

/*
Eine Enumeration in C ist ein benutzerdefinierter Datentyp, der eine Sammlung von benannten Ganzzahlen enthält. Sie wird verwendet, um eine Liste von Konstanten zu definieren,
die jeweils mit einem Namen versehen sind, wodurch der Code lesbarer und verständlicher wird.Durch das typedef-Schlüsselwort wird der Enum-Typ benannt (failsafePhase_e), 
sodass er wie ein normaler Datentyp verwendet werden kann (ähnlich wie int oder float).

Nach der Definition kann der Typ failsafePhase_e verwendet werden, um Variablen zu deklarieren, die nur einen der definierten Zustände annehmen können.*/
typedef enum {
    FAILSAFE_IDLE = 0,
    FAILSAFE_RX_LOSS_DETECTED,
    FAILSAFE_LANDING,
    FAILSAFE_LANDED,
    FAILSAFE_RX_LOSS_MONITORING,
    FAILSAFE_RX_LOSS_RECOVERED,
    FAILSAFE_GPS_RESCUE
} failsafePhase_e;

typedef enum {
    FAILSAFE_RXLINK_DOWN = 0,
    FAILSAFE_RXLINK_UP
} failsafeRxLinkState_e;

typedef enum {
    FAILSAFE_PROCEDURE_AUTO_LANDING = 0,
    FAILSAFE_PROCEDURE_DROP_IT,

/*
#ifdef USE_GPS_RESCUE
    FAILSAFE_PROCEDURE_GPS_RESCUE,


//!!!!!Sehr Fragwürdig !!!!!!
#endif */
    FAILSAFE_PROCEDURE_COUNT   // must be last

} failsafeProcedure_e;

/*
extern: Der Compiler weiß, dass diese Variable existiert, aber die Definition der Werte kommt später, irgendwo anders im Code.
char * bedeutet, dass es sich um einen Zeiger auf einen String handelt.
failsafeProcedureNames ist der Name des Arrays, das du deklariert hast.
Das Array enthält mehrere Zeiger auf Strings.
FAILSAFE_PROCEDURE_COUNT ist eine Zahl, die dir sagt, wie viele Einträge (Strings) dieses Array haben wird.
*/
extern const char * const failsafeProcedureNames[FAILSAFE_PROCEDURE_COUNT];

typedef enum {
    FAILSAFE_SWITCH_MODE_STAGE1 = 0,
    FAILSAFE_SWITCH_MODE_KILL,
    FAILSAFE_SWITCH_MODE_STAGE2
} failsafeSwitchMode_e;

typedef struct failsafeState_s {
    int16_t events;
    bool monitoring;
    bool active;
    uint32_t rxDataFailurePeriod;
    uint32_t rxDataRecoveryPeriod;
    uint32_t validRxDataReceivedAt;
    uint32_t validRxDataFailedAt;
    uint32_t throttleLowPeriod;             // throttle stick must have been below 'min_check' for this period
    uint32_t landingShouldBeFinishedAt;
    uint32_t receivingRxDataPeriod;         // period for the required period of valid rxData
    uint32_t receivingRxDataPeriodPreset;   // preset for the required period of valid rxData
    failsafePhase_e phase;
    failsafeRxLinkState_e rxLinkState;
    bool boxFailsafeSwitchWasOn;
} failsafeState_t;

void failsafeInit(void);
void failsafeReset(void);

void failsafeStartMonitoring(void);
void failsafeUpdateState(void);
void failsafeCheckDataFailurePeriod(void);

failsafePhase_e failsafePhase(void);
bool failsafeIsMonitoring(void);
bool failsafeIsActive(void);
bool failsafeIsReceivingRxData(void);
void failsafeOnRxSuspend(uint32_t suspendPeriod);
void failsafeOnRxResume(void);
void failsafeOnValidDataReceived(void);
void failsafeOnValidDataFailed(void);
