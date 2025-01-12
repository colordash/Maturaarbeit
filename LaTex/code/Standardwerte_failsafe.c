// Definition von Standardwerten für die Failsafe-Konfiguration
PG\_RESET\_TEMPLATE(failsafeConfig\_t, failsafeConfig,
// Hier wird die abstiegsgeschwindigkeit definiert
.failsafe\_throttle = 1200,                                     // default throttle off. am anfang waren es 1000
//.target\_altitude = 500,    eher kritisch so                  // Zielhöhe in cm (beispielsweise 5 Meter)
.failsafe\_throttle\_low\_delay = 100,                          // default throttle low delay for "just disarm" on failsafe condition
.failsafe\_delay = 0,                                           // 1.5 sec stage 1 period, can regain control on signal recovery, at idle in drop mode
.failsafe\_off\_delay = 50,                                     // 1 sec in landing phase, if enabled
.failsafe\_switch\_mode = FAILSAFE\_SWITCH\_MODE\_STAGE1,       // default failsafe switch action is identical to rc link loss
.failsafe\_procedure = FAILSAFE\_PROCEDURE\_DROP\_IT,           // default full failsafe procedure is 0: auto-landing
.failsafe\_recovery\_delay = DEFAULT\_FAILSAFE\_RECOVERY\_DELAY,
.failsafe\_stick\_threshold = 30                                // 30 percent of stick deflection to exit GPS Rescue procedure
);