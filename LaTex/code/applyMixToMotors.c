Funktion applyMixToMotors(motorMix, activeMixer):
    Für jeden Motor (i von 0 bis motorAnzahl):
        motorOutput = (Mixwert für Motor * Vorzeichen) + (Schub * Mixwert im Mixer)
        motorOutput = minMotorWert + (Motorbereich * motorOutput)
        Wenn Failsafe aktiv:
            Falls Protokoll DSHOT:
                Wenn motorOutput unterhalb minimalem Bereich:
                    motorOutput = disarmedWert
            motorOutput = begrenze(motorOutput, disarmedWert, maxMotorWert)
        Sonst:
            motorOutput = begrenze(motorOutput, minMotorWert, maxMotorWert)

        Setze Motorwert[i] = motorOutput

    Falls nicht aktiviert (disarmed):
        Für jeden Motor:
            Setze Motorwert[i] = disarmedWert
