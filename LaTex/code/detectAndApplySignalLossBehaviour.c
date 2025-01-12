Funktion detectAndApplySignalLossBehaviour(void):
	currentTime = aktuelle Zeit in Millisekunden
	failsafeSchalterAktiv = istFailsafeSchalterAktiv()

	rxKanäleValid = signalEmpfangen UND NICHT failsafeSchalterAktiv

	Für jeden Kanal:
		sample = aktueller RC-Wert
		kanalValid = rxKanäleValid UND impulsGültig(sample)

		Wenn kanalValid:
			validTimeout[channel] = currentTime + MAX_INVALID_PULSE_TIME
		Sonst wenn failsafe aktiv:
			Wenn Kanal ein Steuerkanal:
				Wenn ungültig UND Kanal == Gas:
					sample = failsafeThrottleWert
				Sonst:
					sample = midrcWert
		sample = begrenze(sample, MIN_PWM, MAX_PWM)
		setzeRCWert(channel, sample)
