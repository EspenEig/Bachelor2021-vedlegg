from time import perf_counter


def GetNewMeasurement(robot, measurements):
    # Legg til differansen mellom nulltida og perf_counter() ("nå")
    measurements["time"].append(perf_counter() - measurements["zeroTime"])

    # Her legges de målte verdiene til.
    # Husk å kommenter vekk alle sensorer som ikke blir brukt!

    # Legg til den målte lysverdien
    measurements["light"].append(robot["myColorSensor"].reflection())
    # Legg til den målte fargeverdien
    # measurements["color"].append(robot["myColorSensor"].color())
    # Legg til den målte ambient-/lysintensitetsverdien
    # measurements["ambient"].append(robot["myColorSensor"].ambient())

    # Legg til den målte bryterverdien
    measurements["touch"].append(robot["myTouchSensor"].pressed())

    # Legg til den målte distanseverdien til ultrasonicsensoren
    measurements["distance"].append(robot["myUltrasonicSensor"].distance())

    # Legg til den målte vinkelverdien til gyrosensoren
    measurements["gyroAngle"].append(robot["myGyroSensor"].angle())
    # Legg til den målte farten (angular velocity) til gyrosensoren
    measurements["gyroRate"].append(robot["myGyroSensor"].speed())

    # Legg tild en målte akkumulerte vinkelverdien til motor A
    # motor A tilsvarer altså motoren som er festet til port A
    measurements["motorAangle"].append(robot["motorA"].angle())
    # Legg til den målte farten (angular velocity) til motor A
    measurements["motorAspeed"].append(robot["motorA"].speed())

    # Legg til en målte akkumulerte vinkelverdien til motor B
    # measurements["motorBangle"].append(robot["motorB"].angle())
    # Legg til den målte farten (angular velocity) til motor B
    # measurements["motorBspeed"].append(robot["motorB"].speed())

    # Legg til en målte akkumulerte vinkelverdien til motor C
    # measurements["motorCangle"].append(robot["motorC"].angle())
    # Legg til den målte farten (angular velocity) til motor C
    # measurements["motorCspeed"].append(robot["motorC"].speed())

    # Legg til en målte akkumulerte vinkelverdien til motor D
    # measurements["motorDangle"].append(robot["motorD"].angle())
    # Legg til den målte farten (angular velocity) til motor D
    # measurements["motorDspeed"].append(robot["motorD"].speed())
