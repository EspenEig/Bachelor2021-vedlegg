from time import perf_counter


def GetFirstMeasurement(robot):
    # Dictionary for å holde styr på alle de ulike målingene
    measurements = {}

    # Hver måling (og tider) blir lagret i ei liste.
    # Disse listene blir så lagt til i measurements for fremtidig bruk.

    # Tid.
    # Dette blir nulltida,
    # og alle fremtidige tidsmålinger blir satt i forhold til denne
    zeroTime = perf_counter()
    measurements["zeroTime"] = zeroTime
    time = [0]  # tida vil bli gitt i forhold til zeroTime
    measurements["time"] = time
    ts = []  # tidsskritt
    measurements["ts"] = ts

    # Sensorer
    # Kommenter vekk de som ikke skal brukes!
    # Alternative funksjoner til sensorene kan finnes på
    # https://docs.pybricks.com/en/latest/ev3devices.html
    light = [robot["myColorSensor"].reflection()]
    measurements["light"] = light
    # color = [robot["myColorSensor"].color()]
    # measurements["color"] = color
    # ambient = [robot["myColorSensor"].ambient()]
    # measurements["ambient"] = ambient

    touch = [robot["myTouchSensor"].pressed()]
    measurements["touch"] = touch

    distance = [robot["myUltrasonicSensor"].distance()]
    measurements["distance"] = distance

    # robot["myGyroSensor"].reset_angle(0)
    gyroAngle = [robot["myGyroSensor"].angle()]
    measurements["gyroAngle"] = gyroAngle
    gyroRate = [robot["myGyroSensor"].speed()]
    measurements["gyroRate"] = gyroRate

    # Motorer
    # Kommenter bort de som ikke skal brukes!

    motorAspeed = [robot["motorA"].speed()]
    measurements["motorAspeed"] = motorAspeed
    motorAangle = [robot["motorA"].angle()]
    measurements["motorAangle"] = motorAangle

    # motorBspeed = [robot["motorB"].speed()]
    # measurements["motorBspeed"] = motorBspeed
    # motorBangle= [robot["motorB"].angle()]
    # measurements["motorBangle"] = motorBangle

    # motorCspeed = [robot["motorC"].speed()]
    # measurements["motorCspeed"] = motorCspeed
    # motorCangle= [robot["motorC"].angle()]
    # measurements["motorCangle"] = motorCangle

    # motorDspeed = [robot["motorD"].speed()]
    # measurements["motorDspeed"] = motorDspeed
    # motorDangle= [robot["motorD"].angle()]
    # measurements["motorDangle"] = motorDangle

    # Her legges til alle de verdiene som skal regnes ut i prosjektet.
    # Nedenfor er verdiene som er brukt i prosjektet for integrasjon gitt.
    flow = []
    volume = [0]
    measurements["flow"] = flow
    measurements["volume"] = volume

    return measurements
