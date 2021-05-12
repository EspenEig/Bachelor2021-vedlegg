#!/usr/bin/env pybricks-micropython
# ------------------------------------------------
# Prosjekt0X_ProsjektNavn
#
# Hensikten med programmet er
# Følgende sensorer brukes:
# Følgende motorer brukes:
#
# ------------------------------------------------

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.ev3devices import *
import struct
import socket
import json
import _thread
from time import perf_counter, sleep

# Er du koblet til NXT eller ikke?
online = True

# joyMainSwitch settes lik True når styrestikken trykkes
# inn, og programmet vil da avslutte
joyMainSwitch = False

joyForwardInstance = 0
joySideInstance = 0


def main():
    try:
        # Med flere sensorer må man legge det til her
        # robot, myColorSensor, out_file, motorB, motorC = Initialize()
        robot, myColorSensor, motorA = Initialize(online)

        print("Initialized.")

        zeroTimeInit, time, light, powerA, joyForward, joySide = \
            GetFirstMeasurements(myColorSensor)

        print("First measurements acquired.")

        # Read joystick
        _thread.start_new_thread(getJoystickValues, [robot])

        print("Ready.")

        while True:
            if joyMainSwitch:
                break
            GetNewMeasurement(zeroTimeInit, time, light, myColorSensor,
                              joyForward, joySide)
            CalculateAndSetMotorPower(motorA, powerA, light,
                                      joyForward, joySide)
            SendData(robot, online, time, light)

            # Hvis du får socket timeouts, fjern kommentar foran sleep(1)
            # sleep(1)
    except Exception as e:
        print("main:", e)
    finally:
        CloseMotorsAndSensors(robot, online)

        # Det er 3 forskjellige måter å stoppe motorene på:
        # stop() stopper motorpådraget, men bremser ikke.
        # brake() stopper motorpådraget, og bruker strømmen.
        # generert av rotasjonen til å bremse.
        # hold stopper motorpådraget og låser rotasjonsvinkelen.
        motorA.stop()
        # motorB.brake()
        # motorC.hold()


def Initialize(online):
    """
    Initialiserer robot-dictionaryen som innheolder robot-objektet og
    en socket-forbindelse (hvis online = True) for kommunikasjon til roboten.
    Initialiserer alle sensorer og motorer.
    Initialiserer fila på EV3en som brukes for å lagre målinger.
    Det eneste som skal forandres på fra prosjekt til prosjekt er sensorer,
    motorer og hva som returneres av funksjonen.

    Parametre:
    online - bool; bestemmer om det kjøres i online modus eller ikke.
    Ved kjøring i online modus blir det satt opp ett socket-objekt
    for live kommunikasjon til PC.
    """

    # robot inneholder all info om roboten
    robot = {}

    ev3 = EV3Brick()
    robot["brick"] = ev3

    # joystick inneholder all info om joysticken.
    joystick = {}
    joystick["id"] = IdentifyJoystick()
    joyScale = 0
    if joystick["id"] == "logitech":
        joyScale = 1024
    elif joystick["id"] == "dacota":
        joyScale = 255
    joystick["scale"] = joyScale
    joystick["FORMAT"] = 'llHHI'
    joystick["EVENT_SIZE"] = struct.calcsize(joystick["FORMAT"])
    try:
        joystick["in_file"] = open("/dev/input/event2", "rb")
    except OSError:  # hvis ingen joystick er koblet til
        joystick["in_file"] = None
    robot["joystick"] = joystick

    # Fila hvor målingene lagres
    robot["outfile"] = open("measurements.txt", "w")

    if online:
        # Sett opp socketobjektet, og hør etter for "connection"
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot["sock"] = sock
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", 8070))
        sock.listen(1)

        # Gi et pip fra robotten samt print i terminal
        # for å vise at den er klar for socketkobling fra PC
        print("Waiting for connection from computer.")
        ev3.speaker.beep()

        # Motta koblingen og send tilbake "acknowledgment" som byte
        connection, _ = sock.accept()
        connection.send(b"ack")
        print("Acknowlegment sent to computer.")
        robot["connection"] = connection

    # Initialiser sensorer.

    # Her legges det til alle sensorer som skal brukes i prosjektet.
    # I tillegg til sensoren,
    # må det og legges til hvilken port sensoren er koblet til.
    # Utfyllende info om de ulike sensorklassene,
    # inkludert paramerte og metoder finnes på:
    # https://docs.pybricks.com/en/latest/ev3devices.html
    # Her er det viktig å kommentere vekk de sensorene som ikke skal brukes.
    # Pass på at sensoren er koblet til den porten som er gitt som parameter!
    myColorSensor = ColorSensor(Port.S1)

    # myTouchSensor = TouchSensor(Port.S2)

    # myUltrasonicSensor = UltrasonicSensor(Port.S3)

    # myGyroSensor = GyroSensor(Port.S4)

    # Initialiserer motorer.

    # Her legges det til alle motorer som skal brukes i prosjektet.
    # I tillegg til motorene,
    # må det og legges til hvilken port motorene er koblet til.
    # Utfyllende info om Motor-klassen,
    # inkludert parametre og metoder finnes på:
    # https://pybricks.github.io/ev3-micropython/ev3devices.html
    # Her er det viktig å kommentere vekk de motorene som ikke skal brukes.
    # Pass på at motoren er koblet til den porten som er gitt som parameter!
    motorA = Motor(Port.A)
    motorA.reset_angle(0)

    # motorB = Motor(Port.B)
    # motorB.reset_angle(0)

    # motorC = Motor(Port.C)
    # motorC.reset_angle(0)

    # motorD = Motor(Port.D)
    # motorD.reset_angle(0)

    return robot, myColorSensor, motorA


def GetFirstMeasurements(myColorSensor):
    """
    Får inn første måling fra sensorer og motorer.

    Parametre:
    robot - robot dictionaryen
    myColorSensor - Fargesensoren

    Andre sensorer og motorer må legges til som parametre
    hvis de skal brukes.
    """

    # Tid.
    # Dette blir nulltida,
    # og alle fremtidige tidsmålinger blir satt i forhold til denne
    zeroTimeInit = perf_counter()  # nulltida
    time = [0]  # tida

    # Sensorer
    # Kommenter vekk de som ikke skal brukes!
    # Alternative funksjoner til sensorene kan finnes på
    # https://docs.pybricks.com/en/latest/ev3devices.html

    light = [myColorSensor.reflection()]
    # ambient = [myColorSensor.ambient()]

    # touch = [myTouchSensor.pressed()]

    # distance = [myUltrasonicSensor.distance()]

    # myGyroSensor.reset_angle(0)
    # gyroAngle = [myGyroSensor.angle()]

    # gyroRate = [myGyroSensor.speed()]

    # Motorer
    # Kommenter bort de som ikke skal brukes!

    powerA = [0]
    # motorAangle = [motorA.angle()]

    # powerB = [0]
    # motorBangle= [motorB.angle()]

    # motorC = [0]
    # motorCangle= [motorC.angle()]

    # powerD = [0]
    # motorDangle= [motorD.angle()]

    # Joystick
    joyForward = [joyForwardInstance]
    joySide = [joySideInstance]

    # Verdiene er også nødt til å bli lagt ut i return linjen under
    return zeroTimeInit, time, light, powerA, joyForward, joySide


# Aktive sensorer må bli lagt inn som input
def GetNewMeasurement(zeroTimeInit, time, light, myColorSensor,
                      joyForward, joySide):
    """
    Får inn nye målinger fra sensorer og motorer. Denne blir kalt i while-løkka
    i main() funksjonen.

    Parametre:
    zeroTimeInit - nulltida
    time - liste med tider
    light - liste med målte lysverdier fra lyssensoren
    myColorSensor - lyssensoren, for å få inn nye lysverdier
    joyForward, joySide - liste med verdier fra styrestikken

    Andre sensorer og motorer må legges til som parametre
    hvis de skal brukes.
    """
    # Legg til differansen mellom nulltida og perf_counter() ("nå")
    time.append(perf_counter() - zeroTimeInit)
    light.append(myColorSensor.reflection())

    # Her legges de målte verdiene til.
    # Husk å kommenter vekk alle sensorer som ikke blir brukt!

    # Legg til den målte lysverdien
    light.append(myColorSensor.reflection())

    # Legg til den målte fargeverdien
    # color.append(myColorSensor.color())

    # Legg til den målte ambient-/lysintensitetsverdien
    # ambient.append(myColorSensor.ambient())

    # Legg til den målte bryterverdien
    # touch.append(myTouchSensor.pressed())

    # Legg til den målte distanseverdien til ultrasonicsensoren
    # distance.append(myUltrasonicSensor.distance())

    # Legg til den målte vinkelverdien til gyrosensoren
    # gyroAngle.append(myGyroSensor.angle())
    # Legg til den målte farten (angular velocity) til gyrosensoren
    # gyroRate.append(myGyroSensor.speed())

    # Legg til den målte akkumulerte vinkelverdien til motor A
    # motor A tilsvarer altså motoren som er festet til port A
    # motorAangle.append(motorA.angle())

    # Legg til den målte farten (angular velocity) til motor A
    # motorAspeed.append(motorA.speed())

    # Legg til en målte akkumulerte vinkelverdien til motor B
    # motorBangle.append(motorB.angle())

    # Legg til den målte farten (angular velocity) til motor B
    # motorBspeed.append(motorB.speed())

    # Legg til en målte akkumulerte vinkelverdien til motor C
    # motorCangle.append(motorC.angle())

    # Legg til den målte farten (angular velocity) til motor C
    # motorCspeed.append(motorC.speed())

    # Legg til en målte akkumulerte vinkelverdien til motor D
    # motorDangle.append(motorD.angle())

    # Legg til den målte farten (angular velocity) til motor D
    # motorDspeed.append(motorD.speed())

    # Legg til den nyeste joystick posisjonen
    joyForward.append(joyForwardInstance)
    joySide.append(joySideInstance)


def CalculateAndSetMotorPower(motorA, powerA, light, joyForward, joySide):
    """
    Beregner og setter pådrag til motorene som brukes.

    Parametre:
    motorA - Motor A
    powerA - motorpådragliste for motor A
    light - lysliste
    joyForward, joySide - styrestikklister

    Andre motorer og verdier som brukes for å beregne pådrag
    må legges til som parametre.
    """

    # Parametre for beregning til motorpådrag
    a = 1
    b = 1

    # Her brukes siste lysmåling for å beregne motorpådrag.
    lysmaaling = light[-1]

    # pådraget
    powerA.append(lysmaaling + a * joyForward[-1] - b * joySide[-1])
    # powerB = lysmaaling*a
    # powerC = lysmaaling*a
    # powerD = lysmaaling*a

    # Sett hastigen på motorene.
    motorA.run(powerA[-1])
    # motorB.run(powerB[-1])
    # motorC.run(powerC[-1])
    # motorD.run(powerD[-1])


def SendData(robot, online, time, light):
    """
    Sender data fra EV3 til datamaskin, lagrer også målte og beregnede
    verdier på EV3en i målefila som ble definert i Initialize().

    Parametre:
    robot - robot dictionaryen
    online - bool; om det kjøres i online modus eller ikke.
    Hvis det ikke kjøres i online modus vil ikke roboten
    prøve å sende data til PCen.
    time - tidliste
    light - lysliste

    Andre målte og beregnede verdier som skal
    lagres på EV3en og skal sendes til
    PCen må legges til som parametre.
    """

    # for json må alle dataene pakkes inn i en dictionary
    # for hver kjøring tømmes 'dataString'
    data = {}
    dataString = ""

    # for å skrive til measurements.txt
    dataString += str(time[-1]) + ","
    dataString += str(light[-1]) + "\n"

    # hver verdi i dictionaryen må referere til en tuple
    data["time"] = (time[-1])
    data["light"] = (light[-1])

    # Skriv dataString til fil
    robot["outfile"].write(dataString)

    # Send data til PC
    if online:
        msg = json.dumps(data)
        robot["connection"].send(bytes(msg, "utf-b") + b"?")


def CloseMotorsAndSensors(robot, online):
    """
    Lukker filobjektene (målefila hvor målinger lagres på EV3en,
    og joystick fila på EV3en), og socket-koblingene mellom EV3
    og PC.
    Denne funksjonen skal ikke forandres på.

    Parametre:
    robot - robot dictionaryen
    online - bool; om det kjøres i online eller ikke. Hvis det ikke
    kjøres i online så er det ingen socket-objekt å lukke
    """

    robot["joystick"]["in_file"].close()
    robot["outfile"].close()
    if online:
        robot["connection"].send(b"end")
        robot["connection"].close()
        robot["sock"].close()


def IdentifyJoystick():
    """
    Identifiserer hvilken styrestikk som er koblet til;
    enten logitech eller dacota (eventuelt ukjent styrestikk)
    Denne funksjonen skal ikke forandres på.
    """

    for i in range(2, 1000):
        path = ("/dev/bus/usb/001/{:03d}".format(i))
        try:
            with open(path, "rb") as f:
                joy = f.read()
                if joy[2] == 16:
                    return "logitech"
                elif joy[2] == 0:
                    return "dacota"
                else:
                    return "Ukjent joystick."
            break
        except:
            pass


def scale(value, src, dst):
    return ((float(value - src[0])
            / (src[1] - src[0])) * (dst[1] - dst[0])
            + dst[0])


def getJoystickValues(robot):
    global joyMainSwitch, joyForwardInstance, joySideInstance
    print("Thread started")

    if robot["joystick"]["in_file"] is None:
        return
    while True:
        try:
            (_, _, ev_type, code, value) = struct.unpack(
                robot["joystick"]["FORMAT"],
                robot["joystick"]["in_file"].read(
                    robot["joystick"]["EVENT_SIZE"]))
        except Exception as e:
            print("thread:", e)
        if ev_type == 1:
            print("Joystick signal received, stopping program.")
            robot["brick"].speaker.beep()
            joyMainSwitch = True
            return
        elif ev_type == 3:
            if code == 0:
                joySideInstance = scale(
                    value,
                    (robot["joystick"]["scale"], 0),
                    (50, -50))
            elif code == 1:
                joyForwardInstance = scale(
                    value,
                    (0, robot["joystick"]["scale"]),
                    (100, -100))


if __name__ == '__main__':
    main()
