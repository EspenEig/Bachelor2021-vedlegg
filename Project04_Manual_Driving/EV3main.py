#!/usr/bin/env pybricks-micropython
# ------------------------------------------------
# Prosjekt04_ManuellKjøring
#
# Hensikten med programmet er
# Følgende sensorer brukes: Lys
# Følgende motorer brukes: B, C
#
# ------------------------------------------------

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.ev3devices import *
import struct
import socket
import json
import _thread
import uselect
from time import perf_counter, sleep

# Er du koblet til NXT eller ikke?
online = False

joyMainSwitch = False
joyForwardInstance = 0
joySideInstance = 0


def main():
    try:
        robot, myColorSensor, out_file, motorB, motorC = Initialize()
        print("Initialized.")
        zeroTimeInit, time, light, joyForward, joySide, powerB,  powerC = \
            GetFirstMeasurement(myColorSensor)
        print("First measurements acquired.")

        if robot["joystick"]["in_file"] is not None:
            _thread.start_new_thread(getJoystickValues, [robot])
        sleep(0)

        print("Ready.")

        while True:
            if joyMainSwitch:
                break

            GetNewMeasurement(zeroTimeInit, time, light, myColorSensor,
                              joyForward, joySide)
            # print("New measurements acquired.")
            CalculateAndSetMotorPower(motorB, powerB, motorC, powerC,
                                      joyForward, joySide)

            SendData(robot, time, light, out_file,
                     powerB, powerC)
            # print("Data sent to computer.")

            # Hvis du får socket timeouts, fjern kommentar foran sleep(1)
            # sleep(1)
    except Exception as e:
        print(e)
    finally:
        CloseMotorsAndSensors(robot, out_file, motorB, motorC)


def Initialize():
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
    joystick["id"] = identifyJoystick()
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
    out_file = open("measurements.txt", "w")

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

    # Sensorer
    myColorSensor = ColorSensor(Port.S1)

    # Motorer
    motorB = Motor(Port.B)
    motorB.reset_angle(0)
    motorC = Motor(Port.C)
    motorC.reset_angle(0)

    return robot, myColorSensor, out_file, motorB, motorC


def GetFirstMeasurement(myColorSensor):
    """
    Får inn første måling fra sensorer og motorer.

    Parametre:
    robot - robot dictionaryen
    myColorSensor - Fargesensoren

    Andre sensorer og motorer må legges til som parametre
    hvis de skal brukes.
    """

    # Tider
    zeroTimeInit = perf_counter()  # nulltida
    time = [0]  # tida

    # Sensorer
    light = [myColorSensor.reflection()]

    # Motorer
    powerB = [0]
    powerC = [0]

    # Joystick
    joyForward = [joyForwardInstance]
    joySide = [joySideInstance]

    return zeroTimeInit, time, light, joyForward, joySide, \
        powerB, powerC


def GetNewMeasurement(zeroTimeInit, time, light, myColorSensor,
                      joyForward, joySide):
    """
    Får inn nye målinger fra sensorer og motorer. Denne blir kalt i while-løkka
    i main() funksjonen.

    Parametre:
    zeroTimeInit - nulltida
    time - liste med tider
    light - liste med målte lysverdier fra lyssensoren
    myColorSensoren - lyssensoren, for å få inn nye lysverdier

    Andre sensorer og motorer må legges til som parametre
    hvis de skal brukes.
    """

    time.append(perf_counter() - zeroTimeInit)
    light.append(myColorSensor.reflection())

    if light[-1] > 70:
        global joyMainSwitch
        joyMainSwitch = True

    joyForward.append(joyForwardInstance)
    joySide.append(joySideInstance)


def CalculateAndSetMotorPower(motorB, powerB, motorC, powerC,
                              joyForward, joySide):
    """
    Beregner og setter pådrag til motorene som brukes.

    Parametre:
    motorA - Motor A
    lys - lysliste

    Andre motorer og verdier som brukes for å beregne pådrag
    må legges til som parametre.
    """

    # Parametre for beregning til motorpådrag
    a = 2
    b = 1

    # pådraget
    powerB.append(joyForward[-1] * b + joySide[-1] * a)
    powerC.append(joyForward[-1] * b - joySide[-1] * a)

    # Sett hastigheten på motorene.
    motorB.dc(powerB[-1])
    motorC.dc(powerC[-1])


def SendData(robot, time, light, out_file, powerB, powerC):
    """
    Sender data fra EV3 til datamaskin, lagrer også målte og beregnede
    verdier på EV3en i "out_file" som ble definert i Initialize().

    Parametre:
    robot - robot dictionaryen
    time - tidliste
    light - lysliste
    online - bool; om det kjøres i online modus eller ikke.
    Hvis det ikke kjøres i online modus vil ikke roboten
    prøve å sende data til PCen.
    out_file - filobjektet på EV3en som data skal skrives til.

    Andre målte og beregnede verdier som skal
    lagres på EV3en og skal sendes til
    PCen må legges til som parametre.
    """

    # Tøm 'dataString'
    global runTime, msgCount
    # print("Run time: ", perf_counter() - runTime)
    # sendTime = perf_counter()
    dataString = ""

    # for å skrive til measurements.txt
    dataString += str(time[-1]) + ","
    dataString += str(light[-1]) + ","
    dataString += str(powerB[-1]) + ","
    dataString += str(powerC[-1]) + "\n"

    # Trenger ikke å forandre på noe fra linje 227 til linje 229.
    # Skriv dataString til fil
    # print("Writing to measurements.txt")
    out_file.write(dataString)
    # print("Write time: ", perf_counter() - sendTime)
    # runTime = perf_counter()

    # Send data til PC
    if online:
        # For json må alle dataene pakkes inn i en dictionary
        data = {}

        # hver verdi i dictionaryen må referere til en tuple
        data["time"] = (time[-1])
        data["light"] = (light[-1])
        data["powerB"] = (powerB[-1])
        data["powerC"] = (powerC[-1])

        msg = json.dumps(data)
        # print("Sending data from EV3 to computer.")
        robot["connection"].send(bytes(msg, "utf-8") + b"?")


def CloseMotorsAndSensors(robot, out_file, motorB, motorC):
    """
    Lukker filobjektene (out_file hvor målinger lagres på EV3en,
    og joystick fila på EV3en), og socket-koblingene mellom EV3
    og PC.
    Denne funksjonen skal ikke forandres på.

    Parametre:
    robot - robot dictionaryen
    online - bool; om det kjøres i online eller ikke. Hvis det ikke
    kjøres i online så er det ingen socket-objekt å lukke
    out_file - filobjektet som referer til fila på EV3en hvor målinger
    og beregnede verdier lagres.
    """

    robot["joystick"]["in_file"].close()
    out_file.close()
    if online:
        robot["connection"].send(b"end")
        robot["connection"].close()
        robot["sock"].close()

    motorB.brake()
    motorC.brake()


def identifyJoystick():
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

    event_poll = uselect.poll()
    if robot["joystick"]["in_file"] is not None:
        event_poll.register(robot["joystick"]["in_file"], uselect.POLLIN)
    else:
        return
    while True:
        events = event_poll.poll(0)
        if len(events) > 0 and events[0][1] & uselect.POLLIN:
            try:
                (_, _, ev_type, code, value) = struct.unpack(
                    robot["joystick"]["FORMAT"],
                    robot["joystick"]["in_file"].read(
                        robot["joystick"]["EVENT_SIZE"]))
            except Exception as e:
                print(e)
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
