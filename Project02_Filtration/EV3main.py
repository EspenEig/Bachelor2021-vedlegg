#!/usr/bin/env pybricks-micropython
# ------------------------------------------------
# Prosjekt 2, filtrering
#
# Hensikten med programmet er å simulere filtrering av 
# temperatur ved hjelp av lysverdier
# Følgende sensorer brukes: lyssensor
# Følgende motorer brukes:
#
# ------------------------------------------------

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.ev3devices import *
import struct
import socket
import json
from time import perf_counter
# from time import sleep

# Er du koblet til NXT eller ikke?
online = False

def main():
    robot, myColorSensor, out_file = Initialize(online)
    print("Initialized.")
    zeroTimeInit, time, light = GetFirstMeasurement(robot, myColorSensor)
    print("First measurements acquired.")

    # diskret tellevariabel
    k = 0

    # Read joystick
    if robot["joystick"]["in_file"] is not None:
        event = robot["joystick"]["in_file"].read(
            robot["joystick"]["EVENT_SIZE"])
    print("Ready.")

    while True:
        if robot["joystick"]["in_file"] is not None:
            (tv_sec, tv_usec, ev_type, code, value) = struct.unpack(
                robot["joystick"]["FORMAT"], event)
            if ev_type == 1:
                print("Joystick signal received, stopping program.")
                robot["brick"].speaker.beep()
                break

        # inkrementer den diskrete tellevariabelen
        k+=1
        GetNewMeasurement(zeroTimeInit, time, light, myColorSensor)
        # print("New measurements acquired.")
        # CalculateAndSetMotorPower(motorA, light)
        SendData(robot, k, time, light, online, out_file)
        # if online:
        #     print("Data sent to computer and measurements file.")
        # else:
        #     print("Data sent to measurements file.")


        if robot["joystick"]["in_file"] is not None:
            event = robot["joystick"]["in_file"].read(
                robot["joystick"]["EVENT_SIZE"])

        # Hvis du får socket timeouts, fjern kommentar foran sleep(1)
        # og "from time import sleep" på linje 18
        # sleep(1)

    CloseMotorsAndSensors(robot, online, out_file)


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

    # robot inneholder all info om robotten
    robot = {}

    ev3 = EV3Brick()
    robot["brick"] = ev3

    # joystick inneholder all info om joysticken.
    joystick = {}
    joystick["id"] = identifyJoystick()
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
        #ev3.speaker.beep()

        # Motta koblingen og send tilbake "acknowledgment" som byte
        connection, _ = sock.accept()
        connection.send(b"ack")
        print("Acknowlegment sent to computer.")
        robot["connection"] = connection

    # Sensorer
    myColorSensor = ColorSensor(Port.S1)

    # Motorer
    # motorA = Motor(Port.A)
    # motorA.reset_angle(0)

    return robot, myColorSensor, out_file


def GetFirstMeasurement(robot, myColorSensor):
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

    return zeroTimeInit, time, light


def GetNewMeasurement(zeroTimeInit, time, light, myColorSensor):
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


def CalculateAndSetMotorPower(motorA, lys):
    """
    Beregner og setter pådrag til motorene som brukes.

    Parametre:
    motorA - Motor A
    lys - lysliste

    Andre motorer og verdier som brukes for å beregne pådrag
    må legges til som parametre.
    """

    # Parametre for beregning til motorpådrag
    a = 1
    b = 1

    # Her brukes siste lysmåling for å beregne motorpådrag.
    lysmaaling = lys[-1]

    # pådraget
    motorPaadragA = lysmaaling*a*b

    # Sett hastigen på motorene.
    motorA.run(motorPaadragA)


def SendData(robot, k, time, light, online, out_file):
    """
    Sender data fra EV3 til datamaskin, lagrer også målte og beregnede
    verdier på EV3en i "out_file" som ble definert i Initialize().

    Parametre:
    robot - robot dictionaryen
    k - diskret tellevariabel
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

    # for json må alle dataene pakkes inn i en dictionary
    # hvor hver kjøring tømmes 'data' og 'dataString'
    data = {}
    dataString = ""

    # hver verdi i dictionaryen må referere til en tuple
    # print("xd")
    # print(k)
    # print(time)
    # print(light)
    data["tellevariabel"] = (k)
    data["time"] = (time[-1])
    data["light"] = (light[-1])

    # for å skrive til measurements.txt
    dataString += str(time[-1]) + ","
    dataString += str(light[-1]) + "\n"

    # Trenger ikke å forandre på noe fra linje 227 til linje 229.
    # Skriv dataString til fil
    # print("Writing to measurements.txt")
    out_file.write(dataString)

    # Send data til PC
    if online:
        msg = json.dumps(data)
        # print("Sending data from EV3 to computer.")
        robot["connection"].send(msg)
        
        # Vent til PC annerkjenner data før fortsettelse
        if not robot["connection"].recv(3) == b"ack":
            print("No data ack")


def CloseMotorsAndSensors(robot, online, out_file):
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

if __name__ == '__main__':
    main()