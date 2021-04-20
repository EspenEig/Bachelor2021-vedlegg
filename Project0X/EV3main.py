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
import uselect
from time import perf_counter
from time import sleep

# Er du koblet til NXT eller ikke?
online = True



# variabel som settes lik True når styrestikken trykkes
# inn, og programmet vil da avslutte
joyMainSwitch = False

#joyForwardInstance = 0
#joySideInstance = 0

def main():
    try:
        #Med flere sensorer må man legge det til her
        #robot, myColorSensor, out_file, motorB, motorC = Initialize()
        robot, myColorSensor = Initialize(online)

        print("Initialized.")

        zeroTimeInit, time, light = GetFirstMeasurements(robot, myColorSensor)

        print("First measurements acquired.")
        # initialiser diskret tellevariabel
        k = 0

        # Read joystick
        _thread.start_new_thread(JoyMainSwitch, [robot])

        print("Ready.")

        while True:
            if joyMainSwitch:
                break

            # inkrementer den diskrete tellevariabelen
            k+=1
            GetNewMeasurement(zeroTimeInit, time, light, myColorSensor)
            print("New measurements acquired.")
            # CalculateAndSetMotorPower(motorA, light)
            SendData(robot, time, light, online)
            if online:
                print("Data sent to computer and measurements file.")
            else:
                print("Data sent to measurements file.")

            # Hvis du får socket timeouts, fjern kommentar foran sleep(1)
            # og "from time import sleep" på linje 18
            # sleep(1)
    except Exception as e:
        print(e)
    finally:
        CloseMotorsAndSensors(robot, online)


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
    joystick["id"] = IdentifyJoystick()
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

    #myTouchSensor = TouchSensor(Port.S2)

    #myUltrasonicSensor = UltrasonicSensor(Port.S3)

    #myGyroSensor = GyroSensor(Port.S4)


    # Initialiserer motorer.

    # Her legges det til alle motorer som skal brukes i prosjektet.
    # I tillegg til motorene,
    # må det og legges til hvilken port motorene er koblet til.
    # Utfyllende info om Motor-klassen,
    # inkludert parametre og metoder finnes på:
    # https://pybricks.github.io/ev3-micropython/ev3devices.html
    # Her er det viktig å kommentere vekk de motorene som ikke skal brukes.
    # Pass på at motoren er koblet til den porten som er gitt som parameter!
    #motorA = Motor(Port.A)
    #motorA.reset_angle(0)

    #motorB = Motor(Port.B)
    #motorB.reset_angle(0)

    #motorC = Motor(Port.C)
    #motorC.reset_angle(0)

    #motorD = Motor(Port.D)
    #motorD.reset_angle(0)

    return robot, myColorSensor


def GetFirstMeasurements(robot, myColorSensor):
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

    #touch = [myTouchSensor.pressed()]

    #distance = [myUltrasonicSensor.distance()]

    # myGyroSensor.reset_angle(0)
    # gyroAngle = [myGyroSensor.angle()]

    # gyroRate = [myGyroSensor.speed()]

    
    


    # Motorer
    # Kommenter bort de som ikke skal brukes!

    #motorAspeed = [motorA.speed()]
    #motorAangle = [motorA.angle()]

    # motorBspeed = [motorB.speed()]
    # motorBangle= [motorB.angle()]

    # motorCspeed = [motorC.speed()]
    # motorCangle= [motorC.angle()]

    # motorDspeed = [motorD.speed()]
    # motorDangle= [motorD.angle()]

    # Verdiene er også nødt til å bli lagt ut i return linjen under
    return zeroTimeInit, time, light


# Aktive sensorer må bli lagt inn som input
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
    #touch.append(myTouchSensor.pressed())

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
    # motorPaadragB = lysmaaling*a
    # motorPaadragC = lysmaaling*a
    # motorPaadragD = lysmaaling*a

    # Sett hastigen på motorene.
    motorA.run(motorPaadragA)
    #motorB.run(motorPaadragB)
    #motorC.run(motorPaadragC)
    #motorD.run(motorPaadragD)


def SendData(robot, time, light, online):
    """
    Sender data fra EV3 til datamaskin, lagrer også målte og beregnede
    verdier på EV3en i målefila som ble definert i Initialize().

    Parametre:
    robot - robot dictionaryen
    time - tidliste
    light - lysliste
    online - bool; om det kjøres i online modus eller ikke.
    Hvis det ikke kjøres i online modus vil ikke roboten
    prøve å sende data til PCen.

    Andre målte og beregnede verdier som skal
    lagres på EV3en og skal sendes til
    PCen må legges til som parametre.
    """

    # for json må alle dataene pakkes inn i en dictionary
    # hvor hver kjøring tømmes 'data' og 'dataString'
    data = {}
    dataString = ""

    # hver verdi i dictionaryen må referere til en tuple
    data["time"] = (time[-1])
    data["light"] = (light[-1])

    # for å skrive til measurements.txt
    dataString += str(time[-1]) + ","
    dataString += str(light[-1]) + "\n"

    # Trenger ikke å forandre på noe fra linje 227 til linje 229.
    # Skriv dataString til fil
    print("Writing to measurements.txt")
    robot["outfile"].write(dataString)

    # Send data til PC
    if online:
        msg = json.dumps(data)
        print("Sending data from EV3 to computer.")
        robot["connection"].send(msg)
        
        # Vent til PC annerkjenner data før fortsettelse
        if not robot["connection"].recv(3) == b"ack":
            print("No data ack")


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

def JoyMainSwitch(robot):
    global joyMainSwitch
    print("Thread started")
    while True:
        if robot["joystick"]["in_file"] is not None:
            event = robot["joystick"]["in_file"].read(
                robot["joystick"]["EVENT_SIZE"])
            (tv_sec, tv_usec, ev_type, code, value) = struct.unpack(
                robot["joystick"]["FORMAT"], event)
            if ev_type == 1:
                print("Joystick signal received, stopping program.")
                robot["brick"].speaker.beep()
                joyMainSwitch = True
                return

if __name__ == '__main__':
    main()