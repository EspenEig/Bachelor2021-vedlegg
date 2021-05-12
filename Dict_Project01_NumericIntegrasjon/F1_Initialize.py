from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.ev3devices import *
from identifyJoystick import identifyJoystick
import struct
import socket


def Initialize(online):
    # dictionary for å holde på all infoen om EV3en.
    # Den inneholder  info om hvilken joystick som er koblet til,
    # navnet til filen der målingene skal lagres,
    # hvilke sensorer som skal brukes,
    # og socketobjektet som gjør det mulig å kommunisere
    # mellom PC og EV3 live (for plotting).
    # Sistnevnte tas kun med hvis det kjøres i online-modus.
    robot = {}

    # Initialiser EV3en. Dette objektet vil altså referere til EV3en,
    # og ved hjelp av denne er det mulig å få EV3en til å utføre kommandoer.
    # F.eks. vil "ev3.speaker.beep()" få robotten til å gi fra seg et lite pip.
    ev3 = EV3Brick()
    robot["brick"] = ev3

    # joystick er en dictionary som inneholder
    # all nødvendig info om joysticken.
    joystick = {}
    joystick["id"] = identifyJoystick()
    joystick["FORMAT"] = 'llHHI'
    joystick["EVENT_SIZE"] = struct.calcsize(joystick["FORMAT"])
    try:
        joystick["in_file"] = open("/dev/input/event2", "rb")
    except OSError:  # hvis ingen joystick er koblet til
        joystick["in_file"] = None
    robot["joystick"] = joystick

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
    robot["myColorSensor"] = myColorSensor

    # myTouchSensor = TouchSensor(Port.S2)
    # robot["myTouchSensor"] = myTouchSensor

    # myUltrasonicSensor = UltrasonicSensor(Port.S3)
    # robot["myUltrasonicSensor"] = myUltrasonicSensor

    # myGyroSensor = GyroSensor(Port.S4)
    # robot["myGyroSensor"] = myGyroSensor

    # Initialiserer motorer.
    # Her legges det til alle motorer som skal brukes i prosjektet.
    # I tillegg til motorene,
    # må det og legges til hvilken port motorene er koblet til.
    # Utfyllende info om Motor-klassen,
    # inkludert parametre og metoder finnes på:
    # https://pybricks.github.io/ev3-micropython/ev3devices.html
    # Her er det viktig å kommentere vekk de motorene som ikke skal brukes.
    # Pass på at motoren er koblet til den porten som er gitt som parameter!
    # motorA = Motor(Port.A)
    # motorA.reset_angle(0)
    # robot["motorA"] = motorA

    # motorB = Motor(Port.B)
    # motorB.reset_angle(0)
    # robot["motorB"] = motorB

    # motorC = Motor(Port.C)
    # motorC.reset_angle(0)
    # robot["motorC"] = motorC

    # motorD = Motor(Port.D)
    # motorD.reset_angle(0)
    # robot["motorD"] = motorD

    # Fila hvor målingene lagres.
    # OBS: Her er 'w' parameteren gitt, slik at for hver kjøring så vil
    # alle gamle målinger slettes og erstattes med nye.
    # For andre mulige parametre, se:
    # https://docs.python.org/3/library/functions.html#open
    robot["out_file"] = open("measurements.txt", "w")

    if online:
        # Sett opp socketobjektet, og hør etter for "connection"
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        robot["sock"] = sock
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", 8070))
        sock.listen(1)

        # Gi et pip fra robotten samt print i terminal
        # for å vise at den er klar
        print("Waiting for connection")
        ev3.speaker.beep()

        # Motta koblingen og send tilbake "acknowledgment"
        connection, address = sock.accept()
        connection.send(b"ack")
        print("Acknowlegment sent")
        robot["connection"] = connection

    # returnerer robot dictionaryen med all informasjonen
    return robot
