#!/usr/bin/env pybricks-micropython
# ------------------------------------------------
# Prosjekt0X_ProsjektNavn
#
# Hensikten med programmet er
# Følgende sensorer brukes:
# Følgende motorer brukes:
#
# ------------------------------------------------

import struct
from F1_Initialize import Initialize
from F2_GetFirstMeasurement import GetFirstMeasurement
from F3_GetNewMeasurement import GetNewMeasurement
from F4_MathCalculations import MathCalculations
from F5_CalculateAndSetMotorPower import CalculateAndSetMotorPower
from F6_SendData import SendData
from F7_CloseMotorsAndSensors import CloseMotorsAndSensors
# from time import sleep

# Er du koblet til NXT eller ikke?
online = True

robot = Initialize(online)
print("Initialized")
measurements = GetFirstMeasurement(robot)
print("First measurement aquired")

# sjekk om styrestikk er koblet til
if robot["joystick"]["in_file"] != None:
    # Read joystick
    event = robot["joystick"]["in_file"].read(robot["joystick"]["EVENT_SIZE"])

print("Ready")

while True:

    # sjekk om styrestikk er koblet til
    if robot["joystick"]["in_file"] != None:
        (tv_sec, tv_usec, ev_type, code, value) = struct.unpack(
            robot["joystick"]["FORMAT"], event)
        if ev_type == 1:
            print("Stopping")
            robot["brick"].speaker.beep()
            break

    GetNewMeasurement(robot, measurements)
    MathCalculations(measurements)
    CalculateAndSetMotorPower(robot, measurements)
    SendData(robot, measurements, online)

    if robot["joystick"]["in_file"] != None:
        event = robot["joystick"]["in_file"].read(robot["joystick"]["EVENT_SIZE"])

    # If you get socket timeouts, remove comments form sleep(1)
    # and the import statement at the top of the file
    # sleep(1)

CloseMotorsAndSensors(robot, online)
