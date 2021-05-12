#!/usr/bin/env pybricks-micropython

import time as time
from math import log
from sys import stderr

from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.iodevices import AnalogSensor

class ntc():

    def __init__(self):
        self.beta = 3984.0
        self.To = 25.0
        self.Ro = 10.0e3
        self.Rt = None
        self._maxADC = None
        self._bitsADC = None
        self.bitsADC(10)

    def bitsADC(self, newval=None):
        if newval != None:
            self._bitsADC = newval
            self._maxADC = 2 ** self._bitsADC
        return self._bitsADC

    def maxADC(self, newval=None):
        if newval != None:
            self._maxADC = newval
        return self._maxADC
    
    def _steinhart_temperature_C_(self, r):
        steinhart = log(r / self.Ro) / self.beta      # log(R/Ro) / beta
        steinhart += 1.0 / (self.To + 273.15)         # log(R/Ro) / beta + 1/To
        steinhart = (1.0 / steinhart) - 273.15   # Invert, convert to C
        return steinhart

    def temp_C(self, ADC: [int, float]):
        self.Rt = self.Ro / ((self._maxADC / ADC - 1))
        return self._steinhart_temperature_C_(self.Rt)

# Initialiserer EV3 brikken
ev3 = EV3Brick()

#NTC koeffisienter
a = ntc()
a.beta = 3984
a.maxADC(5000)

# Spesifiserer at temperatursensoren er koblet til port4.
ntc_sensor = AnalogSensor(Port.S4)

#while True:
    #ntc_msr = ntc_sensor.voltage()

    #tmp = a.temp_C(ntc_msr)

    # print til VS Code output panel
    #print(tmp, file=stderr)

    #time.sleep(1)