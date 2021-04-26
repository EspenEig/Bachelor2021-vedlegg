#!/usr/bin/env python3 

from math import log
import time as time
from sys import stderr

#import matplotlib.pyplot as plt

#from pybricks.hubs import EV3Brick
#from pybricks.parameters import Port
#from pybricks.ev3devices import *
#from pybricks.nxtdevices import TemperatureSensor

from ev3dev2.port import LegoPort
from ev3dev2.sensor import INPUT_4
from ev3dev2.sensor.lego import Sensor

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

#Definerer NTC parametere:
a = ntc()
a.beta = 3984
a.maxADC(5000)

#
#tempS = TemperatureSensor(Port.S4)


#Definerer port 4 til å være analog inngang:
p4 = LegoPort(INPUT_4)
p4.mode = 'nxt-analog'
p4.set_device = 'nxt-analog'

#Definerer at temperatursensoren er koblet til port 4: 
ntc_sensor = Sensor(INPUT_4)

while True:
    
    #Leser av sensorverdi i volt:
    msr = ntc_sensor.value()

    #Konverterer til temperatur:
    t = a.temp_C(msr)

    # Printer temperaturen til EV3 LCD skjerm
    #print('The temperature is: %f deg' %t)
    # Printer temperaturen til VS Code output panel
    print('The temperature is: %f deg' %t, file=stderr)

    time.sleep(0.3)