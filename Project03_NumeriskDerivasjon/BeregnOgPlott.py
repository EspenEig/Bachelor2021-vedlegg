# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import socket
import sys
import json
import numpy as np

# Set online flag, ip address and filename.
online = True
EV3_IP = "169.254.123.203"
filename = "measurements.txt"

# Initialize lists.
light = [0]
time = []
k = 0
ts = [0]
lysDerivert = [0]

# FIR-filtrering
temp_fir = [0]
# bestemmer "mengden" filtrering
m = 1000
temp_fir_der = [0]

# IIR-filtrering
temp_iir = [0]
# "vekten"
alpha = 0.25
temp_iir_der = [0]

filtrertLysDerivert = [0]

def FIRfilter(temp, m):
    """
    Funksjon som beregner FIR-filtrering
    """
    newlist = []
    temp_fir = []
    for i in range(len(temp)):
        newlist.append(temp[i])
        if i < m-1:
            temp_fir.append( (1/(1+i)) * sum(newlist[-m:]) )
        else:
            temp_fir.append((1/m) * sum(newlist[-m:]))
    return temp_fir

def IIRfilter(temp, alpha):
    """
    Funksjon som beregner IIR-filtrering
    """
    # temp_iir er rekursiv, legger til første
    # temperaturmåling som basetilfelle
    temp_iir = [temp[0]]
    for i in range(len(temp)):
        temp_iir.append(alpha*temp[i] + (1-alpha)*temp_iir[i])
    return temp_iir


def Derivasjon(light, timestep):
    #Derivation of input values
    #Returns the derivative of the secant between two points
    #FunctionValues = [x, x+a]
    if timestep[-1] == 0 or len(light) < 2:
        sekant = 0
    else:
        sekant = ((light[-1]-light[-2])/(timestep[-2]))
    return sekant



def MathCalculations():
    """
    Legger til beregnede verdier basert på målte verdier.
    Legger også til tidsskrittet 'ts'.

    Parametre:
    flow - flow
    ts - tidsskritt
    volume - volum
    light - lys
    time - tid

    Andre målte og beregnede verdier må legges til som parametre
    hvis de skal bruke
    """
    if online: # hvis online modus
        # derivert
        if len(time) == 1:
            temp_iir[0] = light[-1]

        if len(time) > 1:
            # ts
            ts.append(time[-1] - time[-2])
            # lys Derivert
        
            lysDerivert.append(Derivasjon(light,ts))
            # Filtrert lys derivert
            #filtrertLysDerivert.append(derivasjon(filtrertLys[-1],filtrertLys[0],ts))
        
        # FIR-filter
        #temp_fir.append((1/m) * sum(light[-m:]))

        #temp_fir_der.append(derivasjon(temp_fir,ts))

        # IIR-filter

            temp_iir.append((alpha*light[-1]) + ((1-alpha)*temp_iir[-1]))
        
            temp_iir_der.append(Derivasjon(temp_iir,ts))


    #else: # hvis offline modus


        # flow
        #for lightvalue in light[1:]:
        #    flow.append(lightvalue - light[0])
        # timestep
        #for i in range(len(time)):
        #    if i+1 < len(time):
        #        ts.append(time[i+1] - time[i])
        # volume:
        #for i in reversed(range(len(light))):
        #    volume.append(volume[len(volume)-1] + flow[i-1] * ts[i-1])
        #volume.pop()

# Define figure and subplots.
fig, (ax1, ax3) = plt.subplots(nrows=2, ncols=1, sharex=True)


def plotData():
    # Clear subplots to prepare for next frame.
    ax1.cla()
    #ax2.cla()
    ax3.cla()

    #temp_fir_der_fir = FIRfilter(temp_fir_der, m)
    #temp_iir_der_iir = IIRfilter(temp_iir_der, alpha)
   
    #print("fir:" + str(temp_fir[-1]))
    #print("fir_der"+ str(temp_fir_der[-1]))

    #print("fir_der_fir"+ str(temp_fir_der_fir[-1]))

    #ax1.plot(time[0:-1], light[1:-1])
    #ax1.plot(time[0:-1], temp_fir[1:-1], label="Temperatur FIR-filtrert, m = " + str(m))
    ax1.plot(time[0:-1], temp_iir[0:-1], '-', label="Temperatur IIR-filtrert, alfa = " + str(alpha))
    ax1.set_title('Light IIR Filtrert')
    #ax1.legend(loc="lower right")
    ax1.set_xlabel('Tid [sek]')

    #ax2.plot(time[0:-1], lysDerivert[0:-1])
    #ax2.set_title('Lys Derivert')
    #ax2.set_xlabel('Tid [sek]')


    #ax3.plot(time[0:-1], temp_fir_der_fir[1:-1], label="Temperatur FIR-filtrert derivert, m = " + str(m))
    ax3.plot(time[0:-1], temp_iir_der[0:-1], label="Temperatur IIR-filtrert derivert, alfa = " + str(alpha))
    ax3.set_title('Lys IIR Filtrert Derivert')
    ax3.axhline(y=0, color="red", linewidth=0.2)
    ax3.set_xlabel('Tid [sek]')


    #Sax3.legend(loc="lower right")











def live(i):
    # Recieve data from EV3.
    try:
        data = sock.recv(1024)
    except:
        print("Lost connection to EV3")
        livePlot.event_source.stop()
        return
    # If the data recieved is the end signal, freeze plot.
    if data == b"end":
        print("Recieved end signal")
        livePlot.event_source.stop()
        return

    # If data is not the end signal, decode it.
    print(data)
    try:
        data = json.loads(data)
    except:
        print("Lost connection to EV3")
        livePlot.event_source.stop()
        return

    # Append the recieved data to the lists.
    light.append(data["light"])
    time.append(data["time"])
    # denne funksjonen tar seg av flow og volume
    MathCalculations()

    # Ensure no more data is sent before this set is handled
    sock.send(b"ack")

    # Plot the data in the lists.
    plotData()


def offline(filename):
    # Read from file row by row and append data to lists.
    with open(filename) as f:
        for row in f:
            row = row.split(",")
            time.append(float(row[0]))
            light.append(int(row[1]))

    MathCalculations()

    # Plot data in lists.
    plotData()
    # Set plot layout and show plot.
    fig.set_tight_layout(True)
    plt.show()


if online:
    # If online, setup socket object and connect to EV3.
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        addr = (EV3_IP, 8070)
        print("Attempting to connect to {}".format(addr))
        sock.connect(addr)
        data = sock.recv(1024)
        if data == b"ack":
            print("Connection established")
        else:
            print("no ack")
            sys.exit()
    except socket.timeout:
        print("failed")
        sys.exit()

    # Start live plotting.
    livePlot = FuncAnimation(fig, live, interval=1)

    # Set plot layout and show plot.
    fig.set_tight_layout(True)
    plt.show()
else:
    # If offline, plot from file defined by filename.
    offline(filename)
    