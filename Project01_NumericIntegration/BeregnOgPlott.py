# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import socket
import sys
import json

# Set online flag, ip address and filename.
online = True
EV3_IP = "169.254.243.41"
filename = "measurements.txt"

# Initialize lists.
light = []
time = []
ts = [0]
flow = [0]
volume = [0]

#
def EulerForward( IntValuOld,  FunctionValue ,  TimeStep):
    IntValueNew =  IntValuOld + (FunctionValue * TimeStep)
    return IntValueNew
#


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
        # Flow
        flow.append(light[-1] - light[0])
        if len(time) > 1:
            # ts
            ts.append(time[-1] - time[-2])
            # Volume - "light integrated" Euler metoden
            volume.append(EulerForward(volume[-1],flow[-2],ts[-1]))

    else: # hvis offline modus
        # flow
        for lightvalue in light[1:]:
            flow.append(lightvalue - light[0])
        # timestep
        for i in range(len(time)):
            if i+1 < len(time):
                ts.append(time[i+1] - time[i])
        # volume:
        for i in reversed(range(len(light))):
            volume.append(volume[len(volume)-1] + flow[i-1] * ts[i-1])
        volume.pop()

# Define figure and subplots.
fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)


def plotData():
    # Clear subplots to prepare for next frame.
    ax1.cla()
    ax2.cla()
    ax3.cla()

    ax1.plot(time[0:-1], light[0:-1])
    ax1.set_title('Light')

    print(len(flow))
    print(len(time))
    # for å printe offline, bruk flow[0:-1]
    ax2.plot(time[0:-1], flow[1:-1])
    ax2.set_title('Flow')

    ax3.plot(time[0:-1], volume[0:-1])
    ax3.set_title('Volume')
    ax3.set_xlabel('Time [sec]')


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
   
    fig.set_tight_layout(True)# mac
    
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
    livePlot = FuncAnimation(fig, live, interval=10)

    # Set plot layout and show plot.
    fig.set_tight_layout(True)
    plt.show()
else:
    # If offline, plot from file defined by filename.
    offline(filename)