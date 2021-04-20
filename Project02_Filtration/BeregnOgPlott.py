import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import socket
import sys
import json

# Set online flag, ip address and filename.
online = False
EV3_IP = "192.168.137.3"
filename = "measurements.txt"

# temperatur, tid, k, tidsskritt 
temp = []
time = []
k = 0
ts = [0]
avgts = 0

# FIR-filtrering
temp_fir = []
# bestemmer "mengden" filtrering
m = 1000

# IIR-filtrering
temp_iir = []
# "vekten"
alpha = 0.0025

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


def MathCalculations():
    """
    Legger til beregnede verdier basert på målte verdier.
    Legger også til tidsskrittet 'ts'.
    """
    if online: # hvis online modus
        # tidsskrittet
        if len(time) > 1:
            # ts
            ts.append(time[-1] - time[-2])

        # FIR-filter
        temp_fir.append((1/m) * sum(temp[-m:]))

        # IIR-filter
        temp_iir.append(alpha*temp[-1] + (1-alpha)*temp_iir[-1])

    else: # hvis offline modus
        # timestep
        for i in range(len(time)):
            if i+1 < len(time):
                ts.append(time[i+1] - time[i])
        #print(sum(ts))
        #print(avgts)
        

        # FIR-filter
        newlist = []
        for i in range(len(temp)):
            newlist.append(temp[i])
            if i < m-1:
                temp_fir.append( (1/(1+i)) * sum(newlist[-m:]) )
            else:
                temp_fir.append((1/m) * sum(newlist[-m:]))

        # IIR-filter
        temp_iir.append(temp[0])
        for i in range(len(temp)):
            temp_iir.append(alpha*temp[i] + (1-alpha)*temp_iir[i])


# Define figure and subplots.
fig, ax1 = plt.subplots()

def plotData():
    # Clear subplots to prepare for next frame.
    ax1.cla()

    ax1.plot(time[0:-1], temp[0:-1], label="Temperatur")
    ax1.plot(time[0:-1], temp_fir[0:-1], label="Temperatur FIR-filtrert, m = " + str(m))
    ax1.plot(time[0:-1], temp_iir[1:-1], label="Temperatur IIR-filtrert, alfa = " + str(alpha))
    ax1.set_xlabel("Tid [sek]")
    ax1.set_ylabel("Temperatur [C]")
    plt.legend(loc="lower right")
    #ax1.legend()
    # print("testing")
    # ax1.set_title("tid")
    # plt.xlabel("Tid [sek]")
    # plt.ylabel("Temperatur [C]")
    # plt.title("Eksperiment gjennomført med gjennomsnittlig tidssteg lik: 2ms")


def live(i):
    # Recieve data from EV3.
    data = sock.recv(1024)

    # If the data recieved is the end signal, freeze plot.
    if data == b"end":
        print("Recieved end signal")
        livePlot.event_source.stop()
        return

    # If data is not the end signal, decode it.
    print(data)
    data = json.loads(data)

    # Append the recieved data to the lists.
    temp.append(data["light"])
    time.append(data["time"])
    k = data["tellevariabel"]
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
            temp.append(int(row[1]))
    MathCalculations()

    # Plot data in lists.
    plotData()
    # Plot title
    avgts = sum(ts)/len(ts)
    print(avgts)
    plt.title("Eksperiment utført med gjennomsnittlig tidssteg lik " + "{:.4f}".format(1000*avgts) + "ms")
    # Set plot layout and show plot.
    plt.tight_layout()
    plt.show()


if online:
    # If online, setup socket object and connect to EV3.
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        addr = (EV3_IP, 8070)
        print("Attempting to connect to {}.".format(addr))
        sock.connect(addr)
        data = sock.recv(1024)
        if data == b"ack":
            print("Connection established.")
        else:
            print("No ack.")
            sys.exit()
    except socket.timeout:
        print("Failed.")
        sys.exit()

    # Start live plotting.
    livePlot = FuncAnimation(fig, live, interval=10)
    # Set plot layout and show plot.
    fig.set_tight_layout(True)
    plt.show()
else:
    # If offline, plot from file defined by filename.
    offline(filename)