import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from MineFunksjoner import EulerForward
import numpy as np
import socket
import sys
import json
from time import perf_counter

# Set online flag, ip address and filename.
online = False
EV3_IP = "169.254.165.29"
filename = "driveOffline.txt", "driveOffline2.txt"

# Define figure and subplots.
fig, ax = plt.subplots(nrows=2, ncols=2, sharex=True)

# Tmp data
tmp = ""

# Initialize variables.
ylim = 0

light = []
mean = 0
std = 0

light2 = []
mean2 = 0
std2 = 0

# Testing
msgCount = 0


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
    global mean, mean2, std, std2, ylim
    mean = np.mean(light)
    mean2 = np.mean(light2)
    std = np.std(light)
    std2 = np.std(light2)
    ylim = 250


def initPlot():
    ax[0, 0].set_title('Remi, mean = {:.1f}, std = {:.1f}'.format(mean, std))
    ax[0, 1].set_title('Per, mean = {:.1f}, std = {:.1f}'.format(mean2, std2))


def plotData():
    # Histogram
    ax[0, 0].hist(light)
    # Mean
    ax[0, 0].plot((mean, mean), (0, ylim))
    # Standard deviation
    ax[0, 0].plot((mean - std, mean + std), (ylim - 10, ylim - 10))
    # Axis limits
    ax[0, 0].set(xlim=(0, 70), ylim=(0, ylim))

    # Histogram
    ax[0, 1].hist(light2)
    # Mean
    ax[0, 1].plot((mean2, mean2), (0, ylim))
    # Standard deviation
    ax[0, 1].plot((mean2 - std2, mean2 + std2), (ylim - 10, ylim - 10))
    # Axis limits
    ax[0, 1].set(xlim=(0, 70), ylim=(0, ylim))

    ax[1, 0].set(xlim=(0, 70), ylim=(0, ylim))

    ax[1, 1].set(xlim=(0, 70), ylim=(0, ylim))


def live(i):
    global msgCount, tmp
    recvTime = perf_counter()
    # Recieve data from EV3.
    try:
        data = sock.recv(1024)
        print("Data recieved: ")
        print(data, "\n")
    except Exception as e:
        print(e)
        print("Lost connection to EV3")
        stopPlot()
        return

    print("Recv time: ", perf_counter() - recvTime)

    # If data is not the end signal, decode it.
    # print(data)
    try:
        data = data.split(b"?")

        # Reconstruct split data
        if tmp != "":
            data[0] = tmp + data[0]
            tmp = ""
            print("Reconstructed data: ", data[0])

        print("Data packets: ", len(data))
        for datum in data:
            try:
                if datum == b'':
                    continue
                # If the data recieved is the end signal, freeze plot.
                elif datum == b'end':
                    print("Recieved end signal")
                    print("Message count: ", msgCount)
                    stopPlot()
                    return
                # Append the recieved data to the lists.
                datum = json.loads(datum)
                appendLists(datum)
                MathCalculations()
                msgCount += 1
            except Exception as e:
                # Save incomplete data
                tmp = datum
                print("Datum error: \n", e, "\nDatum: ", datum)
    except Exception as e:
        print(e)
        print("Data error")
        print("Message count: ", msgCount)
        stopPlot()
        return

    print("Total recv time: ", perf_counter() - recvTime)

    runTime = perf_counter()

    # Plot the data in the lists.
    plotData()

    print("Plot time: ", perf_counter() - runTime)
    print("\n")


def appendLists(datum):
    light.append(datum["light"])
    initLight.append(light[0])
    time.append(datum["time"])
    powerB.append(datum["powerB"])
    powerC.append(datum["powerC"])


def stopPlot():
    try:
        livePlot.event_source.stop()
    except:
        pass

    # Alt som skal regnes ut når programmet stoppes legges her
    print("Referanse: ", light[0])
    print("Middelverdi: {:.1f}".format(np.mean(light)))
    print("|Referanse - middelverdi|: {:.1f}".format(
        abs(light[0] - np.mean(light))))
    print("Standardavik: {:.1f}".format(np.std(light)))
    print("Antall målinger: ", len(light))


def offline(filename):
    # Read from file row by row and append data to lists.
    with open(filename[0]) as f:
        for row in f:
            row = row.split(",")
            light.append(int(row[1]))

    with open(filename[1]) as f:
        for row in f:
            row = row.split(",")
            light2.append(int(row[1]))

    # Perform calculations
    MathCalculations()

    # Plot data in lists.
    initPlot()
    plotData()

    stopPlot()

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
    livePlot = FuncAnimation(fig, live, init_func=initPlot(), interval=10)

    # Set plot layout and show plot.
    fig.set_tight_layout(True)
    plt.show()
else:
    # If offline, plot from file defined by filename.
    offline(filename)
