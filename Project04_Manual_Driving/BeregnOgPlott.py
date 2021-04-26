import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from MineFunksjoner import EulerForward
import numpy as np
import socket
import sys
import json
from time import perf_counter

# Set online flag, ip address and filename.
online = True
EV3_IP = "169.254.17.214"
filename = "measurements.txt"

# Define figure and subplots.
fig, ax = plt.subplots(nrows=4, ncols=3, sharex=True)

# Initialize lists.
light = []
initLight = []
e = []
time = []
ts = [0]
joyForward = []
joySide = []
powerB = [0]
powerC = [0]

IAE = [0]
ITAE = [0]
MAE = [0]
TV_B = [0]
TV_C = [0]

meanLight = []
stdLight = []
varLight = []

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
    # if online:  # hvis online modus
    #     # Flow
    #     flow.append(light[-1] - light[0])
    #     if len(time) > 1:
    #         # ts
    #         ts.append(time[-1] - time[-2])
    #         # Volume - "light integrated"
    #         volume.append(volume[-1] + flow[-2] * ts[-1])

    # else:  # hvis offline modus
    #     # flow
    #     for lightvalue in light[1:]:
    #         flow.append(lightvalue - light[0])
    #     # timestep
    #     for i in range(len(time)):
    #         if i+1 < len(time):
    #             ts.append(time[i+1] - time[i])
    #     # volume:
    #     for i in reversed(range(len(light))):
    #         volume.append(volume[len(volume)-1] + flow[i-1] * ts[i-1])
    #     volume.pop()
    if len(time) < 2:
        ts.append(time[-1])
    else:
        ts.append(time[-1] - time[-2])
    e.append(light[0] - light[-1])
    IAE.append(EulerForward(IAE[-1], abs(e[-1]), ts[-1]))
    ITAE.append(EulerForward(ITAE[-1], abs(e[-1]), time[-1]))
    MAE.append(IAE[-1] / len(light))

    TV_B.append(EulerForward(TV_B[-1], abs(powerB[-1] - powerB[-2]), ts[-1]))
    TV_C.append(EulerForward(TV_C[-1], abs(powerC[-1] - powerC[-2]), ts[-1]))

    meanLight.append(np.mean(light))
    stdLight.append(np.std(light))
    varLight.append(np.var(light))


def initPlot():
    global ax
    ax[0, 0].set_title('Lys(b) mot Referanse(r)')
    ax[0, 1].set_title('Avvik')
    ax[0, 2].set_title('joyForward(r), joySide(b)')
    ax[1, 0].set_title('Middelverdi(b) av lys mot Referanse(r)')
    ax[1, 1].set_title('Integral Absolute Error(IAE)')
    ax[1, 2].set_title('Power B(r), Power C(b)')
    ax[2, 0].set_title('Standardavvik av lys')
    ax[2, 1].set_title('Integral Time Absolute Error(ITAE)')
    ax[2, 2].set_title('Total Variation(TV) B(r), C(b)')
    ax[3, 0].set_title('Varians av lys')
    ax[3, 1].set_title('Mean Absolute Error(MAE)')

    ax[2, 2].set_xlabel('Time [sec]')
    ax[3, 0].set_xlabel('Time [sec]')
    ax[3, 1].set_xlabel('Time [sec]')


def plotData():
    # Clear subplots to prepare for next frame.
    # clearTime = perf_counter()
    # for row in range(len(ax)):
    #     for col in range(len(ax[0])):
    #         ax[row, col].cla()
    # print("Clear time: ", perf_counter() - clearTime)

    ax[0, 0].plot(time[0:-1], initLight[0:-1], 'r', linewidth=2)
    ax[0, 0].plot(time[0:-1], light[0:-1], 'b-*')

    ax[0, 1].plot(time[0:-1], e[0:-1], 'b')

    ax[0, 2].plot(time[0:-1], joyForward[0:-1], 'r')
    ax[0, 2].plot(time[0:-1], joySide[0:-1], 'b')

    ax[1, 0].plot(time[0:-1], initLight[0:-1], 'r', linewidth=2)
    ax[1, 0].plot(time[0:-1], meanLight[0:-1], 'b')
    ax[1, 1].plot(time[0:-1], IAE[1:-1], 'b')

    ax[1, 2].plot(time[0:-1], powerB[1:-1], 'r')
    ax[1, 2].plot(time[0:-1], powerC[1:-1], 'b')
    ax[2, 0].plot(time[0:-1], stdLight[0:-1], 'b')

    ax[2, 1].plot(time[0:-1], ITAE[1:-1], 'b')

    ax[2, 2].plot(time[0:-1], TV_B[1:-1], 'r')
    ax[2, 2].plot(time[0:-1], TV_C[1:-1], 'b')

    ax[3, 0].plot(time[0:-1], varLight[0:-1], 'b')

    ax[3, 1].plot(time[0:-1], MAE[1:-1], 'b')


def live(i):
    global msgCount
    recvTime = perf_counter()
    # Recieve data from EV3.
    try:
        data = sock.recv(1024)
        print(data)
    except:
        print("Lost connection to EV3")
        livePlot.event_source.stop()
        return

    print("Recv time: ", perf_counter() - recvTime)

    # If data is not the end signal, decode it.
    # print(data)
    try:
        data = data.split(b"?")
        for datum in data:
            if datum == b'':
                continue
            # If the data recieved is the end signal, freeze plot.
            elif datum == b'end':
                print("Recieved end signal")
                print("Message count: ", msgCount)
                livePlot.event_source.stop()
                return
            # Append the recieved data to the lists.
            datum = json.loads(datum)
            appendLists(datum)
            MathCalculations()
            msgCount += 1
    except:
        print("Lost connection to EV3")
        print("Message count: ", msgCount)
        livePlot.event_source.stop()
        return

    print("Total recv time: ", perf_counter() - recvTime)

    runTime = perf_counter()

    # Ensure no more data is sent before this set is handled
    # sock.send(b"ack")

    # Plot the data in the lists.
    plotData()

    print("Math and plot time: ", perf_counter() - runTime)
    print("\n")


def appendLists(datum):
    light.append(datum["light"])
    initLight.append(light[0])
    time.append(datum["time"])
    joyForward.append(datum["joyForward"])
    joySide.append(datum["joySide"])
    powerB.append(datum["powerB"])
    powerC.append(datum["powerC"])


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
    fig.set_tight_layout()
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
