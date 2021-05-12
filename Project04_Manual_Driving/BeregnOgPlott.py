import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from MineFunksjoner import EulerForward
import numpy as np
import socket
import sys
import json

# Set online flag, ip address and filename.
online = False
EV3_IP = "169.254.165.29"
filename = "driveOffline2.txt"

# Define figure and subplots.
fig, ax = plt.subplots(nrows=3, ncols=2, sharex=True)

# Initialize lists.
light = []
initLight = []
e = []
time = []
ts = [0]
powerB = [0]
powerC = [0]

IAE = [0]
MAE = [0]
TV_B = [0]
TV_C = [0]

# Tmp data
tmp = ""


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
    if len(time) < 2:
        ts.append(time[-1])
    else:
        ts.append(time[-1] - time[-2])
    e.append(light[0] - light[-1])
    IAE.append(EulerForward(IAE[-1], abs(e[-1]), ts[-1]))
    MAE.append(IAE[-1] / len(light))

    TV_B.append(EulerForward(TV_B[-1], abs(powerB[-1] - powerB[-2]), ts[-1]))
    TV_C.append(EulerForward(TV_C[-1], abs(powerC[-1] - powerC[-2]), ts[-1]))


def initPlot():
    global ax
    ax[0, 0].set_title('Lys(b) mot Referanse(r)')
    ax[0, 1].set_title('Avvik')
    ax[1, 0].set_title('Power B(r), Power C(b)')
    ax[1, 1].set_title('Integral Absolute Error(IAE)')
    ax[2, 0].set_title('Total Variation(TV) B(r), C(b)')
    ax[2, 1].set_title('Mean Absolute Error(MAE)')

    ax[2, 0].set_xlabel('Time [sec]')
    ax[2, 1].set_xlabel('Time [sec]')


def plotData():
    # Lys mot referanseverdi
    ax[0, 0].plot(time[0:-1], initLight[0:-1], 'r', linewidth=2)
    ax[0, 0].plot(time[0:-1], light[0:-1], 'b-')

    # Avvik
    ax[0, 1].plot(time[0:-1], e[0:-1], 'b')

    # Pådrag for motorene
    ax[1, 0].plot(time[0:-1], powerB[1:-1], 'r')
    ax[1, 0].plot(time[0:-1], powerC[1:-1], 'b')

    # Integral Absolute Error(IAE)
    ax[1, 1].plot(time[0:-1], IAE[1:-1], 'b')

    # Total Variation(TV) for motorene
    ax[2, 0].plot(time[0:-1], TV_B[1:-1], 'r')
    ax[2, 0].plot(time[0:-1], TV_C[1:-1], 'b')

    # Mean Absolute Error(MAE)
    ax[2, 1].plot(time[0:-1], MAE[1:-1], 'b')


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
    print("Kjøretid [sek]: {:.1f}".format(time[-1]))
    print("IAE: {:.1f}".format(IAE[-1]))
    print("MAE: {:.1f}".format(MAE[-1]))
    print("TV_B: {:.1f}".format(TV_B[-1]))
    print("TV_C: {:.1f}".format(TV_C[-1]))
    print("Middelverdi av Ts [sek]: {:.3f}".format(np.mean(ts)))
    print("Antall målinger: ", len(light))


def offline(filename):
    # Read from file row by row and append data to lists.
    with open(filename) as f:
        for row in f:
            row = row.split(",")
            time.append(float(row[0]))
            light.append(int(row[1]))
            initLight.append(light[0])
            powerB.append(float(row[2]))
            powerC.append(float(row[3]))

            # Perform calculations
            MathCalculations()

    # Plot data in lists.
    initPlot()
    plotData()

    stopPlot()

    # Set plot layout and show plot.
    fig.set_tight_layout(True)
    plt.show()


def appendLists(datum):
    light.append(datum["light"])
    initLight.append(light[0])
    time.append(datum["time"])
    powerB.append(datum["powerB"])
    powerC.append(datum["powerC"])


def live(i):
    global tmp
    # Recieve data from EV3.
    try:
        data = sock.recv(1024)
    except Exception as e:
        print(e)
        print("Lost connection to EV3")
        stopPlot()
        return

    try:
        data = data.split(b"?")

        # Reconstruct split data
        if tmp != "":
            data[0] = tmp + data[0]
            tmp = ""
            print("Reconstructed data: ", data[0])

        for datum in data:
            try:
                if datum == b'':
                    continue
                # If the data recieved is the end signal, freeze plot.
                elif datum == b'end':
                    print("Recieved end signal")
                    stopPlot()
                    return
                # Append the recieved data to the lists.
                datum = json.loads(datum)
                appendLists(datum)
                MathCalculations()
            except Exception as e:
                # Save incomplete data
                tmp = datum
                print("Datum error: \n", e, "\nDatum: ", datum)
    except Exception as e:
        print(e)
        print("Data error")
        stopPlot()
        return

    # Plot the data in the lists.
    plotData()


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
