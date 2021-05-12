import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import socket
import sys
import json

# Tmp data
tmp = ""

# Set online flag, ip address and filename.
online = False
EV3_IP = "169.254.70.247"
filename = "measurements.txt"

# Initialize lists.
light = []
time = []
ts = [0]
flow = []
volume = [0]

# Define figure and subplots.
fig, ax = plt.subplots(nrows=3, ncols=1, sharex=True)


def initPlot():
    global ax
    ax[0].set_title('Light')
    ax[1].set_title('Flow')
    ax[2].set_title('Volume')
    ax[2].set_xlabel('Time [sec]')


def plotData():
    ax[0].plot(time[0:-1], light[0:-1], 'b')

    ax[1].plot(time[0:-1], flow[0:-1], 'b')

    ax[2].plot(time[0:-1], volume[0:-1], 'b')


def stopPlot():
    try:
        livePlot.event_source.stop()
    except:
        pass
    # Alt som skal regnes ut når programmet stoppes legges her


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

    # Flow
    flow.append(light[-1] - light[0])
    if len(time) > 1:
        # ts
        ts.append(time[-1] - time[-2])
        # Volume - "light integrated"
        volume.append(volume[-1] + flow[-2] * ts[-1])


def appendLists(datum):
    if isinstance(datum, dict):
        light.append(datum["light"])
        time.append(datum["time"])
    else:
        time.append(float(datum[0]))
        light.append(int(datum[1]))


# Ikke endre under her


def offline(filename):
    # Read from file row by row and append data to lists.
    with open(filename) as f:
        for row in f:
            datum = row.split(",")
            appendLists(datum)
            MathCalculations()

    # Plot data in lists.
    initPlot()
    plotData()

    stopPlot()
    # Set plot layout and show plot.

    fig.set_tight_layout(True)  # mac


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

        for datum in data:
            if datum == b'':
                continue
            # If the data recieved is the end signal, freeze plot.
            elif datum == b"end":
                print("Recieved end signal")
                stopPlot()
                return
            try:
                datum = json.loads(datum)
            except:
                # Save incomplete data
                tmp = datum
                continue
            # Append the recieved data to the lists.
            appendLists(datum)
            MathCalculations()
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
