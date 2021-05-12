import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import socket
import sys
import json

# Set online flag, ip address and filename.
online = True
EV3_IP = "169.254.251.207"
filename = "measurements.txt"

# Initialize lists.
light = []
time = []
ts = []
flow = []
volume = []

# Define figure and subplots.
fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1, sharex=True)


def plotData():
    # Clear subplots to prepare for next frame.
    ax1.cla()
    ax2.cla()
    ax3.cla()

    ax1.plot(time[0:-1], light[0:-1])
    ax1.set_title('Light')

    ax2.plot(time[0:-1], flow[0:-1])
    ax2.set_title('Flow')

    ax3.plot(time[0:-1], volume[0:-1])
    ax3.set_title('Volume')
    ax3.set_xlabel('Time [sec]')


def live(i):
    # Recieve data from EV3.
    data = sock.recv(1024)

    # If the data recieved is the end signal, freeze plot.
    if data == b"end":
        print("Recieved end signal")
        livePlot.event_source.stop()
        return

    # If data is not the end signal, decode it.
    data = json.loads(data)

    # Append the recieved data to the lists.
    light.append(data["light"])
    time.append(data["time"])
    flow.append(data["flow"])
    volume.append(data["volume"])

    # Ensure no more data is sent before this set is handled
    sock.send(b"ack")

    # Plot the data in the lists.
    plotData()


def offline(filename):
    # Read from file row by row and append data to lists.
    with open(filename) as f:
        for row in f:
            row = row.split(",")
            light.append(int(row[0]))
            time.append(float(row[1]))
            flow.append(int(row[2]))
            volume.append(float(row[3]))

    # Plot data in lists.
    plotData()
    # Set plot layout and show plot.
    plt.tight_layout()
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
    plt.tight_layout()
    plt.show()
else:
    # If offline, plot from file defined by filename.
    offline(filename)
