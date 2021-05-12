import json


def SendData(robot, measurements, online):
    data = {}
    for key in measurements:
        if key == "zeroTime" or key == "ts":
            continue

        data[key] = (measurements[key][-1])

    if online:
        msg = json.dumps(data)
        robot["connection"].send(msg)

        if not robot["connection"].recv(3) == b"ack":
            print("No data ack")

    robot["out_file"].write("{},{},{},{}\n".format(
        data["light"],
        data["time"],
        data["flow"],
        data["volume"]))
