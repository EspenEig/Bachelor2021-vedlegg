import json


def SendData(robot, measurements, online):
    # Ensure data and dataString are empty.
    data = {}
    dataString = ""
    # For every measurement other than zeroTime and ts,
    # add it to data and dataString.
    for key in measurements:
        if key == "zeroTime" or key == "ts":
            continue

        data[key] = (measurements[key][-1])
        dataString += "," + str(data[key])

    # Write dataString to file, excepting the first comma.
    robot["out_file"].write(dataString[1:])

    # Send data to pc
    if online:
        msg = json.dumps(data)
        robot["connection"].send(msg)

        # Wait until pc has acknowledged data before continuing.
        if not robot["connection"].recv(3) == b"ack":
            print("No data ack")
