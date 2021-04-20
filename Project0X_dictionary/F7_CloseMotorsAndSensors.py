def CloseMotorsAndSensors(robot, online):
    robot["joystick"]["in_file"].close()
    robot["outfile"].close()
    if online:
        robot["connection"].send(b"end")
        robot["connection"].close()
        robot["sock"].close()
