def CalculateAndSetMotorPower(robot, measurements):
    # Pybricks siden har  alternative funksjoner som kan utnyttes
    # https://pybricks.github.io/ev3-micropython/ev3devices.html
    # for eksempel vil run_time() kjøre motoren med en konstant
    # hastighet for en gitt tid (i millisekund)

    # Parametre for beregning til motorpådrag
    a = 1
    b = 1

    # Her brukes siste lysmåling for å beregne motorpådrag.
    lys = measurements["light"][-1]

    # Lagrer pådragene i measurements dictionaryen.
    measurements["motorPaadragA"] = lys*a*b
    # measurements["motorPaadragB"] = lys*a
    # measurements["motorPaadragC"] = lys*a
    # measurements["motorPaadragD"] = lys*a

    # Sett hastigen på motorene.
    robot["motorA"].run(measurements["motorPaadragA"])
    # robot["motorB"].run(measurements["motorPaadragB"])
    # robot["motorC"].run(measurements["motorPaadragC"])
    # robot["motorD"].run(measurements["motorPaadragD"])
