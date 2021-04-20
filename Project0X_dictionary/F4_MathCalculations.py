def MathCalculations(measurements):
    # Flow
    measurements["flow"].append(measurements["light"][-1] -
                                measurements["light"][0])
    if len(measurements["flow"]) > 1:
        # ts
        measurements["ts"].append(measurements["time"][-1] -
                                  measurements["time"][-2])
        # light integrated
        measurements["volume"].append(
            measurements["volume"][-1] +
            measurements["flow"][-2] *
            measurements["ts"][-1])
