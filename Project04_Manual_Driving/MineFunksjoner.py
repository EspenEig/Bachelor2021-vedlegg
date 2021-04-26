def EulerForward(intValueOld, FunctionValue, timeStep):
    '''
    EulerForward Numerical integration
    EulerForward (IntValueOld, FunctionValue, TimeStep) use the
    following values from the previous discrete index k -1:
    - IntValueOld (the previous integral value)
    - FunctionValue (the function value to be integrated)
    - TimeStep (time difference between discrete index k and k -1)
    to calculate the integral value at discrete index k.
    '''
    return intValueOld + FunctionValue * timeStep
