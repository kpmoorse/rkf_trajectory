import numpy

def phase(frequency, dt):
    return numpy.cumsum(frequency)*dt