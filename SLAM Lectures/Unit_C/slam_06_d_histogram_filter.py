# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)



# --->>> Copy your convolve(a, b) and multiply(a, b) functions here.
def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""
    if a.offset > b.offset:
        start = a.offset
    else:
        start = b.offset
    if a.stop() > b.stop():
        end = b.stop()
    else:
        end = a.stop()
    array = [0]
    c = start
    array.append(0)
    while c < end+1:
        if a.value(c) * b.value(c) > 0:
            array.append((a.value(c)*b.value(c)))
        c += 1
    array.append(0)


    distribution = Distribution(start,array)
    distribution.normalize()



    # --->>> Put your code here.

    return distribution # Modify this to return your result.


def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""
    #print b.offset
    offset_a = a.offset
    offset_b = b.offset
    #print a.offset
    values = [0]*(len(a.values)+len(b.values)-1)
    for i in xrange(len(a.values)):
        for j in xrange(len(b.values)):
            values[j+i] += a.value(offset_a+i)*b.value(offset_b+j)
    #print values
    return Distribution(offset_b,values)  # Replace this by your own result.


if __name__ == '__main__':
    arena = (0,500)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 10

    # Measurement data. Assume (for now) that the measurement data
    # is correct. - This code just builds a cumulative list of the controls,
    # plus the start position.
    p = start_position
    measurements = []
    for c in controls:
        p += c
        measurements.append(p)
    print measurements

    # This is the filter loop.
    for i in xrange(len(controls)):
        # Move, by convolution. Also termed "prediction".
        control = Distribution.triangle(20, 10)
        position = move(position,controls[i])
        position = convolve(control,position)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b', linestyle='steps')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 10)
        plot(measurement.plotlists(*arena)[0], measurement.plotlists(*arena)[1],
             color='y', linestyle='steps')
        position = multiply(position, measurement)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
            color='r', linestyle='steps')

    show()
