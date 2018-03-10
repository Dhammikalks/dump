# Instead of moving a distribution, move (and modify) it using a convolution.
# 06_b_convolve_distribution
# Claus Brenner, 26 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)

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
    arena = (0,2000)

    # Move 3 times by 20.
    moves = [20]*100

    # Start with a known position: probability 1.0 at position 10.
    position = Distribution.unit_pulse(10)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Now move and plot.
    move_distribution = Distribution.triangle(10, 2)
    for m in moves:
        print(m)
        move_distribution = move(move_distribution,m)
        position = convolve(position, move_distribution)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             linestyle='steps')

    ylim(0.0, 1.1)
    show()
