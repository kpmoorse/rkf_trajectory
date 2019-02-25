import numpy
import matplotlib.pyplot as plt
import random


class Trajectory(object):

    # Initialize parameters
    def __init__(self, t):
        self.t = t
        self.dt = numpy.diff(t)[0]

        self.frequency = t * 0
        self.phase = t * 0
        self.position = t * 0

    # Set frequency profile and propagate through parameters
    def set_frequency(self, frequency, amplitude=1):
        self.frequency = frequency
        self.calc_position(amplitude=amplitude)

    # Return a constant frequency profile
    def sinusoid(self, freq):
        frequency = self.t * 0 + freq
        return 2*numpy.pi*frequency

    # Return a stepwise constant frequency profile
    def stepwise(self, freq_list, rnd=False):
        if rnd:
            random.shuffle(freq_list)
        step_ix = numpy.insert(numpy.cumsum([int(freq[0] / self.dt) for freq in freq_list]), 0, 0)
        frequency = self.t * 0
        for i in range(len(freq_list)):
            frequency[step_ix[i]:step_ix[i + 1]] = freq_list[i][1]
        return 2*numpy.pi*frequency

    # Return a triangular ramp frequency profile
    def freq_ramp(self, max_freq):
        tau = self.t[-1]
        frequency = max_freq * (1 - numpy.abs(2 * self.t / tau - 1))
        return 2*numpy.pi*frequency

    # Calculate phase by integrating frequency
    def calc_phase(self):
        self.phase = numpy.cumsum(self.frequency) * self.dt

    # Calculate position by
    def calc_position(self, fun='cos', amplitude=1):
        self.calc_phase()
        if fun.lower() in ['cos', 'cosine']:
            self.position = amplitude * numpy.cos(self.phase)
        elif fun.lower() in ['sin', 'sine']:
            self.position = amplitude * numpy.sin(self.phase)

    # Generate plots to visualize parameters
    def visualize(self):
        plt.subplot(3, 1, 1)
        plt.plot(self.t, self.frequency)
        plt.ylabel('Freq (rad/s)')
        plt.xticks([], [])

        plt.subplot(3, 1, 2)
        plt.plot(self.t, self.phase)
        plt.ylabel('Phase (rad)')
        plt.xticks([], [])

        plt.subplot(3, 1, 3)
        plt.plot(self.t, self.position)
        plt.ylabel('Position (deg)')
        plt.xlabel('Time (s)')

        plt.show()


# t = numpy.arange(0, 120, 0.01)
# trj = Trajectory(t)
# trj.set_frequency(trj.stepwise([[20, 0.5*(x+1)] for x in range(6)]), 80)
# trj.visualize()
