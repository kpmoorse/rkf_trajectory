import numpy
import matplotlib.pyplot as plt


class Trajectory(object):

    def __init__(self, t):
        self.t = t
        self.dt = numpy.diff(t)[0]

        self.frequency = t * 0
        self.phase = t * 0
        self.position = t * 0

    def set_frequency(self, frequency, amplitude=1):
        self.frequency = amplitude * frequency
        self.calc_position()

    def sinusoid(self, freq):
        frequency = self.t * 0 + freq
        return frequency

    def stepwise(self, freq_list):
        step_ix = numpy.insert(numpy.cumsum([int(freq[0] / self.dt) for freq in freq_list]), 0, 0)
        frequency = self.t * 0
        for i in range(len(freq_list)):
            frequency[step_ix[i]:step_ix[i + 1]] = freq_list[i][1]
        return frequency

    def freq_ramp(self, max_freq):
        tau = self.t[-1]
        frequency = max_freq * (1 - numpy.abs(2 * self.t / tau - 1))
        return frequency

    def calc_phase(self):
        self.phase = numpy.cumsum(self.frequency) * self.dt

    def calc_position(self, fun='cos'):
        self.calc_phase()
        if fun.lower() in ['cos', 'cosine']:
            self.position = numpy.cos(self.phase)
        elif fun.lower() in ['sin', 'sine']:
            self.position = numpy.sin(self.phase)

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


def phase(frequency, dt):
    return numpy.cumsum(frequency)*dt


# t = numpy.arange(0, 30, 0.1)
# trj = Trajectory(t)
# trj.set_frequency(trj.freq_ramp(5))
# trj.visualize()
