#!/usr/bin/env python
from __future__ import print_function
import sys
import time
import scipy
import numpy as np
import matplotlib.pyplot as plt
# import trajectory
import argparse

from autostep_proxy import AutostepProxy

# Initialize argument parser
parser = argparse.ArgumentParser(description='Square-Fourier Trajectory')
parser.add_argument('-d', '--dur', dest='duration', type=float,
                    help='Trajectory duration (sec)')
parser.add_argument('-r', '--rep', dest='repetitions', type=float,
                    help='Number of repetitions (runtime = dur * rep)')
parser.add_argument('-f', '--frq', dest='frequency', type=float, nargs=2,
                    help='Pair of frequency endpoints')
parser.add_argument('-a', '--amp', dest='amplitude', type=float,
                    help='Scaled trajectory amplitude (deg)')
args = parser.parse_args()

autostep = AutostepProxy()

print()
print('* trajectory example')
print()

jog_params = {'speed': 200, 'accel': 500, 'decel': 500}
max_params = {'speed': 1000, 'accel': 10000, 'decel': 10000}

# num_cycle = 6
# period = 5.0
duration = args.duration if args.duration else 60  # sec
reps = args.repetitions if args.repetitions else 1
f = args.frequency if args.frequency else [1, 2]
A = args.amplitude if args.amplitude else 80

# Create trajectory
dt = AutostepProxy.TrajectoryDt
num_pts = int(duration/dt)
t = dt*scipy.arange(num_pts)


# Smoothly log-truncate large values in an array and scale
def lognorm(y, a=0, A=None):

    y = np.array(y).astype(float)
    b = max(np.abs(y))
    if not A:
        A = b

    vec = y.copy() / b
    lg = np.array(np.abs(vec) > a).astype(bool)
    vec[lg] = np.sign(y[lg]) * (a + np.log(abs(vec[lg] - a) + 1))
    vec = A/max(abs(vec)) * vec

    return vec


# Calculate x such that the fourier transform of dx/dt is a square wave
def ifft_square(t, f1, f2):
    e = (t[1] - t[0]) / 10
    return 1/(2*np.pi*1j*(t+e)) * (np.exp(2*np.pi*1j*(t+e)*f1) - np.exp(2*np.pi*1j*(t+e)*f2))


velocity = ifft_square(t - max(t)/2, f[0], f[1]).real
position = np.cumsum(velocity) * dt
position = lognorm(position, A=A)

plt.plot(t, position)
plt.grid('on')
plt.xlabel('t (sec)')
plt.ylabel('position (deg)')
plt.title('Trajectory')
plt.show()

print('  move to start position')
autostep.set_move_mode('jog')
autostep.move_to(position[0])
autostep.busy_wait()
time.sleep(1.0)

print('  running trajectory ...', end='')
sys.stdout.flush()
autostep.set_move_mode('max')
autostep.run_trajectory(position)
autostep.busy_wait()
print('  done')
time.sleep(1.0)
autostep.set_move_mode('jog')
print('  move to 0')
autostep.move_to(0.0)
autostep.busy_wait()
print()
