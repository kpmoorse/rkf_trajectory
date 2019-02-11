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
# Initialize argument parser
parser = argparse.ArgumentParser(description='Square-Fourier Trajectory')
parser.add_argument('-d', type=float,
                    help='Trajectory duration (sec)')
parser.add_argument('-f', type=float, nargs=2,
                    help='Pair of frequency endpoints')
parser.add_argument('-a', type=float,
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
duration = args.d if args.d else 60  # sec
f = args.f if args.f else [1, 2]
A = args.a if args.a else 80

# Create trajectory
dt = AutostepProxy.TrajectoryDt
num_pts = int(duration/dt)
t = dt*scipy.arange(num_pts)


# Smoothly log-truncate large values in an array
def loglim(y, y0=0):

    y = np.array(y).astype(float)
    a = y.copy()
    lg = np.array(np.abs(y) > y0).astype(bool)
    a[lg] = np.sign(y[lg]) * (y0 + np.log(abs(y[lg]) - y0 + 1))

    return a


# Calculate x such that the fourier transform of dx/dt is a square wave
e = dt/10
ifft_square = lambda x, f1, f2: 1/(2*np.pi*1j*x+e) * (np.exp(2*np.pi*1j*x*f1) - np.exp(2*np.pi*1j*x*f2))
velocity = ifft_square(t - max(t)/2, f[0], f[1]).real
position = loglim(A * np.cumsum(velocity) * dt, 0)

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
