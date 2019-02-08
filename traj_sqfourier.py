#!/usr/bin/env python
from __future__ import print_function
import sys
import time
import scipy
import numpy as np
import matplotlib.pyplot as plt
import trajectory
import argparse

from autostep_proxy import AutostepProxy

# Initialize argument parser
# Initialize argument parser
parser = argparse.ArgumentParser(description='Square-Fourier Trajectory')
parser.add_argument('-d', type=float,
                    help='Trajectory duration (sec)')
parser.add_argument('-f', type=float, nargs=2,
                    help='Pair of frequency endpoints')
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

# Create trajectory
dt = AutostepProxy.TrajectoryDt
num_pts = int(duration/dt)
t = dt*scipy.arange(num_pts)

# Calculate x such that the fourier transform of dx/dt is a square wave
f = lambda x, f1, f2: 1/(2*np.pi*1j*x) * (np.exp(2*np.pi*1j*x*f1) - np.exp(2*np.pi*1j*x*f2))
velocity = f(t, f[0], f[1])
position = np.cumsum(velocity, dt)

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
