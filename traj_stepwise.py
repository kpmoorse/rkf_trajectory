#!/usr/bin/env python
from __future__ import print_function
import sys
import time
import scipy
import numpy
import matplotlib.pyplot as plt
import trajectory
import argparse

from autostep_proxy import AutostepProxy


# Initialize argument parser
parser = argparse.ArgumentParser(description='Stepwise Trajectory')
parser.add_argument('-t', type=float, nargs='+',
                    help='List of trajectory parameters')
args = parser.parse_args()

autostep = AutostepProxy()

print()
print('* trajectory example')
print()

jog_params = {'speed': 200, 'accel': 500, 'decel': 500}
max_params = {'speed': 1000, 'accel': 10000, 'decel': 10000}

# Read trajectory params from command line or use default
if args.t:
    assert len(args.t) % 2 == 0
    freq_list = [args.t[i:i+2] for i in range(0, len(args.t), 2)]
else:
    freq_list = [[10, 1], [10, 2], [10, 1]]

# Create trajectory
dt = AutostepProxy.TrajectoryDt
tau = sum([freq[0] for freq in freq_list])
num_pts = int(tau/dt)
t = dt*numpy.arange(num_pts)

# Calculate stepwise frequency profile
trj = trajectory.Trajectory(t)
trj.set_frequency(trj.stepwise(freq_list), 80)

position = trj.position
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
