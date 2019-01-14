#!/usr/bin/env python
from __future__ import print_function
import sys
import time
import scipy
import numpy
import matplotlib.pyplot as plt
import traj

from autostep_proxy import AutostepProxy


autostep = AutostepProxy()

print()
print('* trajectory example')
print()

jog_params = {'speed': 200, 'accel': 500, 'decel': 500}
max_params = {'speed': 1000, 'accel': 10000, 'decel': 10000}

num_cycle = 6
period = 5.0

# Create trajectory
dt = AutostepProxy.TrajectoryDt
num_pts = int(period*num_cycle/dt)
t = dt*scipy.arange(num_pts)


# Calculate triangle-ramp frequency profile
def freq_ramp(t, max_freq):
    tau = t[-1]
    frequency = max_freq * (1 - numpy.abs(2*t/tau - 1))
    return frequency


position = 80*numpy.cos(traj.phase(freq_ramp(t, 3)), dt)
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
