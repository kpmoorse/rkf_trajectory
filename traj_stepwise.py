#!/usr/bin/env python
from __future__ import print_function
import sys
import time
import scipy
import numpy
import matplotlib.pyplot as plt
import trajectory
import argparse
import rospy
from std_msgs.msg import Float64

from autostep_proxy import AutostepProxy


# Initialize argument parser
parser = argparse.ArgumentParser(description='Stepwise Trajectory')
parser.add_argument('-t', '--traj', metavar='TRAJECTORY', type=float, nargs='+',
                    help='List of trajectory parameters; format = t f [t f [...')
parser.add_argument('-r', '--rng', metavar='RANGE', type=float, nargs='+',
                    help='Frequency range parameters; format = t [start] stop [step]')
parser.add_argument('--nopre', dest='nopre', action='store_true')
args = parser.parse_args()

autostep = AutostepProxy()

print()
print('* trajectory example')
print()

jog_params = {'speed': 200, 'accel': 500, 'decel': 500}
max_params = {'speed': 1000, 'accel': 10000, 'decel': 10000}

assert bool(args.traj) ^ bool(args.rng), "Arguments must be TRAJECTORY or LOOP but not both"

# Read trajectory params from command line or use default
if args.traj:
    assert len(args.traj) % 2 == 0
    freq_list = [args.traj[i:i+2] for i in range(0, len(args.traj), 2)]
elif args.rng:
    t = args.rng[0]
    args.rng = args.rng[1:]
    assert len(args.rng) in [1, 2, 3]
    freq_list = [[t, i] for i in numpy.arange(*args.rng)]

preview = False if args.nopre else True
print(freq_list)

rospy.init_node('freq_counter')
freq_pub = rospy.Publisher(rospy.resolve_name("~frequency"), Float64, queue_size=10)

# Create trajectory
dt = AutostepProxy.TrajectoryDt
tau = sum([freq[0] for freq in freq_list])
num_pts = int(tau/dt)
t = dt*numpy.arange(num_pts)

# Calculate stepwise frequency profile
trj = trajectory.Trajectory(t)
trj.set_frequency(trj.stepwise(freq_list, rnd=True), 80)

# Display trajectory plot
position = trj.position
if preview:
    plt.plot(t, position)
    plt.grid('on')
    plt.xlabel('t (sec)')
    plt.ylabel('position (deg)')
    plt.title('Trajectory')
    plt.show()

# Initialize to zero-point
print('  move to start position')
autostep.set_move_mode('jog')
autostep.move_to(position[0])
autostep.busy_wait()
time.sleep(1.0)

# Loop over stepwise chunks
print('  running trajectory ...', end='')
sys.stdout.flush()
autostep.set_move_mode('max')

for i, freq in enumerate(freq_list):

    start = int(sum([x[0] for x in freq_list[:i]]) / dt)
    end = int(sum([x[0] for x in freq_list[:i+1]]) / dt)
    rng = range(start, end)

    freq_pub.publish(freq[1])
    autostep.run_trajectory(position[rng])
    autostep.busy_wait()

print('  done')
time.sleep(1.0)
autostep.set_move_mode('jog')
print('  move to 0')
autostep.move_to(0.0)
autostep.busy_wait()
print()
