for i in {1..3}; do
    rosbag record -o test/ /tethermask/truncation __name:=bagfile &
    python traj_stepwise --rng 5 0.1 0.4 0.1 --nopre
    sleep 20
    rosnode kill /bagfile
done
