#!/bin/bash

# Define the array of numbers
speeds=(5 7 9 11)
#speeds=(11)
num_iter=50

# Loop through each number in the array
for num in "${speeds[@]}"; do
    # Loop from 1 to num_iter for iterations
    for ((i=1; i<=num_iter; i++)); do
    #for ((i=28; i<=num_iter; i++)); do
        filename="mavs_odoa_rosbag_speed_${num}_test_${i}"
        echo $filename
        ros2 launch mavs_odoa_experiment.launch.py vehicle_speed:=${num}.0 bag_name:="$filename"

    done
done
