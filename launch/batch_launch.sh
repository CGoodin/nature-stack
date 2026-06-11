#!/bin/bash

# Define the array of numbers
speeds=(5 7 9 11)
num_iter=10
# Loop through each number in the array
for num in "${speeds[@]}"; do
    # Loop from 1 to 50 for iterations
    for ((i=1; i<=num_iter; i++)); do
        # Create the descriptive filename string
        filename="mavs_odoa_rosbag_speed_${num}_test_${i}"
        echo $filename
        ros2 launch mavs_odoa_experiment.launch.py vehicle_speed:=15.0 bag_name:="$filename"
        # Launch the Python script with arguments
        #python3 script.py "$num" "$i" "$filename"
    done
done
