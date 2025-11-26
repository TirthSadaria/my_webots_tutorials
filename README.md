# Walker Robot - ROS2 Webots Simulation

## Overview

This package implements a simple walker algorithm similar to a Roomba robot vacuum cleaner. The robot moves forward until it encounters an obstacle (without colliding), then rotates in place until the path ahead is clear. After that, it moves forward again and repeats the process. Each time the robot rotates, it alternates between rotating clockwise and counterclockwise.

The implementation uses the **State design pattern** to manage the robot's behavior through a state machine with two main states:
- **MovingForwardState**: Robot moves forward until an obstacle is detected
- **RotatingState**: Robot rotates in place (alternating direction) until path is clear

## Dependencies

- **ROS2 Humble Hawksbill**: Required ROS2 distribution
- **Webots**: Robot simulation environment (R2023b or compatible)
- **webots_ros2_driver**: ROS2 package for Webots integration
  ```bash
  sudo apt install ros-humble-webots-ros2
  ```
- **rosbag2**: For recording and playing back bag files
  ```bash
  sudo apt install ros-humble-rosbag2
  ```

## Assumptions

- ROS2 Humble is installed and sourced
- Webots is installed and configured
- `webots_ros2_driver` package is installed
- The workspace is built using `colcon`
- All dependencies are available in the ROS2 environment

## Build Instructions

1. Navigate to your ROS2 workspace:
   ```bash
   cd /path/to/your/workspace
   ```

2. Clone or copy the `walker` package to your workspace `src` directory:
   ```bash
   # If package is already in workspace root:
   cd my_webots_tutorials
   ```

3. Build the package:
   ```bash
   colcon build --packages-select walker
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Running the Simulation

### Basic Launch (No Recording)

To launch the simulation without recording a bag file:

```bash
ros2 launch walker walker_launch.py record_bag:=false
```

This will:
- Launch Webots with the walker world
- Start the Webots controller node
- Start the walker node with state machine
- Display the simulation in the Webots GUI

### Launch with Bag File Recording

To launch the simulation and record a bag file:

```bash
ros2 launch walker walker_launch.py record_bag:=true
```

The bag file will be saved to `walker/results/walker_bag_<timestamp>/` with a timestamped folder name.

**Note**: The recording excludes all `/camera/*` topics to keep the bag file size manageable.

### Disable Bag File Recording

To disable recording during a running simulation, you can stop the rosbag process manually, or simply launch with `record_bag:=false`.

## Recording Bag Files

### Using Launch File

The launch file provides a convenient way to record bag files:

```bash
ros2 launch walker walker_launch.py record_bag:=true
```

Let the simulation run for 15-30 seconds to capture sufficient walker behavior, then close Webots to stop the recording.

### Manual Recording

Alternatively, you can record manually in a separate terminal:

```bash
ros2 bag record -a -x "/camera/.*" --output walker/results/walker_bag
```

## Inspecting Bag Files

To view information about a recorded bag file:

```bash
cd /home/tirth/ENPM700/ros2/webots_tutorials/my_webots_tutorials
source install/setup.bash
# Use the directory name (not the .db3 file)
ros2 bag info walker/results/walker_bag
```

**Note**: `ros2 bag info` requires the directory path (the folder containing `metadata.yaml` and `.db3` files), not the individual `.db3` file path.

This will display:
- Duration of the recording
- Number of messages per topic
- Topic names and message types
- Total size of the bag file

**Example output**:
```
Files:             walker_bag_<timestamp>/
Version:           5
Duration:          20.123s
Start:             Jan 01 2024 12:00:00.00 (1704110400.00)
End:               Jan 01 2024 12:00:20.12 (1704110420.12)
Size:              2.5 MiB
Messages:          1234
Topics:            /cmd_vel    500 msgs    : geometry_msgs/msg/Twist
                  /scan       734 msgs    : sensor_msgs/msg/LaserScan
```

## Playing Back Bag Files

To play back a recorded bag file and see the robot replay the movements:

**Terminal 1** - Start Webots (without walker node):
```bash
cd /home/tirth/ENPM700/ros2/webots_tutorials/my_webots_tutorials
source install/setup.bash
ros2 launch webots_ros2_turtlebot robot_launch.py world:=turtlebot3_burger_example.wbt mode:=realtime
```

**Terminal 2** - Play back the bag file:
```bash
cd /home/tirth/ENPM700/ros2/webots_tutorials/my_webots_tutorials
source install/setup.bash
# Use the directory name (not the .db3 file)
ros2 bag play walker/results/walker_bag
```

**Note**: If your bag file has a timestamp, use: `ros2 bag play walker/results/walker_bag_<timestamp>` (the directory name, not the .db3 file).

**Terminal 3** (Optional) - Monitor topics:
```bash
source install/setup.bash
ros2 topic echo /cmd_vel
# Or monitor scan data
ros2 topic echo /scan
```

**Note**: Replace `<timestamp>` with the actual timestamp from your bag file name, or use `walker_bag` if using the default name.

### Compressing Bag Files

If bag files are too large for GitHub, you can compress them:

```bash
ros2 bag compress walker/results/walker_bag_<timestamp>
```

This will create a compressed version of the bag file.

### Large Bag Files

If bag files are still too large after compression, consider:
1. Uploading to Google Drive and including the link in this README
2. Reducing recording duration
3. Excluding additional topics if not needed

## Project Structure

```
walker/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata and dependencies
├── include/walker/         # Header files
│   ├── state.hpp           # Abstract State base class
│   ├── state_context.hpp   # State machine context
│   ├── moving_forward_state.hpp
│   ├── rotating_state.hpp
│   └── walker_node.hpp    # Main ROS2 node
├── src/                    # Source files
│   ├── state.cpp
│   ├── state_context.cpp
│   ├── moving_forward_state.cpp
│   ├── rotating_state.cpp
│   └── walker_node.cpp
├── launch/                 # Launch files
│   └── walker_launch.py
├── worlds/                 # Webots world files
│   └── walker_world.wbt
├── resource/               # URDF and other resources
│   └── walker_robot.urdf
└── results/                # Bag files (gitignored)
    └── walker_bag_*/
```

## Parameters

The walker node supports the following ROS2 parameters:

- `obstacle_threshold` (default: 0.5 m): Distance threshold for obstacle detection
- `forward_velocity` (default: 0.2 m/s): Forward linear velocity
- `angular_velocity` (default: 0.5 rad/s): Angular velocity for rotation
- `timer_period` (default: 0.1 s): State machine update period

These can be modified in the launch file or via parameter files.

## State Machine Behavior

1. **Initial State**: `MovingForwardState`
   - Robot moves forward at `forward_velocity`
   - Monitors laser scan for obstacles

2. **Obstacle Detected**: Transition to `RotatingState`
   - When minimum distance < `obstacle_threshold`
   - Robot stops forward motion
   - Begins rotation (alternating direction)

3. **Path Clear**: Transition back to `MovingForwardState`
   - When minimum distance > `obstacle_threshold`
   - Robot resumes forward motion

4. **Rotation Direction**: Alternates between clockwise and counterclockwise on each rotation cycle

## Troubleshooting

### Webots Not Launching
- Ensure Webots is installed and `WEBOTS_HOME` is set if needed
- Check that `webots_ros2_driver` is installed

### No Laser Scan Data
- Verify the robot in the world file has a laser scanner
- Check topic name: `ros2 topic list` should show `/scan`
- Ensure Webots controller is running

### Bag File Not Recording
- Check that `record_bag:=true` is set
- Verify `results/` directory exists and is writable
- Check disk space

### Robot Not Moving
- Verify `/cmd_vel` topic is being published: `ros2 topic echo /cmd_vel`
- Check that walker node is running: `ros2 node list`
- Review node logs for errors

## License

Copyright (c) 2024 Tirth Sadaria

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

