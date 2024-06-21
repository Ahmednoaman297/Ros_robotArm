# Robot Arm Simulation Project

This project simulates a robot arm using URDF, MoveIt, Gazebo, and RViz. The simulation environment allows for motion planning, control, and visualization of the robot arm in various scenarios.

## Overview

The project is structured to facilitate the simulation of a robot arm using various tools:

- **URDF (Unified Robot Description Format)**: Defines the robot model.
- **Gazebo**: Provides a physics-based simulation environment.
- **MoveIt**: Handles motion planning and control.
- **RViz**: Visualizes the robot and its environment.



## Files and Directories

- **config/**: Configuration files for MoveIt, including controller parameters, joint limits, and kinematics settings.
- **launch/**: Launch files to start Gazebo, MoveIt, RViz, and the robot state publisher.
- **urdf/**: The URDF file defining the robot arm's physical and visual properties.
- **worlds/**: Gazebo world files defining the simulation environment.
- **CMakeLists.txt**: CMake build configuration.
- **package.xml**: ROS package configuration.

## Dependencies

- ROS (Robot Operating System)
- Gazebo
- MoveIt
- RViz

## Installation

1. Ensure you have ROS installed on your system. Follow the [ROS installation guide](http://wiki.ros.org/ROS/Installation) for your distribution.
2. Install the required ROS packages:
    ```sh
    sudo apt-get install ros-noetic-gazebo-ros ros-noetic-moveit ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher ros-noetic-rviz
    ```
3. Clone this repository into your ROS workspace:
    ```sh
    cd ~/catkin_ws/src
    git clone https://github.com/yourusername/robot_arm_project.git
    cd ..
    catkin_make
    ```

## Usage

### Launching the Simulation

1. **Gazebo Simulation**: Launch the Gazebo simulation environment with the robot arm:
    ```sh
    roslaunch robot_arm_project gazebo.launch
    ```
2. **MoveIt Motion Planning**: Launch the MoveIt motion planning and execution:
    ```sh
    roslaunch robot_arm_project moveit_planning_execution.launch
    ```
3. **RViz Visualization**: Launch RViz to visualize the robot arm and interact with the MoveIt motion planning interface:
    ```sh
    roslaunch robot_arm_project moveit_rviz.launch
    ```

### Interacting with the Robot Arm

- Use the RViz MotionPlanning plugin to set target poses for the robot arm.
- Plan and execute motions using the interactive markers in RViz.
- Monitor the simulation in Gazebo to see the physical interactions.

## Customization

### Modifying the URDF

- Edit the `urdf/robot_arm.urdf` file to change the robot's physical properties, add new links, joints, or sensors.

### Updating MoveIt Configuration

- Modify the configuration files in the `config/` directory to adjust the robot's motion planning and control parameters.

### Creating Custom Worlds

- Add new Gazebo world files in the `worlds/` directory and update the `gazebo.launch` file to load them.

## Troubleshooting

- Ensure all ROS environment variables are set correctly:
    ```sh
    source ~/catkin_ws/devel/setup.bash
    ```
- Check for missing dependencies and install them using `rosdep`:
    ```sh
    rosdep install --from-paths src --ignore-src -r -y
    ```
- Refer to the ROS, Gazebo, and MoveIt documentation for additional support.

## Contributing

Feel free to open issues or submit pull requests to contribute to this project. For major changes, please open an issue first to discuss what you would like to change.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

This project was inspired by the ROS and MoveIt tutorials, and leverages open-source tools for robot simulation and control.
