# Darwin Robot Control

This repository contains ROS packages for controlling a robot named Darwin. The robot can be commanded to reach any specified coordinate using a PID controller that controls its speed.

## Packages

- **darwin_gazebo**: Contains launch files and configuration for simulating the Darwin robot in Gazebo and the Python script for controlling the Darwin robot to reach specified coordinates.
- **darwin_description**: Provides the URDF and mesh files for describing the Darwin robot.
- **darwin_control**: Includes the controllers and the yaml file.

## Running the Robot and Coordinate Node

To run the Darwin robot and the coordinate node, follow these steps:

1. **Create a Workspace**: Create a new ROS workspace.
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    ```

2. **Clone the Repository**: Clone this repository into the `src` directory of your ROS workspace.
    ```bash
    git clone https://github.com/samarthag06/Darwin-and-Turtlebot-simulation.git
    ```

3. **Build the Workspace**: Build the ROS workspace using `catkin_make`.
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

4. **Source the Workspace**: Source the setup script of your workspace.
    ```bash
    source devel/setup.bash
    ```

5. **Set Executable Permissions**: Navigate to the `scripts` folder within `darwin_gazebo` package and set executable permissions for the required Python scripts.
    ```bash
    cd ~/catkin_ws/src/Darwin-and-Turtlebot-simulation/Darwin_packages/darwin_gazebo/scripts
    chmod +x walker.py
    chmod +x odom.py
    chmod +x cordinate3.py
    ```

6. **Launch Gazebo Simulation**: Launch the Gazebo simulation environment for the Darwin robot from a separate terminal and sourcing the bash file .
    ```bash
    cd ~/catkin_ws/src
    source devel/setup.bash
    roslaunch darwin_gazebo darwin_gazebo.launch
    ```

7. **Launch Odometry Node**: In a separate terminal, launch the odometry node using the provided launch file.
    ```bash
    cd ~/catkin_ws/src
    source devel/setup.bash
    roslaunch darwin_gazebo start_odom.launch
    ```

8. **Run Coordinate Node**: In another terminal, run the `cordinate3.py` script to specify the goal coordinates for the robot to reach.
    ```bash
    cd ~/catkin_ws/src
    source devel/setup.bash
    rosrun darwin_gazebo cordinate3.py
    ```

9. **Enter Goal Coordinates**: Upon running the `cordinate3.py` script, you'll be prompted to enter the x and y coordinates of the goal. Enter the desired coordinates and press Enter.

10. **Monitor the Robot**: Monitor the robot's movement as it navigates towards the specified goal coordinates.

## Contributors

- [Samarth Agarwal](https://github.com/samarthag06)

