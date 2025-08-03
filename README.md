# ROS2 Arduinobot Visualization with Joint State Publisher

This project demonstrates the basic visualization of a URDF-based robot model (Arduinobot) in ROS2's 3D visualization tool, RViz. The robot's pose is controlled interactively using the `joint_state_publisher_gui`.

## Overview

In ROS2, visualizing a robot's model is a fundamental first step before moving on to more complex tasks like simulation and physical control. This process involves a few key nodes working together:

1.  **`robot_state_publisher`**: This node reads the robot's description from a URDF file and subscribes to the `/joint_states` topic to get the current angles of all joints. It then calculates the 3D poses of all the robot's links and publishes them as TF2 transforms.
2.  **`joint_state_publisher_gui`**: This node provides a simple graphical interface with sliders for each non-fixed joint defined in the URDF. By moving the sliders, this node publishes messages to the `/joint_states` topic, effectively telling the `robot_state_publisher` what the "current" joint angles are.
3.  **RViz**: This is the primary 3D visualization tool in ROS. It subscribes to the TF2 transforms published by the `robot_state_publisher` to display the `RobotModel` in its correct pose.

The goal of this task is to launch these components and use the GUI sliders to manually control the pose of the Arduinobot model within RViz.

## How It Was Done

To achieve this, I executed the following ROS2 launch command in my terminal. This command starts RViz, the `robot_state_publisher`, and the `joint_state_publisher_gui`, all pre-configured for the Arduinobot.

```bash
ros2 launch arduinobot_description simulation.launch.py
```

Once the RViz window appeared, I used the sliders in the "Joint State Publisher" window to change the values for `joint_1` and `joint_2`. This action published the new joint angles to the `/joint_states` topic, which RViz then used to update the visualization of the robot arm in real-time.

## Result

The screenshot below confirms the successful execution of the task. It shows:
- The RViz window displaying the Arduinobot model.
- The "Joint State Publisher" panel with sliders used to manipulate the joint angles.
- The robot arm in a custom pose corresponding to the values set by the sliders, demonstrating that the visualization is being actively controlled.
