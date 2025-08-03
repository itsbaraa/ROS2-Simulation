# ROS2 Arduinobot Control and Visualization

This project demonstrates two fundamental methods for interacting with and controlling a robot arm in ROS2 using the Arduinobot model.

1.  **Direct Joint Visualization**: Manually posing the robot using GUI sliders.
2.  **Motion Planning**: Using the MoveIt framework to intelligently plan a trajectory to a goal.

# Prerequisite
Install the robot arm package from this [repo](https://github.com/smart-methods/Robot_Arm_ROS2).

---

## Task 1: Interactive Visualization with Joint State Publisher

This task focuses on visualizing the robot model in RViz and manually controlling its pose. By launching the `display.launch.xml` file, the `joint_state_publisher_gui` is started. This tool provides sliders to directly manipulate the angle of each joint, which is useful for debugging the robot's model and understanding its basic movements.

### Usage

The following command starts RViz and the necessary nodes for visualization:

```bash
ros2 launch arduinobot_description simulation.launch.py
```

### Result

The screenshot below shows the `joint_state_publisher_gui` being used to articulate the robot arm inside RViz.

<img width="1454" height="769" alt="image" src="https://github.com/user-attachments/assets/8c9a2efb-bb3a-4d2f-b645-f99db1292879" />

---

## Task 2: Motion Planning with MoveIt

This task demonstrates a more advanced method of control using the MoveIt motion planning framework. Instead of controlling individual joints, we define a high-level goal (e.g., a pre-defined "down" position) for the robot's end-effector. MoveIt's kinematic solvers and planners then automatically compute a collision-free joint trajectory to reach that goal.

### Usage

The following command starts the MoveIt planning environment for the Arduinobot:

```bash
ros2 launch arduinobot_mc demo.launch.py
```

### Result

The screenshot below shows the MoveIt `MotionPlanning` panel in RViz. A start state (orange robot) and a goal state (green robot) are defined, and MoveIt has successfully generated a planned trajectory (the multi-colored path) between them.

<img width="1916" height="913" alt="image" src="https://github.com/user-attachments/assets/63b97e10-6f3e-4086-9b4b-bd7d2bde1531" />
