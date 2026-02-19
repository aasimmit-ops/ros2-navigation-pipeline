# ROS2 Navigation Pipeline

## 1. Overview

This project implements a complete mobile robot navigation pipeline using ROS2 and TurtleBot3 (Burger) in Gazebo simulation.

The system generates a trajectory, smooths it, and controls a differential drive robot using a Pure Pursuit controller.

The implementation follows modular ROS2 architecture principles with separate nodes for each major task.

---

## 2. System Architecture

The navigation pipeline consists of three main nodes:

### 2.1 Trajectory Node
- Publishes a predefined reference path
- Output topic: `/trajectory`
- Message type: `nav_msgs/Path`

### 2.2 Smoothing Node
- Applies path smoothing to reduce sharp turns
- Subscribes to `/trajectory`
- Publishes `/smooth_path`
- Message type: `nav_msgs/Path`

### 2.3 Controller Node
- Implements Pure Pursuit algorithm
- Subscribes to:
  - `/smooth_path`
  - `/odom`
- Publishes:
  - `/cmd_vel`
- Controls robot linear and angular velocity

---

## 3. Algorithm Explanation

### Pure Pursuit Controller

The controller:
1. Reads robot pose from `/odom`
2. Selects a lookahead point from the reference path
3. Transforms the point into the robot frame
4. Computes curvature:
   
   curvature = (2 * y_r) / (lookahead_distance²)

5. Computes angular velocity:
   
   angular_velocity = linear_velocity × curvature

Angular velocity is limited to avoid instability.

This ensures smooth path tracking.

---

## 4. How to Run

### Step 1: Build Workspace
```bash
colcon build
source install/setup.bash
```

### Step 2: Launch Gazebo
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Step 3: Run Navigation Nodes
```bash
ros2 run path_navigation trajectory_node
ros2 run path_navigation smoothing_node
ros2 run path_navigation controller_node
```
---
### Step 4: Visualize in RViz (Optional but Recommended)

Open a new terminal:

```bash
rviz2
```

Set:

- Fixed Frame → `odom`

Add the following displays:

- RobotModel  
- TF  
- Path (`/trajectory`)  
- Path (`/smooth_path`)  
- Odometry (`/odom`)  

RViz is used to visualize:

- The planned trajectory  
- The smoothed path  
- The robot’s pose  
- Real-time motion tracking  

This helps verify correct path following behavior.


## 5. Design Decisions

- Modular ROS2 node architecture for separation of concerns  
- Pure Pursuit chosen for simplicity and stable tracking  
- Constant linear velocity for predictable motion  
- Angular velocity saturation added for safety  
- Lookahead distance tuned for TurtleBot3 stability  

---

## 6. Extension to Real Robot

To deploy on a real TurtleBot3:

- Replace Gazebo launch with real robot bringup  
- Use real `/odom` feedback from wheel encoders  
- Tune lookahead distance and velocity limits  
- Subscribe to `/scan` (sensor_msgs/LaserScan) for obstacle detection  
- Integrate SLAM for real-world mapping  

---

## 7. AI Tools Used

AI tools were used for:(ChatGpt)

- Code structuring and debugging support  
- Algorithm validation  
- Documentation refinement
- coding format and ideas  

All final implementation decisions and testing were performed manually in simulation.

---

## 8. Extra Credit – Obstacle Avoidance (Proposed Extension)

Future improvements include:

- Subscribing to `/scan` for LiDAR data ((sensor_msgs/LaserScan)
- Detecting obstacles within a safety radius  
- Dynamically adjusting the lookahead target  
- Integrating Dynamic Window Approach (DWA)  
- Implementing Potential Field methods  

---

## 9. Results

The robot successfully:

- Tracks a smooth trajectory  
- Maintains stable curvature transitions  
- Avoids oscillatory motion  
- Produces smooth velocity commands  

---

## 10. Repository Structure

```
path_navigation/
│── controller_node.py
│── smoothing_node.py
│── trajectory_node.py
│── package.xml
│── setup.py
│── README.md
```

---

## 11. Testability & Quality Assurance

### Test Case Design

The following scenarios were tested:

1. Straight Line Tracking  
   - Robot follows a linear path  
   - Expected: minimal angular velocity  

2. Curved Path Tracking  
   - Robot follows a smooth curve  
   - Expected: stable curvature response  

3. Empty Trajectory  
   - No path published  
   - Expected: robot stops safely  

4. Invalid Lookahead Distance  
   - Zero or negative value  
   - Expected: controller does not compute curvature  

5. Angular Velocity Saturation  
   - High curvature input  
   - Expected: velocity limited to max value  

---

### Test Automation

Unit tests were implemented to verify:

- Curvature calculation logic  
- Handling of invalid parameters  
- Mathematical correctness  

Tests can be run using:

```bash
pytest
```

---

### Error Handling

The controller includes:

- Validation of lookahead distance  
- Safe stop if no trajectory available  
- Warnings if odometry is missing  
- Protection against division by zero  

These ensure robustness and predictable behavior.





