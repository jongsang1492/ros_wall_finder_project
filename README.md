# ROS Wall-Following Robot

This ROS package enables a mobile robot to **autonomously detect and follow walls** using laser scan data.

The robot:
- Searches for the nearest wall  
- Aligns itself properly beside the wall  
- Then follows the wall while avoiding obstacles  

---

## Functionality Overview

### 1. Wall Detection
- Uses laser scan data to find the closest wall  
- Rotates and moves toward the wall until it is close enough  
- Aligns the wall to the robot's left side before starting  

### 2. Wall Following
- Maintains a consistent distance from the wall on its right  
- Avoids obstacles in front  
- Continuously adjusts its path using sensor feedback  

---

## Dependencies

- ROS Noetic  
- Python 3  
- `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `actionlib`, `std_msgs`  

---

## Launch Instructions

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch find_wall_pkg find_wall_and_follow.launch
```

---

## File Structure

```
find_wall_pkg/
├── srv/
│   └── FindWall.srv
├── scripts/
│   ├── find_wall_service_server.py
│   └── wall_following_node.py
├── launch/
│   └── find_wall_and_follow.launch
```

---

## Simulation

This project was developed and tested using **The Construct’s ROS Development Studio** simulation environment.

If you want to reproduce or extend the simulation, visit: [The Construct](https://www.theconstructsim.com)

![wall_following](https://github.com/user-attachments/assets/bdad946d-62da-4877-90a5-444d74405ca2)

---

## Author

**Daniel Yoo**  
University of Toronto – Mechanical Engineering  
Developed as part of a self-driven robotics practice project
