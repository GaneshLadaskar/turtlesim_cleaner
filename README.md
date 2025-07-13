# turtlesim_cleaner
ROS 2 Python package to control turtlesim (move, rotate, go to goal)”

# 🐢 ROS 2 Turtlesim Cleaner

This is a beginner-friendly ROS 2 package designed for learning how to move and control the turtle in the `turtlesim` simulator using custom Python scripts. It includes functionalities for:

- Moving the turtle linearly
- Rotating the turtle
- Navigating to a goal position

> Compatible with **ROS 2 Jazzy**.

---

## 📁 Package Structure

turtlesim_cleaner/
├── turtlesim_cleaner/
│ ├── init.py
│ ├── move.py
│ ├── rotate.py
│ └── move2goal.py
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
├── test/
└── README.md

yaml
---

## 🚀 Getting Started

### 1. Build the workspace

cd ~/ros2_ws
colcon build --packages-select turtlesim_cleaner
source install/setup.bash

2. Run Turtlesim Node
ros2 run turtlesim turtlesim_node

3. Use the package
Open a new terminal, source your workspace, and run:
ros2 run turtlesim_cleaner move
ros2 run turtlesim_cleaner rotate
ros2 run turtlesim_cleaner move2goal

🧪 Scripts Description
Script	Description
move.py	Move turtle in a straight line
rotate.py	Rotate turtle clockwise/anticlockwise
move2goal.py	Move turtle to a goal position

📦 Dependencies
rclpy
geometry_msgs
turtlesim

Install via:
sudo apt install ros-jazzy-turtlesim
👨‍💻 Author
Ganesh Ladaskar
GitHub: GaneshLadaskar

📜 License
This project is licensed under the MIT License - see the LICENSE file for details.

---

