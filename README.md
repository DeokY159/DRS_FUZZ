# DRS_FUZZ
**DRS_FUZZ** is a Python-based fuzzer for vulnerability discovery in ROS 2 robotic systems. 
Developed by Team **OOGWAY** for the 2025-1 Capstone Design course in Information Security at Sejong University, 
it targets common ROS 2 communication channels and injects mutated RTPS packets to uncover weaknesses.

## ⚠️ Caution
- This fuzzer uses **Scapy**, so it must be run with **administrator/root** privileges.
- Adapting the fuzzer to other robots or topics may require code modifications.

## 🛠 Requirements
- The host environment must have the target **ROS 2 distribution** installed.
- The host environment must have the corresponding **robot packages** installed.
- **Scapy** Python module must be installed (`pip3 install scapy`).

## 🚀 Supported Configurations
- **ROS 2 distributions:** `humble`, `jazzy`  
- **Robot:** `turtlebot3`  
- **Topic:** `/cmd_vel`

## ▶️ Run Commands
```bash
python3 main.py <ros2_distribution> <robot>
# examples:
python3 main.py humble turtlebot3
python3 main.py jazzy  turtlebot3
```
## 📁 Directory Structure
```bash
├── build
│   ├── builder.py
│   ├── ROBOT
│   │   └── turtlebot3.sh
│   └── ROS2
│       ├── Dockerfile.humble
│       └── Dockerfile.jazzy
├── core
│   ├── executor.py
│   ├── feedback.py
│   ├── fuzzer.py
│   ├── generate.py
│   ├── inspector.py
│   ├── mutator.py
│   ├── oracle
│   │   ├── behavior.py
│   │   ├── differential.py
│   │   └── monitor
│   │       └── ROBOFUZZ.md
│   ├── report.py
│   └── ui.py
├── main.py
└── README.md
```

## 🐞 Discovered Vulnerabilities & Bugs


| ID          | Description                                                                                             | GitHub Issue                                              | Discovered During          |
|-------------|---------------------------------------------------------------------------------------------------------|-----------------------------------------------------------|----------------------------|
| OOGWAY-001  | `ros2 bag play` crashes with a segmentation fault when playing a corrupted `.db3` bag file.             | [rosbag2#1996](https://github.com/ros2/rosbag2/issues/1996) | Development  |
| OOGWAY-002  | Malformed RTPS discovery packet in Fast RTPS causes uncontrolled memory allocation, leading to DoS.     | [rmw_fastrtps#828](https://github.com/ros2/rmw_fastrtps/issues/828) | Development  |
