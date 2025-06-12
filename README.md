# DRS_FUZZ
![DRSFuzz Architecture](https://github.com/user-attachments/assets/6e20b3f1-877f-4426-8c47-965baf2b3bcf)
**DRS_FUZZ** is a Python-based fuzzer for vulnerability discovery in ROS 2 robotic systems. 
Developed by Team **OOGWAY** for the 2025-1 Capstone Design course in Information Security at Sejong University, 
it targets common ROS 2 communication channels and injects mutated RTPS packets to uncover weaknesses.

## ⚠️ Caution
- This fuzzer requires some knowledge of ROS 2 and is intended for security research purposes only.
- This fuzzer uses **Scapy**, so it must be run with **administrator/root** privileges.
- Adapting the fuzzer to other robots or topics may require code modifications.

## 🛠 Requirements
- The host environment must have the target **ROS 2 distribution** installed.<br>
(e.g. https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html )
- Install the required libraries using the following commands:
```bash
sudo pip install scapy rich
sudo apt install x11-xserver-utils
```

## 🧪 Tested

OS: Window11 WSL

Version:
```
No LSB modules are available.
Distributor ID: Ubuntu
Description:    Ubuntu 22.04.5 LTS
Release:        22.04
Codename:       jammy
```

## 🚀 Supported Configurations
- humble / turtlebot3 / cmd_vel
- jazzy / turtlebot3 / cmd_vel (It's better to use `jazzy branch`)

## ▶️ Run Commands
```bash
python3 main.py <ros2_distribution> <robot> <topic_name> [--headless] [--asan]
# examples:
python3 main.py humble turtlebot3 --headless
python3 main.py humble turtlebot3 --headless --asan
```
Normally, the build environment can take a long time. You can build the environment a little faster by pulling below.
```bash
docker pull jmini12/drsfuzz:humble
docker tag jmini12/drsfuzz:humble fuzzer_humble_turtlebot3_asan
```

## 📁 Directory Structure
```bash
./build
├── builder.py
├── ROBOT
│   ├── moveit.sh
│   └── turtlebot3.sh
└── ROS2
    ├── Dockerfile.humble
    └── Dockerfile.jazzy
./core
./core
├── executor.py         # Manages Docker containers, launches robots, and handles logging.
├── feedback.py         # Adjusts mutation strategies and evaluates robot state feedback.
├── fuzzer.py           # Main fuzzer logic—generates, mutates, and sends test cases.
├── inspector.py        # Gathers ROS 2 topic information and manages topic publishers.
├── mutator.py          # Implements RTPS packet mutation strategies and payload generation.
├── oracle.py           # Validates fuzzing results and detects anomalies by analyzing logs.
└── ui.py               # Provides logging utilities and user interface messages.

./main.py               # Entry point—parses arguments and orchestrates the entire fuzzing process.
./tui.py                # (Likely) a text-based user interface for interacting with the tool.
./reproduce.py          # Reproduces bugs/crashes using saved logs and payloads.
./seed_payload
./README.md

```

## 🐞 Discovered Vulnerabilities & Bugs

| ID          | Description                                                                                             | GitHub Issue                                              | Discovered During          |
|-------------|---------------------------------------------------------------------------------------------------------|-----------------------------------------------------------|----------------------------|
| OOGWAY-001  | `ros2 bag play` crashes with a segmentation fault when playing a corrupted `.db3` bag file.             | [rosbag2#1996](https://github.com/ros2/rosbag2/issues/1996) | Development  |
| OOGWAY-002  | Malformed RTPS discovery packet in Fast RTPS causes uncontrolled memory allocation, leading to DoS.     | [rmw_fastrtps#828](https://github.com/ros2/rmw_fastrtps/issues/828) | Development  |


## 🖥️ UI/UX
### CLI Interface
![image1](https://github.com/user-attachments/assets/14f63906-4976-4e1d-9506-7b7a62ff9be6)


### TUI Interface
![image2](https://github.com/user-attachments/assets/04df827b-2ea0-4389-bf48-7e060eb8f1f7)


## 💡 More Options
- DRS_FUZZ can be easily adapted to test different robots: simply adjust the necessary modules or scripts to fit the target robot, and reuse the core framework.

- It is possible to configure the ROS 2 distribution (version) separately for the subscriber and publisher, enabling cross-version compatibility testing

- You can also set different DDS implementations for the subscriber and publisher containers, allowing cross-DDS fuzzing and analysis.

- **For specific setup changes and advanced configurations, please refer to the TODO comments within the code.**
