# DRS FUZZ Tree

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
├── executor.py
├── feedback.py
├── fuzzer.py
├── generate.py
├── inspector.py
├── mutator.py
├── oracle
│   ├── behavior.py
│   ├── differential.py
│   └── monitor
│       └── ROBOFUZZ.md
├── report.py
└── ui.py
./main.py
./README.md
```
