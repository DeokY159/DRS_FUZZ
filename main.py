# main.py
import shutil
from pathlib import Path

output_dir = Path(__file__).resolve().parent / 'output'
if output_dir.exists():
    for entry in output_dir.iterdir():
        if entry.name == 'crash':
            continue
        if entry.is_dir():
            shutil.rmtree(entry)
        else:
            entry.unlink()

import argparse
from build.builder import Builder
from core.fuzzer import Fuzzer
from core.ui import info, banner

class Interface:
    def __init__(self):
        parser = argparse.ArgumentParser(description="Python-based ROS2 RTPS Fuzzer")
        parser.add_argument("version", help="ROS2 version (e.g., humble, jazzy)")
        parser.add_argument("robot", help="Robot target (e.g., turtlebot3, px4)")
        parser.add_argument("topic", help="ROS2 topic name (e.g., cmd_vel)")
        parser.add_argument("--headless", action="store_true", help="Run in headless mode (no GUI)")
        parser.add_argument("--asan", action="store_true", help="Enable ASAN build")

        args = parser.parse_args()
        self.version = args.version
        self.robot = args.robot
        self.topic = args.topic
        self.headless = args.headless
        self.asan = args.asan

        info(f"Starting fuzzer with version='{self.version}', "
             f"robot='{self.robot}', topic='{self.topic}', headless={self.headless}, asan={self.asan}")

if __name__ == "__main__":
    banner()
    interface = Interface()
    builder = Builder()
    builder.build_docker(interface.version, interface.robot,
                         headless=interface.headless, asan=interface.asan)
    while True:
        fuzzer = Fuzzer(
            version=interface.version,
            robot=interface.robot,
            topic_name=interface.topic,
            headless=interface.headless,
            asan=interface.asan
        )
        fuzzer.run()
