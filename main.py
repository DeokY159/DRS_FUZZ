# main.py
import argparse
from build.builder import Builder
from core.fuzzer import Fuzzer
from core.ui import info, warn, error

class Interface:
    def __init__(self):
        parser = argparse.ArgumentParser(description="Python-based ROS2 RTPS Fuzzer")
        parser.add_argument("version", help="ROS2 version (e.g., humble, jazzy)")
        parser.add_argument("robot",   help="Robot target (e.g., turtlebot3, px4)")
        parser.add_argument("topic",   help="ROS2 topic name (e.g., cmd_vel)")
        parser.add_argument("--iface", default="ens33",
                            help="Network interface to use (default: ens33)")
        args = parser.parse_args()
        self.version = args.version
        self.robot   = args.robot
        self.topic   = args.topic
        self.iface   = args.iface

        info(f"Starting fuzzer with version='{self.version}', "
             f"robot='{self.robot}', topic='{self.topic}', interface='{self.iface}'")

if __name__ == "__main__":
    interface = Interface()
    builder = Builder()
    builder.build_docker(interface.version, interface.robot)

    fuzzer = Fuzzer(
        version    = interface.version,
        robot      = interface.robot,
        topic_name = interface.topic,
        iface      = interface.iface
    )
    fuzzer.run()
