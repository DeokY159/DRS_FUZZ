# fuzzer.py
import argparse
from build.builder import Builder

class Fuzzer:
    def __init__(self, version, robot):
        self.version = version
        self.robot = robot
        self.builder = Builder()

    def prepare(self):
        # package and build each DDS implementation
        self.builder.build_docker(self.version, self.robot)

    def run(self):
        print(f"Starting fuzzing on ROS2 {self.version} with {self.robot}")
        self.builder.run_docker(self.version, self.robot)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("version", help="ROS2 version (e.g., humble, jazzy)")
    parser.add_argument("robot", help="Robot target (e.g., turtlebot3, px4)")
    args = parser.parse_args()

    fz = Fuzzer(args.version, args.robot)
    fz.prepare()
    fz.run()
