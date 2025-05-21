# core/executor.py

import subprocess
import os
import sys
import time
import re
from subprocess import Popen, PIPE, STDOUT
from ament_index_python.packages import get_package_share_directory
from core.ui import info, error, warn, done

RETRY_MAX_ATTEMPTS = 5
TIME_DELAY        = 1.0

class FuzzContainer:
    """
    Manages Docker containers for ROS2 fuzzing:
      - run and teardown containers
      - execute commands inside containers (Gazebo, spawn/delete robot)
    """
    ROBOT_MODELS = {"turtlebot3": "burger"}

    def __init__(self, version: str, robot: str, dds_map: dict) -> None:
        self.version      = version
        self.robot        = robot
        self.dds_map      = dds_map
        self.network_name = "drs_fuzz"
        self.subnet       = "192.168.10.0/24"
        self.image_tag    = f"fuzzer_{version}_{robot}"

    def _docker_exec(self, container: str, cmd: list) -> None:
        try:
            subprocess.run(['docker', 'exec', container] + cmd, check=True)
            info(f"Executed in container '{container}': {' '.join(cmd)}")
        except subprocess.CalledProcessError as e:
            error(f"Failed to exec in '{container}': {e}")
            raise

    def _wait_for_log(self, container: str, pattern: str, timeout: float = 30.0) -> None:
        """
        Follow 'docker logs -f' and return once the given regex pattern appears
        in the logs or timeout is reached.
        """
        warn(f"Waiting for log pattern '{pattern}' in '{container}'")
        proc = Popen(['docker', 'logs', '-f', container], stdout=PIPE, stderr=STDOUT, text=True)
        start = time.time()
        try:
            for line in proc.stdout:
                if re.search(pattern, line):
                    done(f"[{container}] log matched: {pattern}")
                    return
                if time.time() - start > timeout:
                    raise TimeoutError(f"Timeout waiting for '{pattern}' in {container}")
        finally:
            proc.kill()

    def run_docker(self) -> None:
        info("Granting X server access: xhost +local:root")
        subprocess.run(['xhost', '+local:root'], check=True)

        # (Re)create network
        existing = subprocess.run(
            ['docker','network','ls','--filter',f'name=^{self.network_name}$','--format','{{.Name}}'],
            capture_output=True, text=True
        ).stdout.splitlines()
        if self.network_name in existing:
            info(f"Removing existing Docker network: {self.network_name}")
            subprocess.run(['docker','network','rm', self.network_name], check=True)

        info(f"Creating Docker network: {self.network_name}")
        subprocess.run([
            'docker','network','create',
            '--driver','bridge', f'--subnet={self.subnet}', self.network_name
        ], check=True)

        # launch all RMW containers
        for dds_name, dds_ip in self.dds_map.items():
            cname = f"{self.version}_{self.robot}_{dds_name}"
            info(f"Starting container '{cname}' on '{self.network_name}'")
            try:
                subprocess.run([
                    'docker','run','--rm','-d','--privileged',
                    '-e', f"DISPLAY={os.environ.get('DISPLAY')}",
                    '-e', f"RMW_IMPLEMENTATION={dds_name}",
                    '-v','/tmp/.X11-unix:/tmp/.X11-unix',
                    '--net', self.network_name, '--ip', dds_ip,
                    '--name', cname, self.image_tag,
                    '-c','tail -f /dev/null'
                ], check=True)
                info(f"Container '{cname}' started")
            except subprocess.CalledProcessError as e:
                error(f"Failed to start '{cname}': {e}")

    def run_gazebo(self) -> None:
        """
        Launch Gazebo server inside each container, detach and wait for "Gazebo multi-robot simulator" in logs.
        """
        for dds_name in self.dds_map:
            cname = f"{self.version}_{self.robot}_{dds_name}"
            info(f"Launching Gazebo in '{cname}' (detached)...")
            subprocess.run([
                'docker', 'exec', '-d', cname,
                'bash', '-ic',
                'ros2 launch turtlebot3_gazebo empty_world.launch.py \
                > /proc/1/fd/1 2>/proc/1/fd/2 &'
            ], check=True)
            self._wait_for_log(cname,r'\[spawn_entity\.py-4\]: process has finished cleanly')
            time.sleep(TIME_DELAY)
            done(f"Gazebo is up in '{cname}'")

    def spawn_robot(self) -> None:
        """
        Spawn the robot model in Gazebo inside each container.
        """
        if self.robot not in self.ROBOT_MODELS:
            error(f"Unsupported robot: '{self.robot}'")
            sys.exit(1)

        for dds_name in self.dds_map:
            cname = f"{self.version}_{self.robot}_{dds_name}"
            pkg_share = get_package_share_directory('turtlebot3_gazebo')
            sdf_path  = os.path.join(pkg_share, 'models', self.ROBOT_MODELS[self.robot],'model.sdf')
            cmd = [
                'ros2','run','gazebo_ros','spawn_entity.py',
                '-entity', self.ROBOT_MODELS[self.robot],
                '-file', sdf_path,
                '-x','0.0','-y','0.0','-z','0.01'
            ]
            for attempt in range(1, RETRY_MAX_ATTEMPTS+1):
                try:
                    self._docker_exec(cname, cmd)
                    break
                except Exception:
                    if attempt == RETRY_MAX_ATTEMPTS:
                        error(f"Failed to spawn robot in '{cname}'")
                        sys.exit(1)
                    info(f"Retrying spawn in '{cname}' (ATTEMPT {attempt})")
                    time.sleep(TIME_DELAY)

    def delete_robot(self) -> None:
        """
        Delete the spawned robot in Gazebo inside each container.
        """
        if self.robot not in self.ROBOT_MODELS:
            return

        for dds_name in self.dds_map:
            cname = f"{self.version}_{self.robot}_{dds_name}"
            delete_args = f"{{name: '{self.ROBOT_MODELS[self.robot]}'}}"
            cmd = [
                'ros2','service','call','/delete_entity',
                'gazebo_msgs/srv/DeleteEntity', delete_args
            ]
            for attempt in range(1, RETRY_MAX_ATTEMPTS+1):
                try:
                    self._docker_exec(cname, cmd)
                    break
                except Exception:
                    if attempt == RETRY_MAX_ATTEMPTS:
                        error(f"Failed to delete robot in '{cname}'")
                        sys.exit(1)
                    info(f"Retrying delete in '{cname}' (ATTEMPT {attempt})")
                    time.sleep(TIME_DELAY)

    def close_docker(self) -> None:
        """
        Stop all containers and remove network.
        """
        for dds_name in self.dds_map:
            cname = f"{self.version}_{self.robot}_{dds_name}"
            info(f"Stopping container '{cname}'")
            subprocess.run(['docker','stop', cname], check=False)

        info(f"Removing Docker network: {self.network_name}")
        subprocess.run(['docker','network','rm', self.network_name], check=False)


class OutputLogger:
    """
    Placeholder for future logging of container output, metrics, etc.
    """
    def __init__(self):
        pass

    def log(self, message: str) -> None:
        # TODO
        pass

    def save(self, path: str) -> None:
        # TODO
        pass
