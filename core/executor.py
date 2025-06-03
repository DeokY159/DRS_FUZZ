# core/executor.py
import subprocess
import os
import sys
import time
import re
from subprocess import Popen, PIPE
from ament_index_python.packages import get_package_share_directory
from core.ui import info, error, warn, done, debug

RETRY_MAX_ATTEMPTS = 5
TIME_DELAY        = 1.0
DOCKER_CPU_CORES  = "4"
DOCKER_MEMORY     = "8g"
DOCKER_MEM_SWAP   = "8g"

class FuzzContainer:
    """
    Manages Docker containers and log capture.
    """
    ROBOT_MODELS = {"turtlebot3": "burger"}

    def __init__(self, version: str, robot: str, dds_map: dict, headless: bool = False, asan: bool = False) -> None:
        self.version      = version
        self.robot        = robot
        self.dds_map      = dds_map
        self.headless     = headless
        self.network_name = "drs_fuzz"
        self.network_iface= "none"
        self.subnet       = "192.168.10.0/24"
        parts = [f"fuzzer_{version}_{robot}"]
        if asan:     parts.append("asan")
        self.image_tag = "_".join(parts)
        self.log_procs: list[tuple[Popen, any]] = []

    def _docker_exec(self, container: str, cmd: list) -> None:
        try:
            subprocess.run(['docker', 'exec', container] + cmd, check=True)
            info(f"Executed in container '{container}': {' '.join(cmd)}")
        except subprocess.CalledProcessError as e:
            error(f"Failed to exec in '{container}': {e}")
            raise

    def _wait_for_log(self, container: str, pattern: str, timeout: float = 30.0) -> None:
        debug(f"Waiting for log pattern '{pattern}' in '{container}'")
        proc = Popen(['docker', 'logs', '-f', container], stdout=PIPE, stderr=PIPE, text=True)
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

        # ensure output/logs dir
        logs_dir = os.path.join(os.getcwd(), 'output', 'logs')
        os.makedirs(logs_dir, exist_ok=True)

        # create network
        subprocess.run(
            ['docker','network','ls','--filter',f'name=^{self.network_name}$','--format','{{.Name}}'],
            capture_output=True, text=True
        ).stdout.splitlines()
        info(f"Creating Docker network: {self.network_name}")
        result = subprocess.run([
            'docker','network','create',
            '--driver','bridge', f'--subnet={self.subnet}', self.network_name
        ], capture_output=True, text=True, check=True)
        network_id = result.stdout.strip()
        self.network_iface = f"br-{network_id[:12]}"
        done(f"Created network {self.network_name} (ID={network_id}, iface={self.network_iface})")

        # launch containers and capture logs
        domain_id = 1
        for rmw_impl, dds_ip in self.dds_map.items():
            cname = f"{self.version}_{self.robot}_{rmw_impl}"
            info(f"Starting container '{cname}' on '{self.network_name}'")
            try:
                subprocess.run([
                    'docker','run','--rm','-d','--privileged',
                    '-e', f"DISPLAY={os.environ.get('DISPLAY')}",
                    '-e', f"RMW_IMPLEMENTATION={rmw_impl}",
                    '-e', f"ROS_DOMAIN_ID={domain_id}",
                    '-v','/tmp/.X11-unix:/tmp/.X11-unix',
                    '--net', self.network_name, '--ip', dds_ip,
                    '--cpus', DOCKER_CPU_CORES,
                    '--memory', DOCKER_MEMORY,
                    '--memory-swap', DOCKER_MEM_SWAP,
                    '--name', cname, self.image_tag,
                    '-c','tail -f /dev/null'
                ], check=True)

                # capture logs to output/logs/{container}.log
                log_path = os.path.join(logs_dir, f"{cname}.log")
                with open(log_path,'w'):
                    pass
                log_file = open(log_path, 'a')
                proc = Popen(['docker', 'logs', '-f', cname], stdout=log_file, stderr=log_file, text=True)
                self.log_procs.append((proc, log_file))

                info(f"Container '{cname}' started and logging to {log_path}")
                domain_id +=1
            except subprocess.CalledProcessError as e:
                error(f"Failed to start '{cname}': {e}")

    def run_gazebo(self) -> None:
        launch = "turtlebot3_world.headless.launch.py" if self.headless else "turtlebot3_world.launch.py"
        for dds_name in self.dds_map:
            cname = f"{self.version}_{self.robot}_{dds_name}"
            info(f"Launching Gazebo in '{cname}'")
            subprocess.run([
                'docker','exec','-d',cname,'bash','-ic',
                f'ros2 launch turtlebot3_gazebo {launch} > /proc/1/fd/1 2>/proc/1/fd/2 &'
            ], check=True)
            self._wait_for_log(cname, r'process has finished cleanly')
            time.sleep(TIME_DELAY)
            done(f"Gazebo up in '{cname}'")
            self.delete_robot(dds_name)
            time.sleep(TIME_DELAY)

    def spawn_robot(self,rmw_impl) -> None:
        if self.robot not in self.ROBOT_MODELS:
            error(f"Unsupported robot: '{self.robot}'")
            sys.exit(1)
        
        cname = f"{self.version}_{self.robot}_{rmw_impl}"
        info(f"Spawning robot in '{cname}' (detached)...")
        sdf = f"/root/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_{self.ROBOT_MODELS[self.robot]}/model.sdf"
        cmd = (
            "ros2 run gazebo_ros spawn_entity.py "
            f"-entity {self.ROBOT_MODELS[self.robot]} "
            f"-file {sdf} -x 0.0 -y 0.0 -z 0.01 "
            "> /proc/1/fd/1 2>/proc/1/fd/2 &"
        )
        subprocess.run(['docker','exec','-d', cname, 'bash','-ic', cmd], check=True)
        self._wait_for_log(cname, r'process has finished cleanly')
        time.sleep(TIME_DELAY)
        done(f"Robot spawned in '{cname}'")

    def delete_robot(self,rmw_impl) -> None:
        if self.robot not in self.ROBOT_MODELS:
            return

        cname = f"{self.version}_{self.robot}_{rmw_impl}"
        info(f"Deleting robot in '{cname}' (detached)...")
        args = f"{{name: '{self.ROBOT_MODELS[self.robot]}'}}"
        cmd = (
            "ros2 service call /delete_entity "
            "gazebo_msgs/srv/DeleteEntity "
            f"\"{args}\" > /proc/1/fd/1 2>/proc/1/fd/2 &"
        )
        subprocess.run(['docker','exec','-d', cname, 'bash','-ic', cmd], check=True)
        self._wait_for_log(cname, r'Successfully deleted entity')
        time.sleep(TIME_DELAY)
        done(f"Robot deleted in '{cname}'")

    def close_docker(self) -> None:
        # stop capturing logs
        for proc, lf in self.log_procs:
            proc.terminate()
            lf.close()
        self.log_procs.clear()

        # stop containers
        for rmw_impl in self.dds_map:
            cname = f"{self.version}_{self.robot}_{rmw_impl}"
            info(f"Stopping container '{cname}'")
            subprocess.run(['docker','stop', cname], check=False)

        info(f"Removing Docker network: {self.network_name}")
        subprocess.run(['docker','network','rm', self.network_name], check=False)

class RobotStateMonitor:
    WATCHLIST = {
        "turtlebot3": ["imu", "odom", "scan"]
    }

    def __init__(self, robot) -> None:
        self.robot = robot

    def record_robot_states(self, rmw_impl) -> None:
        targets = self.WATCHLIST[self.robot]

        log_dir = f"./output/logs/robot_states/{self.robot}/{rmw_impl}"
        os.makedirs(log_dir, exist_ok=True)

        for target in targets:
            cmd = ["ros2", "topic", "echo", target, "--once"]
            log_path = f"{log_dir}/{target}.log"
            try:
                with open(log_path, "w", encoding="utf-8") as out:
                    pid = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT)
                    try:
                        pid.wait(timeout=30)
                        
                    except subprocess.TimeoutExpired as e:
                        pid.kill()
                        pid.wait()
                        raise RuntimeError(f"Subprocess for topic '{target}' didn't finish in time: {e}")

                if pid.returncode != 0:
                    raise RuntimeError(f"Subprocess for topic '{target}' failed with code {pid.returncode}")

                info(f"Robot State(/{target}) log has saved to '{log_path}'")

            except OSError as e:
                raise RuntimeError(f"Unable to open or write to '{log_path}': {e}")