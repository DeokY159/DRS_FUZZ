# core/executor.py
import os
import rclpy
import re
import subprocess
import sys
import time

from builtin_interfaces.msg import Time
from core.ui import info, error, warn, done, debug
from subprocess import Popen, PIPE

RETRY_MAX_ATTEMPTS = 5
TIME_DELAY        = 2.0
TIME_OUT          = 20.0
DOCKER_CPU_CORES  = "4"
DOCKER_MEMORY     = "8g"
DOCKER_MEM_SWAP   = "8g"
INSPECTOR_CNAME   = "drs_inspector"
class FuzzContainer:
    """
    Manages Docker containers and log capture.
    """
    ROBOT_MODELS = {"turtlebot3": "burger"}

    def __init__(self, version: str, robot: str, dds_map: dict, dds_domain: dict, inspector_ip: str,
                 headless: bool = False, asan: bool = False) -> None:
        self.version        = version
        self.robot          = robot
        self.dds_map        = dds_map
        self.dds_domain     = dds_domain
        self.headless       = headless
        self.network_name   = "drs_fuzz"
        self.network_iface  = None
        self.subnet         = "192.168.10.0/24"
        parts = [f"fuzzer_{version}_{robot}"]
        if asan: parts.append("asan")
        self.image_tag     = "_".join(parts)
        self.log_procs: list[tuple[Popen, any]] = []
        # Inspector container infomation
        self.inspector_name = INSPECTOR_CNAME
        self.inspector_ip   = inspector_ip

    def _docker_exec(self, container: str, cmd: list) -> None:
        try:
            subprocess.run(['docker', 'exec', container] + cmd, check=True)
            info(f"Executed in container '{container}': {' '.join(cmd)}")
        except Exception as e:
            raise subprocess.CalledProcessError(f"Failed to exec in '{container}': {e}")

    def _wait_for_log(self, container: str, pattern: str, timeout: float = TIME_OUT) -> None:
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
        try:
            subprocess.run(['xhost', '+local:root'], check=True)
        except Exception as e:
            raise OSError(f"xhost command failed: {e}")
        
        # ensure output/logs dir
        logs_dir = os.path.join(os.getcwd(), 'output', 'logs')
        dds_api_dir = os.path.join(logs_dir, 'dds_api')
        os.makedirs(logs_dir, exist_ok=True)
        os.makedirs(dds_api_dir, exist_ok=True)

        # docker network
        result = subprocess.run(
            ['docker', 'network', 'inspect', self.network_name, '--format', '{{.Id}}'],
            capture_output=True, text=True
        )

        if result.returncode == 0 and result.stdout.strip():
            network_id = result.stdout.strip()
            done(f"Docker network '{self.network_name}' already exists.")
        else:
            try:
                result = subprocess.run([
                    'docker', 'network', 'create',
                    '--driver', 'bridge', f'--subnet={self.subnet}', self.network_name
                ], capture_output=True, text=True, check=True)
                network_id = result.stdout.strip()
                done(f"Created docker network '{self.network_name}'.")
            except Exception as e:
                raise subprocess.SubprocessError(f"Failed to create docker network: {e}")

        self.network_iface = f"br-{network_id[:12]}"
        debug(f"Docker network 'name:{self.network_name}','id:{self.network_iface}'")

        # 1) Start inspector container
        info(f"Starting inspector container '{self.inspector_name}' on '{self.network_name}'")
        try:
            subprocess.run([
                'docker','run','--rm','-d',
                '--net', self.network_name, '--ip', self.inspector_ip,
                '--cpus', '2',
                '--memory', '2g',
                '--memory-swap', '2g',
                '--name', self.inspector_name, self.image_tag,
                '-c', 'tail -f /dev/null'
            ], check=True)
        except Exception as e:
            raise subprocess.SubprocessError(f"Failed to start inspector container: {e}")
        time.sleep(TIME_DELAY)

        # 2) Start RMW Container (Gazebo + robot spawn)
        for rmw_impl, dds_ip in self.dds_map.items():
            cname = f"{self.version}_{self.robot}_{rmw_impl}"
            info(f"Starting container '{cname}' on '{self.network_name}'")
            try:
                subprocess.run([
                    'docker','run','--rm','-d','--privileged',
                    '-e', f"DISPLAY={os.environ.get('DISPLAY')}",
                    '-e', f"RMW_IMPLEMENTATION={rmw_impl}",
                    '-e', f"ROS_DOMAIN_ID={self.dds_domain[rmw_impl]}",
                    '-v','/tmp/.X11-unix:/tmp/.X11-unix',
                    '-v',f'{logs_dir}/dds_api:/tmp/DRSFuzz', # For DDS API
                    '--net', self.network_name, '--ip', dds_ip,
                    '--cpus', DOCKER_CPU_CORES,
                    '--memory', DOCKER_MEMORY,
                    '--memory-swap', DOCKER_MEM_SWAP,
                    '--name', cname, self.image_tag,
                    '-c','tail -f /dev/null'
                ], check=True)

                # capture log
                log_path = os.path.join(logs_dir, f"{cname}.log")
                with open(log_path, 'w'): pass
                lf = open(log_path, 'a')
                proc = Popen(['docker','logs','-f', cname],
                             stdout=lf, stderr=lf, text=True)
                self.log_procs.append((proc, lf))

                info(f"Container '{cname}' started and logging to {log_path}")
            except Exception as e:
                raise subprocess.CalledProcessError(f"Failed to start '{cname}': {e}")
            

    def run_gazebo(self) -> None:
        launch = "empty_world.headless.launch.py" if self.headless else "empty_world.launch.py"
        for rmw_impl in self.dds_map:
            cname = f"{self.version}_{self.robot}_{rmw_impl}"
            info(f"Launching Gazebo in '{cname}'")
            try:
                subprocess.run([
                    'docker','exec','-d', 
                    '-e', "CYCLONEDDS_URI=<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>", 
                    cname, 'bash','-ic',
                    f'ros2 launch turtlebot3_gazebo {launch} '
                    f'> /proc/1/fd/1 2>/proc/1/fd/2 &'
                ], check=True)
                self._wait_for_log(cname, r'process has finished cleanly')
                time.sleep(TIME_DELAY)
                done(f"Gazebo up in '{cname}'")
                self.delete_robot(rmw_impl)
            except TimeoutError as e:
                raise TimeoutError(f"Time to launch Gazebo: {e}")

            except Exception as e:
                raise subprocess.SubprocessError(f"Failed to launch Gazebo: {e}")


    def spawn_robot(self, rmw_impl: str) -> None:
        if self.robot not in self.ROBOT_MODELS:
            raise Exception(f"Unsupported robot: '{self.robot}'")
        cname = f"{self.version}_{self.robot}_{rmw_impl}"
        info(f"Spawning robot in '{cname}' (detached).")
        sdf = (
            "/root/turtlebot3_ws/install/turtlebot3_gazebo/"
            f"share/turtlebot3_gazebo/models/turtlebot3_"
            f"{self.ROBOT_MODELS[self.robot]}/model.sdf"
        )
        cmd = (
            "ros2 run gazebo_ros spawn_entity.py "
            f"-entity {self.ROBOT_MODELS[self.robot]} "
            f"-file {sdf} -x 0.5 -y 0.5 -z 0.01 "
            "> /proc/1/fd/1 2>/proc/1/fd/2 &"
        )
        try:
            subprocess.run(['docker','exec','-d', cname, 'bash','-ic', cmd], check=True)
            self._wait_for_log(cname, r'process has finished cleanly')
            done(f"Robot spawned in '{cname}'")
            time.sleep(TIME_DELAY)
        except TimeoutError as e:
            raise TimeoutError(f"Timeout to spawn robot: {e}")
        except Exception as e:
            raise subprocess.SubprocessError(f"Failed to spawn robot: {e}")

    def delete_robot(self, rmw_impl: str) -> None:
        cname = f"{self.version}_{self.robot}_{rmw_impl}"
        info(f"Deleting robot in '{cname}' (detached)...")
        args = f"{{name: '{self.ROBOT_MODELS[self.robot]}'}}"
        cmd = (
            "ros2 service call /delete_entity "
            "gazebo_msgs/srv/DeleteEntity "
            f"\"{args}\" > /proc/1/fd/1 2>/proc/1/fd/2 &"
        )
        try:
            subprocess.run(['docker','exec','-d', cname, 'bash','-ic', cmd], check=True)
            self._wait_for_log(cname, r'Successfully deleted entity')
            done(f"Robot deleted in '{cname}'")
            time.sleep(TIME_DELAY)
        except TimeoutError as e:
            raise TimeoutError(f"Timeout to delete robot: {e}")
        except Exception as e:
            raise subprocess.SubprocessError(f"Failed to delete robot in '{cname}': {e}")

    def close_docker(self) -> None:
        # stop capturing logs
        for proc, lf in self.log_procs:
            proc.terminate()
            lf.close()
        self.log_procs.clear()
        # delete containers
        for rmw_impl in self.dds_map:
            cname = f"{self.version}_{self.robot}_{rmw_impl}"
            info(f"Deleting container '{cname}'")
            subprocess.run(['docker','rm','-f', cname], check=False)
        subprocess.run(['docker','rm','-f', self.inspector_name], check=False)
        # remove network
        #info(f"Removing Docker network: {self.network_name}")
        #subprocess.run(['docker','network','rm', self.network_name], check=False)


class RobotStateMonitor:
    WATCHLIST = {
        "turtlebot3": ["imu", "odom", "scan"]
    }

    def __init__(self, robot: str) -> None:
        self.robot = robot
        self.targets = self.WATCHLIST[self.robot]

    def record_robot_states(self, rmw_impl: str, dds_id: str) -> None:
        log_dir = f"./output/logs/robot_states/{self.robot}/{rmw_impl}"
        os.makedirs(log_dir, exist_ok=True)

        for topic in self.targets:
            # Call `ros2 topic echo`
            cmd = (
                f"ros2 topic echo /{topic} --once"
            )
            docker_cmd = [
                "docker", "exec",
                "-e", f"RMW_IMPLEMENTATION={rmw_impl}",
                "-e", f"ROS_DOMAIN_ID={dds_id}",
                INSPECTOR_CNAME,
                "bash", "-ic",
                cmd
            ]

            log_path = f"{log_dir}/{topic}.log"

            with open(log_path, "w", encoding="utf-8") as out:
                proc = subprocess.Popen(
                    docker_cmd,
                    stdout=out,
                    stderr=subprocess.DEVNULL
                )
                try: 
                    proc.wait(timeout=TIME_OUT)
                except Exception as e:
                    proc.kill()
                    proc.wait()
                    raise TimeoutError(f"Topic '{topic}' echo timed out: {e}")

            if proc.returncode != 0:
                raise subprocess.SubprocessError(f"Topic '{topic}' echo failed (code {proc.returncode})")

            info(f"Robot State(/{topic}) log saved to '{log_path}'")

