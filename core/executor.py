# core/executor.py
import os
import pickle
import rclpy
import re
import subprocess
import sys
import time
import yaml

from builtin_interfaces.msg import Time
from core.ui import info, error, warn, done, debug
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from subprocess import Popen, PIPE

RETRY_MAX_ATTEMPTS = 5
TIME_DELAY        = 2.0
DOCKER_CPU_CORES  = "4"
DOCKER_MEMORY     = "8g"
DOCKER_MEM_SWAP   = "8g"

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

        # Inspector 전용 컨테이너 정보
        self.domain_map     = {}    # rmw_impl → domain_id
        self.inspector_name = "drs_inspector"
        self.inspector_ip   = inspector_ip

    def _docker_exec(self, container: str, cmd: list) -> None:
        try:
            subprocess.run(['docker', 'exec', container] + cmd, check=True)
            info(f"Executed in container '{container}': {' '.join(cmd)}")
        except subprocess.CalledProcessError as e:
            raise subprocess.CalledProcessError(f"Failed to exec in '{container}': {e}")

    def _wait_for_log(self, container: str, pattern: str, timeout: float = 90.0) -> None:
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

        # docker network
        result = subprocess.run(
            ['docker', 'network', 'inspect', self.network_name, '--format', '{{.Id}}'],
            capture_output=True, text=True
        )

        if result.returncode == 0 and result.stdout.strip():
            network_id = result.stdout.strip()
            done(f"Docker network '{self.network_name}' already exists.")
        else:
            result = subprocess.run([
                'docker', 'network', 'create',
                '--driver', 'bridge', f'--subnet={self.subnet}', self.network_name
            ], capture_output=True, text=True, check=True)
            network_id = result.stdout.strip()
            done(f"Created docker network '{self.network_name}'.")

        self.network_iface = f"br-{network_id[:12]}"
        debug(f"Docker network 'name:{self.network_name}','id:{self.network_iface}'")


        # 1) Inspector 전용 컨테이너 (토픽 info 전용)
        info(f"Starting inspector container '{self.inspector_name}' on '{self.network_name}'")
        subprocess.run([
            'docker','run','--rm','-d',
            '--net', self.network_name, '--ip', self.inspector_ip,
            '--name', self.inspector_name, self.image_tag,
            '-c', 'tail -f /dev/null'
        ], check=True)
        time.sleep(TIME_DELAY)

        # 2) RMW별 본 컨테이너 (Gazebo + robot spawn)
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
            except subprocess.CalledProcessError as e:
                error(f"Failed to start '{cname}': {e}")
            

    def run_gazebo(self) -> None:
        #launch = "turtlebot3_world.headless.launch.py" if self.headless else "turtlebot3_world.launch.py"
        launch = "turtlebot3_world.headless.launch.py" if self.headless else "empty_world.launch.py"
        for rmw_impl in self.dds_map:
            cname = f"{self.version}_{self.robot}_{rmw_impl}"
            info(f"Launching Gazebo in '{cname}'")
            subprocess.run([
                'docker','exec','-d', 
                # for cyclonedds port
                '-e', "CYCLONEDDS_URI=<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>", 
                cname, 'bash','-ic',
                f'ros2 launch turtlebot3_gazebo {launch} '
                f'> /proc/1/fd/1 2>/proc/1/fd/2 &'
            ], check=True)
            self._wait_for_log(cname, r'process has finished cleanly')
            time.sleep(TIME_DELAY)
            done(f"Gazebo up in '{cname}'")
            self.delete_robot(rmw_impl)


    def spawn_robot(self, rmw_impl: str) -> None:
        if self.robot not in self.ROBOT_MODELS:
            error(f"Unsupported robot: '{self.robot}'")
            sys.exit(1)
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
        subprocess.run(['docker','exec','-d', cname, 'bash','-ic', cmd], check=True)
        self._wait_for_log(cname, r'process has finished cleanly')
        time.sleep(TIME_DELAY)
        done(f"Robot spawned in '{cname}'")

    def delete_robot(self, rmw_impl: str) -> None:
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

    def record_robot_states(self, rmw_impl) -> None:
        log_dir = f"./output/logs/robot_states/{self.robot}/{rmw_impl}"
        os.makedirs(log_dir, exist_ok=True)

        for topic in self.targets:
            cmd = ["ros2", "topic", "echo", topic, "--once"]
            log_path = f"{log_dir}/{topic}.log"
            try:
                with open(log_path, "w", encoding="utf-8") as out:
                    pid = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT)
                    try:
                        pid.wait(timeout=30)
                        
                    except subprocess.TimeoutExpired as e:
                        pid.kill()
                        pid.wait()
                        raise RuntimeError(f"Subprocess for topic '{topic}' didn't finish in time: {e}")

                if pid.returncode != 0:
                    raise RuntimeError(f"Subprocess for topic '{topic}' failed with code {pid.returncode}")

                info(f"Robot State(/{topic}) log has saved to '{log_path}'")

            except OSError as e:
                raise RuntimeError(f"Unable to open or write to '{log_path}': {e}")
        
        self._convert_logs_to_pkl(rmw_impl)

    def _parse_imu_from_log(self, log_path: str) -> Imu:
        try:
            with open(log_path, 'r', encoding='utf-8') as f:
                data = next(yaml.safe_load_all(f))
        
        except OSError as e:
            raise RuntimeError(f"Can't open file '{log_path}': {e}")

        msg = Imu()

        msg.header.stamp.sec = data['header']['stamp']['sec']
        msg.header.stamp.nanosec = data['header']['stamp']['nanosec']
        msg.header.frame_id = data['header']['frame_id']

        # Orientation
        msg.orientation.x = data['orientation']['x']
        msg.orientation.y = data['orientation']['y']
        msg.orientation.z = data['orientation']['z']
        msg.orientation.w = data['orientation']['w']
        msg.orientation_covariance = data['orientation_covariance']

        # Angular velocity
        msg.angular_velocity.x = data['angular_velocity']['x']
        msg.angular_velocity.y = data['angular_velocity']['y']
        msg.angular_velocity.z = data['angular_velocity']['z']
        msg.angular_velocity_covariance = data['angular_velocity_covariance']

        # Linear acceleration
        msg.linear_acceleration.x = data['linear_acceleration']['x']
        msg.linear_acceleration.y = data['linear_acceleration']['y']
        msg.linear_acceleration.z = data['linear_acceleration']['z']
        msg.linear_acceleration_covariance = data['linear_acceleration_covariance']

        return msg

    def _parse_laser_from_log(self, log_path: str) -> LaserScan:
        try:
            with open(log_path, 'r', encoding='utf-8') as f:
                data = next(yaml.safe_load_all(f))
        
        except OSError as e:
            raise RuntimeError(f"Can't open file '{log_path}': {e}")

        msg = LaserScan()

        msg.header.stamp.sec = data['header']['stamp']['sec']
        msg.header.stamp.nanosec = data['header']['stamp']['nanosec']
        msg.header.frame_id = data['header']['frame_id']

        msg.angle_min = data['angle_min']
        msg.angle_max = data['angle_max']
        msg.angle_increment = data['angle_increment']
        msg.time_increment = data['time_increment']
        msg.scan_time = data['scan_time']
        msg.range_min = data['range_min']
        msg.range_max = data['range_max']
        msg.ranges = self._sanitize(data['ranges'])
        msg.intensities = self._sanitize(data['intensities'])

        return msg

    def _parse_odom_from_log(self, log_path: str) -> Odometry:
        try:
            with open(log_path, 'r', encoding='utf-8') as f:
                data = next(yaml.safe_load_all(f))

        except OSError as e:
            raise RuntimeError(f"Can't open file '{log_path}': {e}")

        msg = Odometry()    

        msg.header.stamp.sec = data['header']['stamp']['sec']
        msg.header.stamp.nanosec = data['header']['stamp']['nanosec']
        msg.header.frame_id = data['header']['frame_id']    

        msg.child_frame_id = data['child_frame_id'] 

        position = data['pose']['pose']['position']
        orientation = data['pose']['pose']['orientation']
        msg.pose.pose.position = Point(x=position['x'], y=position['y'], z=position['z'])
        msg.pose.pose.orientation = Quaternion(
            x=orientation['x'], y=orientation['y'], z=orientation['z'], w=orientation['w']
        )
        msg.pose.covariance = data['pose']['covariance']    

        linear = data['twist']['twist']['linear']
        angular = data['twist']['twist']['angular']
        msg.twist.twist.linear = Vector3(x=linear['x'], y=linear['y'], z=linear['z'])
        msg.twist.twist.angular = Vector3(x=angular['x'], y=angular['y'], z=angular['z'])
        msg.twist.covariance = data['twist']['covariance']  

        return msg

    def _convert_logs_to_pkl(self, rmw_impl: str) -> None:
        log_dir = f"./output/logs/robot_states/{self.robot}/{rmw_impl}"
        msgs = []

        for topic in self.targets:
            log_path = f"{log_dir}/{topic}.log"

            if topic == 'imu':
                msg = self._parse_imu_from_log(log_path)
                msgs.append(msg)
            elif topic == 'odom':
                msg = self._parse_odom_from_log(log_path)
                msgs.append(msg)
            elif topic =='scan':
                msg = self._parse_laser_from_log(log_path)
                msgs.append(msg)

        try:
            pkl_path = f"{log_dir}/robot_state_{rmw_impl}.pkl"
            with open(pkl_path, "wb") as f:
                pickle.dump(msgs, f)
            info(f"Robot states has been saved to {pkl_path}")
        except OSError as e:
            raise RuntimeError(f"Unable to open or write to '{pkl_path}': {e}")

    def _sanitize(self, ranges_raw):
        cleaned = []
        for val in ranges_raw:
            if val == '...' or val is None:
                continue
            cleaned.append(float(val))
        return cleaned