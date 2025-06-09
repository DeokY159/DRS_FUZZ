import yaml
import json
from core.ui import info, error, debug

def parse_imu_from_log(log_path: str) -> dict:
    with open(log_path, 'r', encoding='utf-8') as f:
        raw_text = f.read()
    blocks = raw_text.strip().split('---')
    for block in blocks:
        block = block.strip()
        if not block:
            continue
        if 'A message was lost' in block:
            continue
        try:
            data = yaml.safe_load(block)
            imu = {
                'orientation_x': data['orientation']['x'],
                'orientation_y': data['orientation']['y'],
                'orientation_z': data['orientation']['z'],
                'orientation_w': data['orientation']['w'],
                'angular_velocity_x': data['angular_velocity']['x'],
                'angular_velocity_y': data['angular_velocity']['y'],
                'angular_velocity_z': data['angular_velocity']['z'],
                'linear_acceleration_x': data['linear_acceleration']['x'],
                'linear_acceleration_y': data['linear_acceleration']['y'],
                'linear_acceleration_z': data['linear_acceleration']['z'],
            }
            return imu
        except Exception as e:
            raise ValueError(f"Failed to parse IMU block:\n{block}\nError: {e}")
    
    raise FileNotFoundError("No valid IMU block found in log.")

def parse_odom_from_log(log_path: str) -> dict:
    with open(log_path, 'r', encoding='utf-8') as f:
        raw_text = f.read()
    blocks = raw_text.strip().split('---')
    for block in blocks:
        block = block.strip()
        if not block:
            continue
        try:
            data = yaml.safe_load(block)
            odom = {
                'position_x': data['pose']['pose']['position']['x'],
                'position_y': data['pose']['pose']['position']['y'],
                'position_z': data['pose']['pose']['position']['z'],
                'orientation_x': data['pose']['pose']['orientation']['x'],
                'orientation_y': data['pose']['pose']['orientation']['y'],
                'orientation_z': data['pose']['pose']['orientation']['z'],
                'orientation_w': data['pose']['pose']['orientation']['w'],
                'linear_velocity_x': data['twist']['twist']['linear']['x'],
                'linear_velocity_y': data['twist']['twist']['linear']['y'],
                'linear_velocity_z': data['twist']['twist']['linear']['z'],
                'angular_velocity_x': data['twist']['twist']['angular']['x'],
                'angular_velocity_y': data['twist']['twist']['angular']['y'],
                'angular_velocity_z': data['twist']['twist']['angular']['z'],
            }
            return odom
        except Exception as e:
            raise ValueError(f"Failed to parse Odometry block:\n{block}\nError: {e}")
    raise FileNotFoundError("No valid Odometry block found in log.")

def parse_scan_from_log(log_path: str) -> dict:
    with open(log_path, 'r', encoding='utf-8') as f:
        raw_text = f.read()

    blocks = raw_text.strip().split('---')

    for block in blocks:
        block = block.strip()
        if not block:
            continue
        try:
            data = yaml.safe_load(block)
            scan = {
                'angle_min': data['angle_min'],
                'angle_max': data['angle_max'],
                'angle_increment': data['angle_increment'],
                'range_min': data['range_min'],
                'range_max': data['range_max'],
                'ranges': data['ranges'],
                'intensities': data['intensities'],
            }
            return scan
        except Exception as e:
            raise ValueError(f"Failed to parse LaserScan block:\n{block}\nError: {e}")

    raise FileNotFoundError("No valid LaserScan block found in log.")

def parse_robot_states(robot: str) -> dict:
    rmw_list = ["rmw_fastrtps_cpp", "rmw_cyclonedds_cpp"]

    robot_states = {}

    if robot == 'turtlebot3':
        for rmw_impl in rmw_list:
            log_path = f"./output/logs/robot_states/{robot}/{rmw_impl}"

            imu = parse_imu_from_log(log_path + '/imu.log')
            odom = parse_odom_from_log(log_path + '/odom.log')
            scan = parse_scan_from_log(log_path + '/scan.log')

            robot_states[rmw_impl] = {
                'imu': imu,
                'odom': odom,
                'scan': scan
            }
    
    return robot_states

def check_robot_states_diff(robot: str, threshold: float = 30.0) -> bool:
    robot_states = parse_robot_states(robot)

    fast = robot_states.get('rmw_fastrtps_cpp', {})
    cyclone = robot_states.get('rmw_cyclonedds_cpp', {})

    if robot == 'turtlebot3':
        for section in ['imu', 'odom', 'scan']:
            data_fast = fast.get(section, {})
            data_cycl = cyclone.get(section, {})

            for key in data_fast:
                a = data_fast.get(key)
                b = data_cycl.get(key)
                if type(a) != type(b):
                    error(f"Type mismatch in {section}.log: \"{key}: {a}, {b}\"")
                    return True

                if isinstance(a, float):
                    if abs(a - b) > threshold:
                        error(f"Value mismatch in {section}.log: \"{key}: {a}, {b}\"")
                        return True

                elif isinstance(a, list):
                    for x, y in zip(a, b):
                        if str(x) == str(y):
                            continue
 
                        try:
                            fx = float(x)
                            fy = float(y)
                            if abs(fx - fy) > threshold:
                                error(f"Value mismatch in {section}.log: \"{key}: {x}, {y}\"")
                                return True
                        except (ValueError, TypeError):
                            error(f"Value mismatch in {section}.log: \"{key}: {x}, {y}\"")
                            return True

        return False