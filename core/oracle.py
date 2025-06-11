# core/oracle.py
import yaml
import re
from datetime import datetime, timezone
from typing import List, Dict, Any
import os
from core.ui import info, error, debug
import re
from datetime import datetime, timezone
from typing import List, Dict, Any
import os

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
                'range_min': data['range_min'],
                'range_max': data['range_max'],
                'ranges': data['ranges'],
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
        scan_range_min = 0.0
        scan_range_max = 0.0
        for section in ['imu', 'odom', 'scan']:
            data_fast = fast.get(section, {})
            data_cycl = cyclone.get(section, {})

            for key in data_fast:
                if section == "scan":
                    if key == "range_min":
                        scan_range_min = data_fast.get(key)
                        continue

                    elif key =="range_max":
                        scan_range_max = data_fast.get(key)
                        continue

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
 
                        fx = float(x)
                        fy = float(y)

                        if abs(fx - fy) > threshold:
                            error(f"Value mismatch in {section}.log: \"{key}: {x}, {y}\"")
                            return True

                        if fx < scan_range_min or fx > scan_range_max:
                            error(f"Invalid value in {section}.log(rmw_fastrtps_cpp): \"{key}: {x}, {{max: {scan_range_max}, min: {scan_range_min}}}\"")
                            return True
                            
                        if fy < scan_range_min or fy > scan_range_min:
                            error(f"Invalid value in {section}.log(rmw_cyclonedds_cpp): \"{key}: {y}, {{max: {scan_range_max}, min: {scan_range_min}}}\"")
                            return True

        return False

def listener_parser(file_path: str):
    events = []
    with open(file_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = re.split(r'[ \t]+', line)
            if len(parts) < 4:
                parts += [""] * (4 - len(parts))
            time_str1, time_str2, event_type, entity = parts[:4]
            time_str = f"{time_str1} {time_str2}"
            event_type = event_type.rstrip(",")
            
            # 1) Parse timestamp
            try:
                if time_str.endswith("Z"):
                    ts = datetime.fromisoformat(time_str.replace("Z", "+00:00"))
                else:
                    ts = datetime.fromisoformat(time_str)
            except ValueError:
                ts = datetime.strptime(time_str, "%Y-%m-%d %H:%M:%S").replace(tzinfo=timezone.utc)

            # 2) Determine if the fourth column is numeric
            events.append({"timestamp": ts, "event": event_type, "entity": entity})
    return events

def compare_listener(file_path_a, file_path_b, topic):
    
    events_fast = listener_parser(file_path_a)
    events_cyclone = listener_parser(file_path_b)
    mapping = {
        "data_available": "DATA_AVAILABLE",
        "subscription_matched": "SUBSCRIPTION_MATCHED",
        "requested_deadline_missed": "REQUESTED_DEADLINE_MISSED",
        "liveliness_lost": "LIVELINESS_LOST",
        "requested_incompatible_qos": "REQUESTED_INCOMPATIBLE_QOS",
        "sample_lost": "SAMPLE_LOST",
    }

    # Log Abstracting for Classification
    def abstract(events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for ev in events:
            if ev["event"] == "data_available":
                if topic not in ev["entity"]:
                    continue
            abs_type = mapping.get(ev["event"])
            if not abs_type:
                continue
            out.append({"timestamp": ev["timestamp"], "type": abs_type})
        return sorted(out, key=lambda e: e["timestamp"])

    seq_fast = abstract(events_fast)
    seq_cyclone = abstract(events_cyclone)

    # analyze events call or counts
    def analyze(seq: List[Dict[str, Any]]) -> Dict[str, Any]:
        data_events = [e for e in seq if e["type"] == "DATA_AVAILABLE"]
        if data_events:
            cutoff = data_events[0]["timestamp"]
        else:
            cutoff = datetime.max.replace(tzinfo=timezone.utc)

        pre_matched = 0
        pre_others = set()
        post_matched = 0
        post_others = set()

        for ev in seq:
            t = ev["timestamp"]
            et = ev["type"]

            # SUBSCRIPTION_MATCHED Count
            if et == "SUBSCRIPTION_MATCHED":
                if t <= cutoff:
                    pre_matched += 1
                else:
                    post_matched += 1

            # Other Events count
            else:
                if t <= cutoff:
                    pre_others.add(et)
                else:
                    post_others.add(et)

        return {
            "pre_matched_cnt": pre_matched,
            "pre_others": sorted(pre_others),
            "data_cnt": len(data_events),
            "post_matched_cnt": post_matched,
            "post_others": sorted(post_others),
        }

    stats_fast = analyze(seq_fast)
    stats_cyclone = analyze(seq_cyclone)

    # compare
    for key in (
        "pre_matched_cnt",
        "pre_others",
        "data_cnt",
        "post_matched_cnt",
        "post_others",
    ):
        if key=="data_cnt": ## Data Packet Implementation Difference
            if not (stats_cyclone[key] - 1 <= stats_fast[key] <= stats_cyclone[key] + 1):
                error(f"Listener Missmatch: \"{key}: {stats_fast[key]}, {stats_cyclone[key]}\"")
                return True
        elif key=="pre_matched_cnt" or key =="post_matched_cnt":
            if abs(stats_fast[key] - stats_cyclone[key]) > 0.15 * stats_cyclone[key]:
                error(f"Listener Missmatch: \"{key}: {stats_fast[key]}, {stats_cyclone[key]}\"")
                return True
        elif stats_fast[key] != stats_cyclone[key]:
            error(f"Listener Missmatch: \"{key}: {stats_fast[key]}, {stats_cyclone[key]}\"")
            return True

    return False