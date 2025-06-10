import argparse
import os
import time
import datetime
import rclpy
import re
import subprocess

from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile

from scapy.all import sendp, Ether, IP, UDP
from scapy.contrib.rtps import RTPSMessage

import core.feedback as feedback
import core.inspector as inspector
import core.oracle as oracle
from core.executor import FuzzContainer, RobotStateMonitor
from core.mutator import RTPSPacket
from core.ui import info, error, warn, debug
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
)

# --- Constants ---
RETRY_MAX_ATTEMPTS = 10     # retry attempts for transient failures
RETRY_DELAY        = 1.5    # seconds between retries
RUN_DELAY          = 2.5    # seconds between different RMW runs
MESSAGES_PER_RUN   = 10     # messages per spin run
MESSAGE_PERIOD     = 0.2    # seconds between packets
UDP_SPORT          = 45569  # source UDP port for RTPS

# base directories
OUTPUT_DIR = os.path.join(os.getcwd(), 'output')
LOGS_DIR      = os.path.join(OUTPUT_DIR, 'logs')

def send_packet(src_ip: str, dst_ip: str, dport: int, iface: str, rtps: RTPSPacket) -> None:
    """Construct and send one RTPS packet via scapy."""
    pkt = (
        Ether() /
        IP(src=src_ip, dst=dst_ip) /
        UDP(sport=UDP_SPORT, dport=dport) /
        rtps.hdr /
        RTPSMessage(submessages=[rtps.dst, rtps.ts, rtps.data])
    )
    sendp(pkt, iface=iface, verbose=False)
    info(f"Packet sent to {dst_ip}:{dport} (seq={rtps.data.writerSeqNumLow})")

class Interface:
    def __init__(self):
        parser = argparse.ArgumentParser(description="Python-based ROS2 RTPS Fuzzer")
        parser.add_argument("version", help="ROS2 version (e.g., humble, jazzy)")
        parser.add_argument("robot", help="Robot target (e.g., turtlebot3, px4)")
        parser.add_argument("topic", help="ROS2 topic name (e.g., cmd_vel)")
        parser.add_argument("reproduce_path", help="Log path for reproducing (e.g., crash/20250611_015346_crash)")
        parser.add_argument("--headless", action="store_true", help="Run in headless mode (no GUI)")
        parser.add_argument("--asan", action="store_true", help="Enable ASAN build")

        args = parser.parse_args()
        self.version = args.version
        self.robot = args.robot
        self.topic = args.topic
        self.repro_path = args.reproduce_path
        self.headless = args.headless
        self.asan = args.asan

        info(f"Starting Reproducer with version='{self.version}', "
             f"robot='{self.robot}', topic='{self.topic}', headless={self.headless}, asan={self.asan}")

class ReproPublisher(Node):
    """
    ROS2 Node that sends a burst of mutated RTPS packets.
    """
    def __init__(self, robot: str, topic_name: str, rtps: RTPSPacket,
                 rmw_impl: str, dds_id: str, qos: QoSProfile, src_ip: str,
                 dst_ip: str, dport: int, container: FuzzContainer,
                 mutated_payloads: list[bytes], state_monitor: RobotStateMonitor) -> None:
        super().__init__('fuzzer_publisher')
        self.robot     = robot
        self.topic_name= topic_name
        self.rtps      = rtps
        self.rmw_impl  = rmw_impl
        self.src_ip    = src_ip
        self.dds_id    = dds_id
        self.dst_ip    = dst_ip
        self.dport     = dport
        self.iface     = container.network_iface
        self.container = container
        self.seq_num   = 1
        self.future    = Future()
        self.qos       = qos
        self.mutated_payloads = mutated_payloads
        self.state_monitor = state_monitor

        try:
            container.spawn_robot(self.rmw_impl)
        except (OSError, subprocess.SubprocessError) as e:
            raise subprocess.SubprocessError(f"Failed to spawn robot: {e}")

        try:
            inspector.create_publisher(
                topic_name=f'/{topic_name}',
                container=container.inspector_name,
                rmw_impl=self.rmw_impl,
                domain_id=self.dds_id,
                qos_profile=qos
            )
        except (OSError, subprocess.SubprocessError) as e:
            raise subprocess.SubprocessError(f"Failed to create publisher: {e}")
        
        for attempt in range(1, RETRY_MAX_ATTEMPTS + 1):
            try:
                info_json = inspector.get_topic_info(
                    f'/{topic_name}',
                    container=container.inspector_name,
                    rmw_impl=self.rmw_impl,
                    domain_id=self.dds_id,
                    indent=2
                )
                break
            except RuntimeError as e:
                if attempt == RETRY_MAX_ATTEMPTS:
                    error(f"Unable to get topic info: {e}")
                    raise RuntimeError(f"Failed to get topic info after {RETRY_MAX_ATTEMPTS} attempts: {e}")
                warn(f"Retrying topic info (attempt {attempt})")
                time.sleep(RETRY_DELAY)
                  
        # prepare mutation strategy and payloads
        self.rtps.build_base_packet(rmw_impl=self.rmw_impl, inspect_info=info_json)
        # timer triggers send loop
        self.timer = self.create_timer(MESSAGE_PERIOD, self._timer_callback)
        

    def _timer_callback(self) -> None:
        # Use the pre-generated mutated payload
        self.rtps.ts = self.rtps._build_info_ts()
        self.rtps.data.data.serializedData = self.mutated_payloads[self.seq_num - 1]
        self.rtps.data.writerSeqNumLow = self.seq_num

        send_packet(
            src_ip=self.src_ip,
            dst_ip=self.dst_ip,
            dport=self.dport,
            iface=self.iface,
            rtps=self.rtps
        )

        self.seq_num += 1

        if self.seq_num > MESSAGES_PER_RUN + 1:
            info("Stopping Robot")
            time.sleep(RUN_DELAY)
            self.state_monitor.record_robot_states(self.rmw_impl, self.dds_id)
            time.sleep(RUN_DELAY)
            info(f"Sent all messages for RMW='{self.rmw_impl}'")
            self.timer.cancel()
            self.container.delete_robot(self.rmw_impl)
            self.future.set_result(True)
            time.sleep(RETRY_DELAY)
            inspector.stop_publisher(topic_name=f'/{self.topic_name}',container=self.container.inspector_name)
            return

class Reproducer:
    INSPECTOR_IP = "192.168.10.100"
    DST_IP_MAP = {
        "rmw_fastrtps_cpp":     "192.168.10.10",
        "rmw_cyclonedds_cpp":   "192.168.10.20",
        #"rmw_connextdds":      "192.168.10.30",
    }

    DOMAIN_ID_MAP = {
        "rmw_fastrtps_cpp":     "1",
        "rmw_cyclonedds_cpp":   "2",
        #"rmw_connextdds":      "3"
    }

    DST_PORT_MAP = {
        "cmd_vel": {"1": 7665, "2": 7915, "3": 8165}
    }

    def __init__(self, version, robot, topic_name, saved_log_path: str, headless=False, asan=False) -> None:
        self.validation = 0
        self.run        = 1
        self.version    = version
        self.robot      = robot
        self.topic_name = topic_name
        self.headless   = headless
        self.asan   = asan

        self.src_ip           = self.INSPECTOR_IP
        self.rtps             = RTPSPacket(self.topic_name)
        self.container        = FuzzContainer(self.version, self.robot, self.DST_IP_MAP, self.DOMAIN_ID_MAP, self.src_ip, self.headless, self.asan)
        self.state_monitor    = RobotStateMonitor(self.robot)
        self.saved_log_path   = os.path.join(OUTPUT_DIR, saved_log_path)
        self.qos_profile      = self.get_qos_profile()
        self.mutated_payloads = self.get_mutated_payloads()

    def _check_asan(self, log_path: str) -> bool:
        """Check if ASAN error appears in given log file."""
        try:
            with open(log_path, 'r', errors='ignore') as f:
                return 'AddressSanitizer' in f.read() or 'asan:' in f.read()
        except Exception:
            return False

    def check_asan_crash(self) -> None:
        """
        Check ASAN in container logs, handle crash and raise RuntimeError for restart.
        """
        for rmw_impl in self.DST_IP_MAP:
            cname    = f"{self.version}_{self.robot}_{rmw_impl}"
            log_path = os.path.join(LOGS_DIR, f"{cname}.log")
            if self._check_asan(log_path):
                error(f"ASAN detected in container {cname}")
                self.validation += 1
                self.container.close_docker()

    def get_qos_profile(self):
        qos_file = os.path.join(self.saved_log_path, 'qos.txt')
        try:
            with open(qos_file, 'r') as f:
                qos_text = f.read()

        except Exception as e:
            raise OSError(f"Unable to read file {qos_file}: {e}")

        profile = QoSProfile(
            durability=DurabilityPolicy[re.search(r"durability:\s*(\w+)", qos_text).group(1)],
            history=HistoryPolicy[re.search(r"history:\s*(\w+)", qos_text).group(1)],
            depth=int(re.search(r"depth:\s*(\d+)", qos_text).group(1)),
            liveliness=LivelinessPolicy[re.search(r"liveliness:\s*(\w+)", qos_text).group(1)]
        )
        
        return profile

    def get_mutated_payloads(self) -> list:
        payload_list = []
        for i in range(1, MESSAGES_PER_RUN + 1):
            payload_path = os.path.join(self.saved_log_path, f"mutated_{i}.bin")
            with open(payload_path, 'rb') as f:
                payload = f.read()
            payload_list.append(payload)
        payload_list.append(b"\x00"*48)

        return payload_list

    def gen_packet_sender(self, rmw_impl: str) -> None:
        os.environ["RMW_IMPLEMENTATION"] = rmw_impl
        os.environ["ROS_DOMAIN_ID"]      = self.DOMAIN_ID_MAP[rmw_impl]
        if not rclpy.ok():
            rclpy.init()

        msg = f"Reproduce: RMW implementation = {rmw_impl}"
        debug(msg)

        node = None
        try:
            node = ReproPublisher(
                robot      = self.robot,
                topic_name = self.topic_name,
                rtps       = self.rtps,
                rmw_impl   = rmw_impl,
                dds_id     = self.DOMAIN_ID_MAP[rmw_impl],
                qos        = self.qos_profile,
                src_ip     = self.src_ip,
                dst_ip     = self.DST_IP_MAP[rmw_impl],
                dport      = self.DST_PORT_MAP[self.topic_name][self.DOMAIN_ID_MAP[rmw_impl]],
                container  = self.container,
                mutated_payloads = self.mutated_payloads,
                state_monitor = self.state_monitor
            )
            rclpy.spin_until_future_complete(node, node.future)

        except RuntimeError as e:
            raise RuntimeError(f"ReproPublisher runtime error: {e}")
        except (OSError, subprocess.SubprocessError) as e:
            raise subprocess.SubprocessError(f"ReproducePublisher subprocess error: {e}")
        finally:
            if node:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def reproduce(self) -> None:
        while True:
            # 1) start containers and Gazebo
            try:
                info(f"Bringing up containers & Gazebo for {self.version}/{self.robot}")
                self.container.run_docker()
                self.container.run_gazebo()
                info("@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                info("Reproducing...")
                info("@@@@@@@@@@@@@@@@@@@@@@@@@@@")

                detected_bug=False

                fast_log = os.path.join(LOGS_DIR, "dds_api", "fast_listener.log")
                cyclone_log = os.path.join(LOGS_DIR, "dds_api", "cyclone_listener.log")
                
                with open(fast_log, 'w') as f:
                    f.write("")

                with open(cyclone_log, 'w') as f:
                    f.write("")

                for rmw_impl in self.DST_IP_MAP.keys():
                    self.gen_packet_sender(rmw_impl)
                    self.check_asan_crash()
                    time.sleep(RUN_DELAY)

                if oracle.check_robot_states_diff(robot=self.robot, threshold=30.0):
                    detected_bug = True

                if oracle.compare_listener(fast_log, cyclone_log, self.topic_name):
                    detected_bug = True

                if detected_bug:
                    self.validation += 1

                self.run += 1

                if self.run > 10:
                    break

            except (RuntimeError,TimeoutError) as e:
                warn("Cleaning up fuzzing container for restart...")
                time.sleep(RUN_DELAY)
                continue
            except KeyboardInterrupt:
                error("Interrupted by user (ctrl+c)")
            except Exception as e:
                error(f"Error occured in Fuzzing Process - {e}")
            finally:
                self.container.close_docker()
                exit(0)

        if validation >= 8:
            info("The bug or crash is validated!")
        else:
            error("False Positive...")

if __name__ == "__main__":
    interface = Interface()
    reproducer = Reproducer(
        version=interface.version,
        robot=interface.robot,
        topic_name=interface.topic,
        saved_log_path=interface.repro_path,
        headless=interface.headless,
        asan=interface.asan
    )
    reproducer.reproduce()