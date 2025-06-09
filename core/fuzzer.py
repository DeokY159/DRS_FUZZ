# core/fuzzer.py
import os
import socket
import sys
import time
import datetime
import shutil
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

from scapy.all import sendp, Ether, IP, UDP
from scapy.contrib.rtps import RTPSMessage

import core.inspector as inspector
from core.executor import FuzzContainer, RobotStateMonitor
from core.mutator import RTPSPacket, DDSConfig
from core.ui import info, error, debug

# --- Constants ---
RETRY_MAX_ATTEMPTS = 10     # retry attempts for transient failures
RETRY_DELAY        = 1.0    # seconds between retries
PACKETS_PER_QOS    = 10     # how often to rotate QoS (in runs)
MESSAGES_PER_RUN   = 10     # messages per spin run
MESSAGE_PERIOD     = 1    # seconds between packets
UDP_SPORT          = 45569  # source UDP port for RTPS
RUN_DELAY          = 3.0    # seconds between different RMW runs

# base directories
OUTPUT_DIR    = os.path.join(os.getcwd(), 'output')
LOGS_DIR      = os.path.join(OUTPUT_DIR, 'logs')
CRASH_DIR     = os.path.join(OUTPUT_DIR, 'crash')
STATE_LOG     = os.path.join(LOGS_DIR, 'current_state.log')

# ensure output directories exist
os.makedirs(LOGS_DIR, exist_ok=True)
os.makedirs(CRASH_DIR, exist_ok=True)

# initialize state log (clear or create)
with open(STATE_LOG, 'w') as f:
    f.write(f"{datetime.datetime.now().isoformat()} - Fuzzing state log created\n")

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

class FuzzPublisher(Node):
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

        container.spawn_robot(self.rmw_impl)

        # fetch topic info for base packet     
        inspector.create_publisher(
                topic_name=f'/{topic_name}',
                container=container.inspector_name,
                rmw_impl=self.rmw_impl,
                domain_id=self.dds_id,
                qos_profile=qos
            )
        
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
            except Exception as e:
                if attempt == RETRY_MAX_ATTEMPTS:
                    error(f"Unable to get topic info: {e}")
                    raise
                info(f"Retrying topic info (attempt {attempt})")
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
            time.sleep(3)
            self.state_monitor.record_robot_states(self.rmw_impl)
            time.sleep(RETRY_DELAY)
            info(f"Sent all messages for RMW='{self.rmw_impl}'")
            self.timer.cancel()
            self.container.delete_robot(self.rmw_impl)
            self.future.set_result(True)
            inspector.stop_publisher(topic_name=f'/{self.topic_name}',container=self.container.inspector_name)
            return

class Fuzzer:
    """
    Top-level controller:
      1) bring up Docker+Gazebo
      2) spawn robot in containers
      3) cycle through RMW impls and launch FuzzPublisher
      4) detect ASAN crashes and save data + logs
      5) restart containers on crash
      6) cleanup
    """
    INSPECTOR_IP = "192.168.10.100"
    DST_IP_MAP = {
        "rmw_fastrtps_cpp":   "192.168.10.10",
        "rmw_cyclonedds_cpp": "192.168.10.20",
    }

    DOMAIN_ID_MAP = {
        "rmw_fastrtps_cpp":   "1",
        "rmw_cyclonedds_cpp": "2",
    }

    DST_PORT_MAP = {
        "cmd_vel": {"1": 7665, "2": 7915, "3": 8165}
    }

    def __init__(self, version, robot, topic_name, headless=False, asan=False) -> None:
        self.version    = version
        self.robot      = robot
        self.topic_name = topic_name
        self.headless   = headless
        self.asan   = asan

        # initialize state counters
        self.crash_count = 0
        self.run_count   = 0
        self.stage       = 0
        self.round       = 0
        
        self.src_ip        = self.INSPECTOR_IP
        self.rtps          = RTPSPacket(self.topic_name)
        self.dds_config    = DDSConfig()
        self.container     = FuzzContainer(self.version, self.robot, self.DST_IP_MAP, self.DOMAIN_ID_MAP, self.src_ip, self.headless, self.asan)
        self.state_monitor = RobotStateMonitor(self.robot)

        # log initial state to state log
        msg = f"Initial state: version={self.version}, robot={self.robot}, topic={self.topic_name}"
        info(msg)
        with open(STATE_LOG, 'a') as f:
            f.write(f"{datetime.datetime.now().isoformat()} - {msg}\n")

    def _check_asan(self, log_path: str) -> bool:
        """Check if ASAN error appears in given log file."""
        try:
            with open(log_path, 'r', errors='ignore') as f:
                return 'AddressSanitizer' in f.read() or 'asan:' in f.read()
        except Exception:
            return False

    def _handle_crash(self) -> None:
        """Save crash data (packets, QoS, logs) and restart containers."""
        self.crash_count += 1
        timestamp   = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        crash_base  = os.path.join(CRASH_DIR, timestamp)
        packets_dir = os.path.join(crash_base, 'packets')
        os.makedirs(packets_dir, exist_ok=True)

        # save all mutated payloads
        for idx, payload in enumerate(self.rtps.mutated_payloads, start=1):
            path = os.path.join(packets_dir, f"mutated_{idx}.bin")
            with open(path, 'wb') as fp:
                fp.write(payload)

        # save QoS settings
        qos_file = os.path.join(crash_base, 'qos.txt')
        with open(qos_file, 'w') as fp:
            qos = self.dds_config.get_qos()
            fp.write(f"QoS Profile:\n")
            fp.write(f"  durability: {qos.durability.name}\n")
            fp.write(f"  history: {qos.history.name}\n")
            fp.write(f"  depth: {qos.depth}\n")
            fp.write(f"  liveliness: {qos.liveliness.name}\n")

        # log crash and state data
        msg = f"Crash #{self.crash_count} detected; data & logs saved to {crash_base}"
        info(msg)
        with open(STATE_LOG, 'a') as f:
            f.write(f"{datetime.datetime.now().isoformat()} - {msg}\n")

        shutil.copy(STATE_LOG, os.path.join(crash_base, 'state.log'))

        for fname in os.listdir(LOGS_DIR):
            src = os.path.join(LOGS_DIR, fname)
            if fname == os.path.basename(STATE_LOG):
                continue
            if fname.endswith('.log'):
                shutil.copy(src, os.path.join(crash_base, fname))
                with open(src, 'w') as f:
                    pass

        # restart containers
        self.container.close_docker()
        self.container.run_docker()
        self.container.run_gazebo()

    def _detect_and_handle(self) -> None:
        """Check ASAN in container logs and handle crash."""
        for rmw_impl in self.DST_IP_MAP:
            cname    = f"{self.version}_{self.robot}_{rmw_impl}"
            log_path = os.path.join(LOGS_DIR, f"{cname}.log")
            if self._check_asan(log_path):
                error(f"ASAN detected in container {cname}")
                self._handle_crash()
                break

    def gen_packet_sender(self, rmw_impl: str, mutated_payloads: list[bytes]) -> None:
        """Instantiate and run the FuzzPublisher node once."""
        os.environ["RMW_IMPLEMENTATION"] = rmw_impl
        os.environ["ROS_DOMAIN_ID"]      = self.DOMAIN_ID_MAP[rmw_impl]
        if not rclpy.ok():
            rclpy.init()
        self.run_count += 1
        msg = f"Run #{self.run_count}: RMW implementation = {rmw_impl}"
        info(msg)
        with open(STATE_LOG, 'a') as f:
            f.write(f"{datetime.datetime.now().isoformat()} - {msg}\n")

        node = None
        try:
            node = FuzzPublisher(
                robot      = self.robot,
                topic_name = self.topic_name,
                rtps       = self.rtps,
                rmw_impl   = rmw_impl,
                dds_id     = self.DOMAIN_ID_MAP[rmw_impl],
                qos        = self.dds_config.get_qos(),
                src_ip     = self.src_ip,
                dst_ip     = self.DST_IP_MAP[rmw_impl],
                dport      = self.DST_PORT_MAP[self.topic_name][self.DOMAIN_ID_MAP[rmw_impl]],
                container  = self.container,
                mutated_payloads = mutated_payloads,
                state_monitor = self.state_monitor
            )
            rclpy.spin_until_future_complete(node, node.future)
        except Exception as e:
            error(f"FuzzPublisher error: {e}")
        finally:
            
            if node:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            

    def run(self) -> None:
        # 1) start containers and Gazebo
        try:
            info(f"Bringing up containers & Gazebo for {self.version}/{self.robot}")
            self.container.run_docker()
            self.container.run_gazebo()
        except Exception as e:
            error(f"Container/Gazebo setup failed: {e}")
            self.container.close_docker()
            exit(1)

        # 2) main fuzz loop
        info("@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        info("Fuzz loop Entrance...")
        info("@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        self.round = 1
        try:
            while True:
                # rotate QoS and increment stage
                if (self.round - 1) % PACKETS_PER_QOS == 0:
                    self.dds_config.update_qos()
                    info(f"==> Stage {self.stage}: QoS settings updated")
                    self.stage += 1
                    self.round = 1
                    with open(STATE_LOG, 'a') as f:
                        f.write(f"{datetime.datetime.now().isoformat()} - Stage {self.stage}\n")

                # Generate mutated payloads once per round
                self.rtps.update_packet_mutation_strategy()
                strat   = self.rtps.packet_mutation_strategy
                weights = {s['func'].__name__: s['weight'] for s in self.rtps.strategies}
                # append to state log
                with open(STATE_LOG, 'a') as f:
                    f.write(f"{datetime.datetime.now().isoformat()} - Mutation strategy: {strat.__name__}, weights: {weights}\n")

                self.rtps.generate_mutated_payloads(MESSAGES_PER_RUN)

                # save mutated payloads to output/logs
                for idx, payload in enumerate(self.rtps.mutated_payloads, start=1):
                    filename = f"mutated_{idx}.bin"
                    path = os.path.join(LOGS_DIR, filename)
                    with open(path, "wb") as fp:
                        fp.write(payload)

                for rmw_impl in self.DST_IP_MAP.keys():
                    self.gen_packet_sender(rmw_impl, self.rtps.mutated_payloads)
                    self._detect_and_handle()
                    time.sleep(RUN_DELAY)

                with open(STATE_LOG, 'a') as f:
                    f.write(f"{datetime.datetime.now().isoformat()} - Round {self.round}\n")
                info(f"----> Round {self.round} completed")
                self.round += 1
                input()

        except KeyboardInterrupt:
            info("Interrupted by user")

        except Exception as e:
            error(f"Fuzzer encountered an error: {e}")

        finally:
            # 3) cleanup
            info("Deleting robot and tearing down containers")
            self.container.close_docker()
