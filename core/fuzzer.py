# core/fuzzer.py
import os
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

import core.feedback as feedback
import core.inspector as inspector
import core.oracle as oracle
from core.executor import FuzzContainer, RobotStateMonitor
from core.mutator import RTPSPacket, DDSConfig
from core.ui import info, error, warn, debug, done

# --- Constants ---
RETRY_MAX_ATTEMPTS = 10     # retry attempts for transient failures
RETRY_DELAY        = 1.0    # seconds between retries
RUN_DELAY          = 2.0    # seconds between different RMW runs
PACKETS_PER_QOS    = 10     # how often to rotate QoS (in runs)
MESSAGES_PER_RUN   = 10     # messages per spin run
MESSAGE_PERIOD     = 0.5    # seconds between packets
UDP_SPORT          = 45569  # source UDP port for RTPS

# base directories
OUTPUT_DIR    = os.path.join(os.getcwd(), 'output')
LOGS_DIR      = os.path.join(OUTPUT_DIR, 'logs')
CRASH_DIR     = os.path.join(OUTPUT_DIR, 'crash')
BUG_DIR       = os.path.join(OUTPUT_DIR, 'semantic_bug')
STATE_LOG     = os.path.join(LOGS_DIR, 'current_state.log')

# ensure output directories exist
os.makedirs(LOGS_DIR, exist_ok=True)
os.makedirs(CRASH_DIR, exist_ok=True)
os.makedirs(BUG_DIR, exist_ok=True)

# initialize state log (clear or create)
with open(STATE_LOG, 'w') as f:
    f.write(f"{datetime.datetime.now().isoformat()} - Fuzzing state log created\n")
    f.write(f"{datetime.datetime.now().isoformat()} - Initial Config: ")
    f.write(f"RETRY_MAX_ATTEMPTS={RETRY_MAX_ATTEMPTS}, ")
    f.write(f"RETRY_DELAY={RETRY_DELAY}s, ")
    f.write(f"RUN_DELAY={RUN_DELAY}s, ")
    f.write(f"PACKETS_PER_QOS={PACKETS_PER_QOS}, ")
    f.write(f"MESSAGES_PER_RUN={MESSAGES_PER_RUN}, ")
    f.write(f"MESSAGE_PERIOD={MESSAGE_PERIOD}/sec\n")
    

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

        try:
            container.spawn_robot(self.rmw_impl)
        except Exception as e:
            raise RuntimeError(f"Failed to spawn robot: {e}")

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
            except Exception as e:
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
            time.sleep(RETRY_DELAY*3)
            try:
                self.state_monitor.record_robot_states(self.rmw_impl, self.dds_id)
            except TimeoutError as e:
                raise TimeoutError(f"{e}")
            done(f"Sent all messages for RMW='{self.rmw_impl}'")
            self.timer.cancel()
            time.sleep(RETRY_DELAY)
            self.container.delete_robot(self.rmw_impl)
            self.future.set_result(True)
            time.sleep(RETRY_DELAY)
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

    def __init__(self, version, robot, topic_name, headless=False, asan=False) -> None:
        self.version    = version
        self.robot      = robot
        self.topic_name = topic_name
        self.headless   = headless
        self.asan   = asan

        # initialize state counters
        self.crash_count = 0
        self.error_count = 0
        self.bug_count   = 0
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
        if headless:
            msg += f", headless={self.headless}"
        if asan:
            msg += f", asan={self.asan}"
            
        info(msg)
        with open(STATE_LOG, 'a') as f:
            f.write(f"{datetime.datetime.now().isoformat()} - {msg}\n")

    def copy_logs(self, type: str) -> None:
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

        if type == 'crash':
            base_dir = os.path.join(CRASH_DIR, timestamp+"_"+type)
            msg = f"Crash #{self.crash_count} detected; data & logs saved to {base_dir}"
            error(msg)

        elif type == 'semantic_bug':
            base_dir = os.path.join(BUG_DIR, timestamp+"_"+type)
            msg = f"Semantic Bug #{self.bug_count} detected; data & logs saved to {base_dir}"
            error(msg)

        with open(STATE_LOG, 'a') as f:
            f.write(f"{datetime.datetime.now().isoformat()} - {msg}\n")

        if os.path.isdir(LOGS_DIR):
            shutil.copytree(LOGS_DIR, base_dir, dirs_exist_ok=True)

        qos_file = os.path.join(base_dir, 'qos.txt')
        with open(qos_file, 'w') as fp:
            qos = self.dds_config.get_qos()
            fp.write(f"QoS Profile:\n")
            fp.write(f"  durability: {qos.durability.name}\n")
            fp.write(f"  history: {qos.history.name}\n")
            fp.write(f"  depth: {qos.depth}\n")
            fp.write(f"  liveliness: {qos.liveliness.name}\n")

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
                self.crash_count +=1
                error(f"ASAN detected in container {cname}")
                # Save crash logs and restart
                self.copy_logs(type="crash")
                if self.rtps and self.dds_config:
                    feedback.increase_mutation_weights(self.rtps, self.dds_config, 0.5)
                raise RuntimeError("Restart container after crash ...")

    def gen_packet_sender(self, rmw_impl: str, mutated_payloads: list[bytes]) -> None:
        """Instantiate and run the FuzzPublisher node once."""
        if not rclpy.ok():
            rclpy.init()
        self.run_count += 1
        msg = f"Run #{self.run_count}: RMW implementation = {rmw_impl}"
        #debug(msg)
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
        except RuntimeError as e:
            self.check_asan_crash()
            raise RuntimeError(f"FuzzPublisher runtime error: {e}")
        except TimeoutError as e:
            self.check_asan_crash()
            raise TimeoutError(f"FuzzPublisher timeout error: {e}")
        except (OSError, subprocess.SubprocessError) as e:
            self.check_asan_crash()
            raise subprocess.SubprocessError(f"FuzzPublisher subprocess error: {e}")
        finally:
            if node:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            

    def run(self) -> None:   
        while True:
            # 1) start containers and Gazebo
            try:
                info(f"Bringing up containers & Gazebo for {self.version}/{self.robot}")
                self.container.run_docker()
                self.container.run_gazebo()
                info("@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                info("Fuzz loop Entrance...")
                info("@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                self.round = 1
                while True:
                    # rotate QoS and increment stage
                    fast_log = os.path.join(LOGS_DIR, "dds_api", "fast_listener.log")
                    cyclone_log = os.path.join(LOGS_DIR, "dds_api", "cyclone_listener.log")
                    with open(fast_log, 'w') as f:
                        f.write("")
                    with open(cyclone_log, 'w') as f:
                        f.write("")                       
    
                    if (self.round - 1) % PACKETS_PER_QOS == 0:
                        self.dds_config.update_qos()
                        info(f"==> Stage {self.stage}: QoS settings updated")
                        self.round = 1
                        self.stage += 1
                        qos = self.dds_config.get_qos()
                        self.rtps._select_input_seed()

                        with open(STATE_LOG, 'a') as f:
                            f.write(f"{datetime.datetime.now().isoformat()} - Stage {self.stage}\n")
                            f.write(f"{datetime.datetime.now().isoformat()} - Seed Selected: {self.rtps.seed_path}\n")
                            f.write(f"{datetime.datetime.now().isoformat()} - QoS Setting durability={qos.durability.name}, history={qos.history.name}, depth={qos.depth}, liveliness={qos.liveliness.name}\n")
                            
                    with open(STATE_LOG, 'a') as f:
                        f.write(f"{datetime.datetime.now().isoformat()} - Round {self.round}\n")

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
                        self.check_asan_crash()
                        time.sleep(RUN_DELAY)

                    if oracle.check_robot_states_diff(robot=self.robot, threshold=30.0):
                        feedback.increase_mutation_weights(self.rtps, self.dds_config, 0.5)

                        if oracle.compare_listener(fast_log, cyclone_log, self.topic_name):
                            feedback.increase_mutation_weights(self.rtps, self.dds_config, 0.5)

                        self.bug_count += 1
                        self.copy_logs("semantic_bug")    

                    if feedback.is_robot_stationary(self.robot):
                        feedback.decrease_mutation_weights(self.rtps, self.dds_config, 0.5)

                    self.round += 1
                    
            except (RuntimeError,TimeoutError) as e:
                error(f"Error Occured - {e}")
                self.error_count += 1
                warn("Cleaning up fuzzing container for restart...")
                self.container.close_docker()
                with open(STATE_LOG, 'a') as f:
                    f.write(f"{datetime.datetime.now().isoformat()} - Error #{self.error_count} occured {e}\n")
                time.sleep(RUN_DELAY)
                continue 
            except KeyboardInterrupt:
                error("Interrupted by user (ctrl+c)")
                self.container.close_docker()
                exit(0)
            except Exception as e:
                error(f"Error occured in Fuzzing Process - {e}")
                self.container.close_docker()
                exit(0)
