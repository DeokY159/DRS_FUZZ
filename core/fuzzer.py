# core/fuzzer.py

import os
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

from scapy.all import sendp, Ether, IP, UDP
from scapy.contrib.rtps import RTPSMessage

import core.inspector as inspector
from core.executor import FuzzContainer
from core.mutator import RTPSPacket, DDSConfig
from core.ui import info, error

# --- Constants ---
RETRY_MAX_ATTEMPTS = 5      # retry attempts for transient failures
RETRY_DELAY        = 1.0    # seconds between retries
PACKETS_PER_QOS    = 100    # how often to rotate QoS
MESSAGES_PER_RUN   = 10     # messages per spin run
MESSAGE_PERIOD     = 1.0    # seconds between packets
UDP_SPORT          = 45569  # source UDP port for RTPS
CMD_VEL_DPORT      = 7415   # dest UDP port for /cmd_vel
RUN_DELAY          = 3.0    # seconds between different RMW runs

def get_host_internal_ip() -> str:
    """Get host IP by opening a UDP socket to the internet."""
    for attempt in range(1, RETRY_MAX_ATTEMPTS + 1):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except OSError as e:
            if attempt == RETRY_MAX_ATTEMPTS:
                error(f"Failed to determine host internal IP: {e}")
                sys.exit(1)
            info(f"Retrying IP lookup (attempt {attempt})")
            time.sleep(RETRY_DELAY)

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
    def __init__(
        self,
        robot: str,
        topic_name: str,
        rtps: RTPSPacket,
        rmw_impl: str,
        qos: QoSProfile,
        src_ip: str,
        dst_ip: str,
        iface: str
    ) -> None:
        super().__init__('fuzzer_publisher')
        self.robot    = robot
        self.rtps     = rtps
        self.rmw_impl = rmw_impl
        self.src_ip   = src_ip
        self.dst_ip   = dst_ip
        self.iface    = iface
        self.seq_num  = 1
        self.future   = Future()

        if topic_name == 'cmd_vel':
            self.publisher = self.create_publisher(Twist, topic_name, qos)
            self.dport     = CMD_VEL_DPORT
        else:
            error(f"Unsupported topic '{topic_name}'")
            sys.exit(1)

        # fetch topic info for base packet
        for attempt in range(1, RETRY_MAX_ATTEMPTS + 1):
            try:
                inspect_info = inspector.get_topic_info(f'/{topic_name}')
                break
            except Exception as e:
                if attempt == RETRY_MAX_ATTEMPTS:
                    error(f"Unable to get topic info: {e}")
                    sys.exit(1)
                info("Retrying get_topic_info()")
                time.sleep(RETRY_DELAY)

        # prepare mutated payloads
        self.rtps.build_base_packet(rmw_impl=rmw_impl, inspect_info=inspect_info)
        self.rtps.update_packet_mutation_strategy()
        self.rtps.generate_mutated_payloads(MESSAGES_PER_RUN)

        # timer triggers send loop
        self.create_timer(MESSAGE_PERIOD, self._timer_callback)

    def _timer_callback(self) -> None:
        if self.seq_num > MESSAGES_PER_RUN:
            info(f"Sent all messages for RMW='{self.rmw_impl}'")
            self.future.set_result(True)
            return

        self.rtps.mutate_packet(self.seq_num)
        send_packet(
            src_ip=self.src_ip,
            dst_ip=self.dst_ip,
            dport=self.dport,
            iface=self.iface,
            rtps=self.rtps
        )
        self.seq_num += 1

class Fuzzer:
    """
    Top-level controller:
      1) bring up Docker+Gazebo
      2) spawn robot in containers
      3) cycle through RMW impls and launch FuzzPublisher
      4) cleanup robot and containers
    """
    DST_IP_MAP = {
        "rmw_fastrtps_cpp":   "192.168.10.10",
        "rmw_cyclonedds_cpp": "192.168.10.20",
    }

    def __init__(self, version: str, robot: str, topic_name: str, iface: str) -> None:
        self.version    = version
        self.robot      = robot
        self.topic_name = topic_name
        self.iface      = iface

        self.src_ip     = get_host_internal_ip()
        self.rtps       = RTPSPacket(self.topic_name)
        self.dds_config = DDSConfig()
        self.container  = FuzzContainer(self.version, self.robot, self.DST_IP_MAP)

    def gen_packet_sender(self, rmw_impl: str) -> None:
        """Instantiate and run the FuzzPublisher node once."""
        os.environ["RMW_IMPLEMENTATION"] = rmw_impl
        if not rclpy.ok():
            rclpy.init()
        info(f"Running RMW implementation: {rmw_impl}")

        node = None
        try:
            node = FuzzPublisher(
                robot      = self.robot,
                topic_name = self.topic_name,
                rtps       = self.rtps,
                rmw_impl   = rmw_impl,
                qos        = self.dds_config.get_qos(),
                src_ip     = self.src_ip,
                dst_ip     = self.DST_IP_MAP[rmw_impl],
                iface      = self.iface
            )
            rclpy.spin_until_future_complete(node, node.future)
        except Exception as e:
            error(f"FuzzPublisher error: {e}")
        finally:
            if node:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

        info(f"Completed RMW='{rmw_impl}' run\n")

    def run(self) -> None:
        # 1) start containers and Gazebo, spawn robot
        try:
            info(f"Bringing up containers & Gazebo for {self.version}/{self.robot}")
            self.container.run_docker()
            self.container.run_gazebo()
            self.container.delete_robot()
        except Exception as e:
            error(f"Container/Gazebo setup failed: {e}")
            sys.exit(1)
        finally:
            info("Deleting robot and tearing down containers")
            #self.container.close_docker()


        # 2) main fuzz loop
        loop_count = 1
        try:
            while True:
                if loop_count % PACKETS_PER_QOS == 1:
                    self.dds_config.update_qos()
                    info("QoS settings updated")

                self.gen_packet_sender("rmw_fastrtps_cpp")
                time.sleep(RUN_DELAY)

                self.gen_packet_sender("rmw_cyclonedds_cpp")
                time.sleep(RUN_DELAY)

                loop_count += MESSAGES_PER_RUN

        except KeyboardInterrupt:
            info("Interrupted by user")

        except Exception as e:
            error(f"Fuzzer encountered an error: {e}")

        finally:
            # 3) cleanup
            info("Deleting robot and tearing down containers")
            self.container.close_docker()
