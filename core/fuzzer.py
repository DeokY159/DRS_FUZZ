import os
import socket
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

from scapy.all import sendp, Ether, IP, UDP
from scapy.contrib.rtps import RTPSMessage

import inspector
from mutator import RTPSPacket, DDSConfig
from ui import info, error

# --- Constants ---
RETRY_MAX_ATTEMPTS = 5      # Maximum number of attempts for retryable operations
RETRY_DELAY        = 1.0    # Delay between retry attempts (seconds)
PACKETS_PER_QOS    = 100    # Change QoS after this many packets have been sent
MESSAGES_PER_RUN   = 10     # Number of messages to send in each run invocation
MESSAGE_PERIOD     = 1.0    # Period between messages (seconds)
UDP_SPORT          = 45569  # Source UDP port for RTPS packets
CMD_VEL_DPORT      = 7415   # Destination UDP port for /cmd_vel RTPS packets
RUN_DELAY          = 3.0    # Delay between runs with different RMW implementations

def get_host_internal_ip() -> str:
    """
    Return the host's internal IP address by opening a UDP socket.
    """
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
            info(f"Retrying getting host's ip (ATTEMPT : {attempt})")
            time.sleep(RETRY_DELAY)

def send_packet(src_ip: str, dst_ip: str, dport: int, iface: str, rtps: RTPSPacket) -> None:
    """
    Build and send a single RTPS packet via Scapy.
    - src_ip: source IP address
    - dst_ip: destination IP address
    - dport: destination UDP port
    - iface: network interface to send on
    - rtps: RTPSPacket instance containing hdr, dst, ts, data
    """
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
    ROS2 node that:
      - Spawns a Gazebo robot
      - Builds an RTPS base packet
      - Generates MESSAGES_PER_RUN mutated payloads
      - Sends them at MESSAGE_PERIOD intervals
    """
    ROBOT_MODELS = {
        "turtlebot3": "burger",
    }

    def __init__(self, robot: str, topic_name: str, rtps: RTPSPacket, rmw_impl: str, qos: QoSProfile, src_ip: str, dst_ip: str, iface: str) -> None:
        super().__init__('fuzzer_publisher')
        self.robot    = robot
        self.rtps     = rtps
        self.rmw_impl = rmw_impl
        self.src_ip   = src_ip
        self.dst_ip   = dst_ip
        self.iface    = iface
        self.seq_num  = 1               # Sequence number for packets
        self.future   = Future()        # Used to signal completion

        if topic_name == 'cmd_vel':
            self.publisher = self.create_publisher(Twist, topic_name, qos)
            self.dport = CMD_VEL_DPORT

        else:
            raise ValueError(f"Unsupported topic '{topic_name}'")
        
        # Spawn the robot model in Gazebo
        self._spawn_robot()

        # Retrieve publisher/subscriber info via ros2 CLI
        for attempt in range(1, RETRY_MAX_ATTEMPTS + 1):
            try:
                inspect_info = inspector.get_topic_info(f'/{topic_name}')
                break
            except Exception as e:
                if attempt == RETRY_MAX_ATTEMPTS:
                    error(f"Unable to get topic info: {e}")
                    sys.exit(1)
                info(f"Retrying getting topic info")  

        # Choose an initial mutation strategy and generate payloads
        self.rtps.build_base_packet(rmw_impl=rmw_impl, inspect_info=inspect_info)
        self.rtps.update_packet_mutation_strategy()
        self.rtps.generate_mutated_payloads(MESSAGES_PER_RUN)

        # Set up a timer to send mutated packets at a fixed period
        self.timer = self.create_timer(MESSAGE_PERIOD, self.timer_callback)

    def _spawn_robot(self) -> None:
        """
        Spawn the specified robot model in Gazebo via ros2 run gazebo_ros spawn_entity.py.
        """
        if self.robot == 'turtlebot3':
            pkg_share = get_package_share_directory('turtlebot3_gazebo')
            sdf_path = os.path.join(pkg_share, 'models', 'turtlebot3_burger', 'model.sdf')

        else:
            error(f"Unsupported robot : '{self.robot}'")
            sys.exit(1)

        cmd = [
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', self.ROBOT_MODELS[self.robot],
            '-file', sdf_path,
            '-x', '0.0', '-y', '0.0', '-z', '0.01'
        ]

        for attempt in range(1, RETRY_MAX_ATTEMPTS + 1):
            try:
                subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
                return
            except subprocess.CalledProcessError as e:
                if attempt == RETRY_MAX_ATTEMPTS:
                    error(f"Falied to spawn robot(MODEL={self.ROBOT_MODELS[self.robot]}): {e}")
                    sys.exit(1)

                info(f"Retrying spawning robot(MODEL={self.ROBOT_MODELS[self.robot]}) (ATTEMPT : {attempt})")
                time.sleep(RETRY_DELAY)

    def _delete_robot(self) -> None:
        """
        Delete the spawned robot entity from Gazebo via ROS2 service call.
        """
        delete_srv_args = f"{{name: '{self.ROBOT_MODELS[self.robot]}'}}"
        cmd = [
            'ros2', 'service', 'call', '/delete_entity',
            'gazebo_msgs/srv/DeleteEntity', delete_srv_args
        ]
        for attempt in range(1, RETRY_MAX_ATTEMPTS + 1):
            try:
                subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
                info(f"Robot(MODEL={self.ROBOT_MODELS[self.robot]}) deleted successfully.")
                return 
            except subprocess.CalledProcessError as e:
                if attempt == RETRY_MAX_ATTEMPTS:
                    error(f"Failed to delete robot: {e}")
                    sys.exit(1)

                info(f"Retrying deleting robot(MODEL={self.ROBOT_MODELS[self.robot]}) (ATTEMPT : {attempt})")
                time.sleep(RETRY_DELAY)

    def timer_callback(self) -> None:
        """
        Called at MESSAGE_PERIOD intervals. Mutates and sends one packet each time.
        After MESSAGES_PER_RUN packets, it stops, deletes the robot, and signals completion.
        """
        if self.seq_num > MESSAGES_PER_RUN:
            info(f"Sent all messages. (RMW : {self.rmw_impl})")
            self.timer.cancel()
            self._delete_robot()
            self.future.set_result(True)
        else:
            # Mutate the payload for this sequence number and send
            self.rtps.mutate_packet(self.seq_num)
            send_packet(src_ip=self.src_ip, dst_ip=self.dst_ip, dport=self.dport, iface=self.iface, rtps=self.rtps)
            self.seq_num += 1

class Fuzzer:
    """
    Top‐level controller that cycles through different RMW implementations,
    updates QoS every PACKETS_PER_QOS messages, and launches FuzzPublisher runs.
    """
    DST_IP_MAP = {
        "rmw_fastrtps_cpp": "192.168.0.7",
        "rmw_cyclonedds_cpp": "192.168.0.7",
        "rmw_opendds_cpp": "192.168.0.7",
    }

    def __init__(self, robot: str, topic_name: str, iface: str) -> None:
        self.robot          = robot
        self.topic_name     = topic_name
        self.iface          = iface
        self.src_ip         = get_host_internal_ip()
        self.rtps           = RTPSPacket(self.topic_name)
        self.dds_config     = DDSConfig()

    def run_with_rmw(self, rmw_impl: str) -> None:
        """
        Initialize rclpy, create and spin a FuzzPublisher node for one run.
        """
        os.environ["RMW_IMPLEMENTATION"] = rmw_impl

        if not rclpy.ok():
            rclpy.init()

        info(f"Run with {rmw_impl}")

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
        finally:
            node.destroy_node()
            rclpy.shutdown()
        info(f"Publisher with {rmw_impl} Terminated\n")
    
    def run(self) -> None:
        """
        Main loop: every PACKETS_PER_QOS messages update QoS, then
        cycle through each RMW implementation.
        """
        fuzz_loop = 1
        try:
            while True:
                # Update QoS combination every PACKETS_PER_QOS messages
                if fuzz_loop % PACKETS_PER_QOS == 1:
                    self.dds_config.update_qos()
                    info("QoS combination has changed")

                # Run publisher for each RMW variant
                self.run_with_rmw("rmw_fastrtps_cpp")
                time.sleep(RUN_DELAY)
                self.run_with_rmw("rmw_cyclonedds_cpp")
                time.sleep(RUN_DELAY)
                self.run_with_rmw("rmw_opendds_cpp")
                time.sleep(RUN_DELAY)
                fuzz_loop += MESSAGES_PER_RUN
        except KeyboardInterrupt:
            info("Terminating Fuzzer...")

if __name__ == '__main__':
    fuzzer = Fuzzer(robot="turtlebot3", topic_name="cmd_vel", iface="ens33")
    fuzzer.run()