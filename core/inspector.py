import subprocess
import time
import json
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from core.ui import debug

TYPE_MAP = {
    '/cmd_vel': 'geometry_msgs/msg/Twist',  # humble
    #'/cmd_vel': 'geometry_msgs/msg/TwistStamped', # Jazzy
    '/chatter': 'std_msgs/msg/String',
}

def parse_topic_info(raw_output: str) -> tuple[dict, dict]:
    """
    Parse the verbose output of `ros2 topic info` into publisher and subscriber dicts.

    :param raw_output: Raw CLI output string
    :return: (publisher_dict, subscriber_dict)
    """
    blocks = raw_output.strip().split("\n\n")
    publisher = {}
    subscriber = {}

    for block in blocks:
        lines = block.strip().splitlines()
        entry = {}
        for line in lines:
            if line.startswith("Node name:"):
                entry["node_name"] = line.split(":", 1)[1].strip()
            elif line.startswith("Node namespace:"):
                entry["namespace"] = line.split(":", 1)[1].strip()
            elif line.startswith("Topic type:"):
                entry["topic_type"] = line.split(":", 1)[1].strip()
            elif line.startswith("Endpoint type:"):
                entry["endpoint_type"] = line.split(":", 1)[1].strip()
            elif line.startswith("GID:"):
                entry["gid"] = line.split(":", 1)[1].strip()
            elif line.strip().startswith("Reliability:"):
                entry["reliability"] = line.split(":", 1)[1].strip()
            elif line.strip().startswith("History"):
                entry["history"] = line.split(":", 1)[1].strip()
            elif line.strip().startswith("Durability:"):
                entry["durability"] = line.split(":", 1)[1].strip()

        n_name = entry.get("node_name", "") 
        e_type = entry.get("endpoint_type", "")
        if e_type == "PUBLISHER" and not n_name.startswith("_"):
            publisher = entry
        elif e_type in "SUBSCRIPTION" and not n_name.startswith("_"):
            subscriber = entry
    return publisher, subscriber

def create_publisher(topic_name: str,
                     container: str,
                     rmw_impl: str,
                     domain_id: str,
                     qos_profile: QoSProfile = None,
                    ) -> None:
    """
    Make DDS participant in inspector container for getting topic info
    """
    msg_type = TYPE_MAP.get(topic_name)
    if msg_type is None:
        raise RuntimeError(f"Unknown topic '{topic_name}'")

    # QoSProfile → CLI 플래그
    flags = []
    if qos_profile:
        hist_map = {
            QoSHistoryPolicy.KEEP_LAST: 'keep_last',
            QoSHistoryPolicy.KEEP_ALL:  'keep_all',
        }
        flags += ['--qos-history', hist_map[qos_profile.history]]
        flags += ['--qos-depth',   str(qos_profile.depth)]
        rel_map = {
            QoSReliabilityPolicy.BEST_EFFORT: 'best_effort',
            QoSReliabilityPolicy.RELIABLE:    'reliable',
        }
        flags += ['--qos-reliability', rel_map[qos_profile.reliability]]
        dur_map = {
            QoSDurabilityPolicy.VOLATILE:        'volatile',
            QoSDurabilityPolicy.TRANSIENT_LOCAL: 'transient_local',
        }
        flags += ['--qos-durability', dur_map[qos_profile.durability]]

    base = f"ros2 topic pub {topic_name} {msg_type} '{{}}' --rate 0.01"
    cmd  = base + ' ' + ' '.join(flags)
    try:
        subprocess.run(['docker','exec','-d', 
                        ### TODO: [option] If you want cross ros2 dds implementaiton testing, you can change rmw_implementation environ
                        ### (e.g. fuzzer_<ROS_DISTRO>_turtlebot3)
                        ### Default: '-e', f"RMW_IMPLEMENTATION={rmw_impl}"

                        '-e', f"RMW_IMPLEMENTATION={rmw_impl}",
                        # '-e', f"RMW_IMPLEMENTATION=rmw_fastrtps_cpp",

                        '-e', f"ROS_DOMAIN_ID={domain_id}",
                        container,'bash','-ic', cmd], check=True)
    except Exception as e:
        raise subprocess.SubprocessError(f"Failed to create publisher: {e}")
    
def get_topic_info(topic_name: str,
                   container: str,
                   rmw_impl: str,
                   domain_id: str,
                   **dump_kwargs) -> str:
    """
    Called `ros2 topic info --verbose` for getting "GID"
    """
    cmd = f"ros2 topic info {topic_name} --verbose"

    proc = subprocess.run([
        'docker', 'exec',
        '-e', f"RMW_IMPLEMENTATION={rmw_impl}",
        '-e', f"ROS_DOMAIN_ID={domain_id}",
        container, 'bash', '-ic', cmd
    ], check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    
    # Parse topic info
    blocks = proc.stdout.strip().split("\n\n")
    pub, sub = {}, {}
    for blk in blocks:
        entry = {}
        for line in blk.splitlines():
            k, _, v = line.partition(':')
            entry[k.strip()] = v.strip()
        et = entry.get("Endpoint type", "")
        if et == "PUBLISHER":  pub = entry
        if et in ("SUBSCRIPTION", "SUBSCRIBER"): sub = entry
    
    if not pub.get("GID") or not sub.get("GID"):
        raise RuntimeError(f"Incomplete discovery for '{topic_name}'")

    return json.dumps({
        "publisher": pub,
        "subscriber": sub
    }, **dump_kwargs)


def stop_publisher(topic_name: str, container: str) -> None:
    """
    kill create_publisher
    """
    subprocess.run([
        'docker', 'exec', container,
        'pkill', '-f', f"ros2 topic pub {topic_name}"
    ], check=False)