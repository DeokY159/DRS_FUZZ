import json
import subprocess

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
        et = entry.get("endpoint_type", "")
        if et == "PUBLISHER":
            publisher = entry
        elif et in ("SUBSCRIPTION", "SUBSCRIBER"):
            subscriber = entry
    return publisher, subscriber

def get_topic_info(topic_name: str, max_wait_secs=30, **dump_kwargs) -> str:
    """
    Invoke `ros2 topic info` for the given topic, parse the output, and return
    a JSON-formatted string containing publisher and subscriber info.

    :param topic_name: The ROS2 topic name (e.g. '/cmd_vel')
    :param dump_kwargs: kwargs passed to json.dumps (e.g. indent)
    :return: JSON string with keys 'publisher' and 'subscriber'
    """
    cmd = ['ros2', 'topic', 'info', topic_name, '--verbose']

    try:
        proc = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, check=True
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Error executing ros2 topic info for '{topic_name}'") from e

    pub, sub = parse_topic_info(proc.stdout)
    if not pub.get("gid") or not sub.get("gid"):
        raise RuntimeError(f"Incomplete topic discovery for '{topic_name}'")
    
    return json.dumps({"publisher": pub, "subscriber": sub}, **dump_kwargs)
    
if __name__ == "__main__":
    info_json = get_topic_info('/cmd_vel', indent=4)
    print(info_json)
