from core.mutator import RTPSPacket, DDSConfig
from core.oracle import parse_odom_from_log
from core.ui import info

def is_robot_stationary(robot: str) -> bool:
    stationary = {"rmw_fastrtps_cpp": False, "rmw_cyclonedds_cpp": False}

    if robot == 'turtlebot3':
        for rmw_impl in stationary:
            log_path = f"./output/logs/robot_states/{robot}/{rmw_impl}/odom.log"
            odom = parse_odom_from_log(log_path)
            
            x = round(odom['position_x'], 1)
            y = round(odom['position_y'], 1)
            if x in [-1.9, -2.0] and y in [-0.5, -0.4]:
                stationary[rmw_impl] = True

    return all(stationary.values())

def increase_mutation_weights(rtps: RTPSPacket, dds_config: DDSConfig, increment: float):
    current_s = rtps.packet_mutation_strategy
    for s in rtps.strategies:
        if s["func"] == current_s:
            s["weight"] += increment
            info(f'Mutation weight increased to {s["weight"]:.2f} for strategy "{s["func"].__name__}"')
            break

    if(dds_config, "qos"):
        current_qos = dds_config.qos
        for comb in dds_config.combinations:
            if comb["profile"] == current_qos:
                comb["weight"] += increment

                qos_desc = (
                    f"durability={current_qos.durability.name}, "
                    f"history={current_qos.history.name}, "
                    f"liveliness={current_qos.liveliness.name}, "
                    f"depth={current_qos.depth}"
                )

                info(f'Mutation weight increased to {comb["weight"]:.2f} for QoSProfile ({qos_desc})')
                break

def decrease_mutation_weights(rtps: RTPSPacket, dds_config: DDSConfig, increment: float):
    current_s = rtps.packet_mutation_strategy
    for s in rtps.strategies:
        if s["func"] == current_s:
            s["weight"] -= increment
            if s["weight"] < 0:
                s["weight"] == 0.0
            info(f'Mutation weight decreased to {s["weight"]:.2f} for strategy "{s["func"].__name__}"')
            break

    if(dds_config, "qos"):
        current_qos = dds_config.qos
        for comb in dds_config.combinations:
            if comb["profile"] == current_qos:
                comb["weight"] -= increment
                if comb["weight"] < 0:
                    comb["weight"] == 0.0

                qos_desc = (
                    f"durability={current_qos.durability.name}, "
                    f"history={current_qos.history.name}, "
                    f"liveliness={current_qos.liveliness.name}, "
                    f"depth={current_qos.depth}"
                )

                info(f'Mutation weight increased to {comb["weight"]:.2f} for QoSProfile ({qos_desc})')
                break

def adjust_mutation_weights(robot: str, rtps: RTPSPacket, dds_config: DDSConfig, increment: float, crash_or_bug: bool):
    if(is_robot_stationary(robot)):
        decrease_mutation_weights(rtps = RTPSPacket, dds_config = DDSConfig, increment = increment)
    if(crash_or_bug):
        increase_mutation_weights(rtps = RTPSPacket, dds_config = DDSConfig, increment = increment)