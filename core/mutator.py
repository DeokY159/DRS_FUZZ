#!/usr/bin/env python3
import json
import os
import random
import time

from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
)

from scapy.contrib.rtps import (
    RTPS,
    ProtocolVersionPacket,
    VendorIdPacket,
    GUIDPrefixPacket,
    RTPSSubMessage_INFO_DST,
    RTPSSubMessage_INFO_TS,
    RTPSSubMessage_DATA,
    DataPacket,
)

__all__ = ["RTPSPacket", "DDSConfig"]

def bit_flip(arr: bytearray, bound: float) -> None:
    """
    Randomly flip individual bits in the array to introduce small perturbations.

    :param arr: The bytearray to modify.
    :param bound: Fraction of bytes to touch (0.0–1.0).
    """
    n = len(arr)
    k = min(n, max(1, int(bound * n)))

    for i in random.sample(range(n), k):
        bit_position = random.randrange(8)
        arr[i] ^= (1 << bit_position)

def byte_flip(arr: bytearray, bound: float) -> None:
    """
    Randomly invert entire bytes in the array to introduce maximum perturbation.

    :param arr: The bytearray to modify.
    :param bound: Fraction of bytes to touch (0.0–1.0).
    """
    n = len(arr)
    k = min(n, max(1, int(bound * n)))

    for i in random.sample(range(n), k):
        arr[i] ^= 0xFF

def interesting_byte(arr: bytearray, bound: float) -> None:
    """
    Replace random bytes with one of {0x00, 0xFF, 0x7F, 0x80} to hit edge cases.

    :param arr: The bytearray to modify.
    :param bound: Fraction of bytes to touch (0.0–1.0).
    """
    # Define a list of byte values that often reveal interesting behavior:
    # 0x00: all bits cleared,  
    # 0xFF: all bits set,  
    # 0x7F: highest signed positive value (0111_1111),  
    # 0x80: highest signed negative if interpreted as signed (1000_0000).
    interesting = [0x00, 0xFF, 0x7F, 0x80]

    n = len(arr)
    k = min(n, max(1, int(bound * n)))

    for i in random.sample(range(n), k):
        arr[i] = random.choice(interesting)

def arithmetic(arr: bytearray, bound: float) -> None:
    """
    Add or subtract small deltas to random bytes to introduce arithmetic perturbations.

    :param arr: The bytearray to modify.
    :param bound: Fraction of bytes to touch (0.0–1.0).
    """
    # Define a list of integer deltas to add or subtract:
    #  +1   : increment by one,
    #  -1   : decrement by one,
    #  +0x7F: large positive jump (127),
    #  -0x7F: large negative jump (-127).
    deltas = [1, -1, 0x7F, -0x7F]

    n = len(arr)
    k = min(n, max(1, int(bound * n)))

    for i in random.sample(range(n), k):
        arr[i] = (arr[i] + random.choice(deltas)) & 0xFF

def swap_bytes(arr: bytearray, bound: float) -> None:
    """
    Swap pairs of bytes at random positions to introduce positional perturbations.

    :param arr: The bytearray to modify.
    :param bound: Fraction of positions to swap (0.0–1.0).
    """
    n = len(arr)
    k = min(n, max(1, int(bound * n)))

    for _ in range(k):
        i, j = random.sample(range(n), 2)
        arr[i], arr[j] = arr[j], arr[i]


class RTPSPacket:
    """
    Encapsulates RTPS packet creation and payload mutation.
    - Loads a random seed payload from disk on init.
    - Supports multiple mutation strategies with weights.
    - Builds RTPS header/info/data submessages from ROS2 inspect_info.
    - Allows pre-generating N mutated payloads, then swapping them in by sequence number.
    """

    # Available mutation strategies and their default weights
    STRATEGY_SETTINGS = [
        {"func": bit_flip,         "weight": 1},
        {"func": byte_flip,        "weight": 1},
        {"func": interesting_byte, "weight": 1},
        {"func": arithmetic,       "weight": 1},
        {"func": swap_bytes,       "weight": 1},
    ]

    # DDS Vendor IDs and version mapping by RMW implementation
    VENDOR_ID_MAP = {
        "rmw_fastrtps_cpp": 0x010F,
        "rmw_cyclonedds_cpp": 0x0110,
        "rmw_opendds_cpp":   0x0103,
    }
    VENDOR_VERSION_MAP = {
        "rmw_fastrtps_cpp": {"major": 2, "minor": 3},
        "rmw_cyclonedds_cpp": {"major": 2, "minor": 1},
        "rmw_opendds_cpp": {"major": 2, "minor": 1},
    }

    def __init__(self, topic_name: str, seed_dir: str | None=None, bound: float = 1.0) -> None:
        """
        :param topic_name: ROS2 topic.
        :param seed_dir:   Optional custom directory of seed files.
        :param bound:      Mutation intensity (0.0–1.0).
        """
        self.topic_name = topic_name
        self.bound      = bound
        self.seed_dir   = seed_dir or os.path.join("./seed_payload", topic_name)

        self._initialize_packet_mutation_strategies()   
        self._select_input_seed()
        self.mutated_payloads: list[bytes] = []

    def _build_header(self, rmw_impl: str) -> RTPS:
        """
        Build the RTPS header for the given RMW implementation.
        """
        if rmw_impl not in self.VENDOR_ID_MAP:
            raise ValueError(f"Unsupported RMW implementation: '{rmw_impl}'")
        
        ver = self.VENDOR_VERSION_MAP[rmw_impl]
        return RTPS(
            magic=b"RTPS",
            protocolVersion=ProtocolVersionPacket(**ver),
            vendorId=VendorIdPacket(vendor_id=self.VENDOR_ID_MAP[rmw_impl]),
            guidPrefix=GUIDPrefixPacket(
                hostId=int.from_bytes(self.w_prefix[:4], 'big'),
                appId=int.from_bytes(self.w_prefix[4:8], 'big'),
                instanceId=int.from_bytes(self.w_prefix[8:12], 'big'),
            ),
        )

    def _build_info_dst(self) -> RTPSSubMessage_INFO_DST:
        """
        Build an INFO_DST submessage.
        """
        return RTPSSubMessage_INFO_DST(
            submessageFlags=0x01,
            octetsToNextHeader=12,
            guidPrefix=GUIDPrefixPacket(
                hostId=int.from_bytes(self.r_prefix[:4], 'big'),
                appId=int.from_bytes(self.r_prefix[4:8], 'big'),
                instanceId=int.from_bytes(self.r_prefix[8:12], 'big'),
            ),
        )

    def _build_info_ts(self) -> RTPSSubMessage_INFO_TS:
        """
        Build an INFO_TS submessage with the current timestamp.
        """
        now = time.time()
        sec = int(now)
        frac = int((now - sec) * (2**32))
        return RTPSSubMessage_INFO_TS(
            submessageFlags=0x01,
            octetsToNextHeader=8,
            ts_seconds=sec,
            ts_fraction=frac,
        )

    def _build_data(self) -> RTPSSubMessage_DATA:
        """
        Build a DATA submessage using self.seed_payload.
        """
        r_key = int.from_bytes(self.r_eid[:3], 'big')
        r_kind = self.r_eid[3]
        w_key = int.from_bytes(self.w_eid[:3], 'big')
        w_kind = self.w_eid[3]
        return RTPSSubMessage_DATA(
            submessageFlags=0x05,
            octetsToNextHeader=24 + len(self.seed_payload),
            extraFlags=0,
            octetsToInlineQoS=16,
            readerEntityIdKey=r_key,
            readerEntityIdKind=r_kind,
            writerEntityIdKey=w_key,
            writerEntityIdKind=w_kind,
            writerSeqNumHi=0,
            writerSeqNumLow=1,
            data=DataPacket(
                encapsulationKind=0x0001,
                encapsulationOptions=0x0000,
                serializedData=self.seed_payload,
            ),
        )
    
    def _initialize_packet_mutation_strategies(self) -> None:
        """Clone STRATEGY_SETTINGS into an instance list for weight-based selection."""
        self.strategies = [
            {"func": s["func"], "weight": s["weight"]}
            for s in self.STRATEGY_SETTINGS
        ]

    def update_packet_mutation_strategy(self) -> None:
        """
        Select one mutation strategy at random (by weight)
        and store it in self.packet_mutation_strategy.
        """
        weights = [s["weight"] for s in self.strategies]
        idx = random.choices(range(len(weights)), weights=weights, k=1)[0]
        selected = self.strategies[idx]
        self.packet_mutation_strategy = selected["func"]

    def _select_input_seed(self) -> None:     
        """
        Pick a random file from seed_dir and load it into self.seed_payload.
        """   
        if not os.path.isdir(self.seed_dir):
            raise FileNotFoundError(f"Directory not found '{self.seed_dir}'")
        
        seed_list = [f for f in os.listdir(self.seed_dir) if os.path.isfile(os.path.join(self.seed_dir, f))]

        if not seed_list:
            raise RuntimeError(f"No seed files found in: {self.seed_dir}")
        
        seed_file = random.choice(seed_list)
        seed_file_path = os.path.join(self.seed_dir, seed_file)

        try:
            with open(seed_file_path, 'rb') as f:
                self.seed_payload = f.read()
        except OSError as e:
            raise IOError(f"Unable to read the seed file '{seed_file_path}': {e}") from e

    def generate_mutated_payloads(self, mutation_cnt: int = 10) -> None:
        """
        Pre-generate mutation_cnt payloads by applying the currently
        selected strategy to copies of seed_payload.
        """
        if not hasattr(self, 'packet_mutation_strategy'):
            raise RuntimeError("mutation strategy not set: call update_packet_mutation_strategy() first")

        self.mutated_payloads.clear()
        for _ in range(mutation_cnt):
            arr = bytearray(self.seed_payload)
            self.packet_mutation_strategy(arr, self.bound)
            self.mutated_payloads.append(bytes(arr))

    def build_base_packet(self, rmw_impl: str, inspect_info: str) -> None:
        """
        Parse inspect_info JSON for GIDs, set prefixes/IDs,
        then build a base RTPS packet.
        """
        try:
            info = json.loads(inspect_info)
            wg = bytes(int(x, 16) for x in info["publisher"]["gid"].split("."))
            rg = bytes(int(x, 16) for x in info["subscriber"]["gid"].split("."))
        except (json.JSONDecodeError, KeyError) as e:
            raise RuntimeError(f"Invalid inspect_info format: {e}") from e

        self.w_prefix, self.w_eid = wg[:12], wg[12:16]
        self.r_prefix, self.r_eid = rg[:12], rg[12:16]

        self.hdr  = self._build_header(rmw_impl)
        self.dst  = self._build_info_dst()
        self.ts   = self._build_info_ts()
        self.data = self._build_data()

    def mutate_packet(self, seq_num: int) -> None:
        """
        Update self.data (and self.ts) with:
         - new timestamp
         - writerSeqNumLow = seq_num
         - serializedData = mutated_payloads[seq_num-1]
        """
        try:
            self.ts = self._build_info_ts()
            self.data.data.serializedData = self.mutated_payloads[seq_num - 1]
            self.data.writerSeqNumLow = seq_num
        except AttributeError as e:
            raise AttributeError(f"Base packet is not initialized before mutation: {e}") from e
        except IndexError as e:
            raise IndexError(f"Invalid index reference: {e}") from e

class DDSConfig:
    """
    Manages a collection of QoSProfile combinations.
    Calling update_qos() selects one at random (by weight) and exposes it as self.qos.
    """
    QOS_SETTINGS = [
        {
            "durability": DurabilityPolicy.VOLATILE,
            "history": HistoryPolicy.KEEP_LAST,
            "liveliness": LivelinessPolicy.AUTOMATIC,
            "weight": 1,
        },
        {
            "durability": DurabilityPolicy.VOLATILE,
            "history": HistoryPolicy.KEEP_LAST,
            "liveliness": LivelinessPolicy.MANUAL_BY_TOPIC,
            "weight": 1,
        },
        {
            "durability": DurabilityPolicy.VOLATILE,
            "history": HistoryPolicy.KEEP_ALL,
            "liveliness": LivelinessPolicy.AUTOMATIC,
            "weight": 1,
        },
        {
            "durability": DurabilityPolicy.VOLATILE,
            "history": HistoryPolicy.KEEP_ALL,
            "liveliness": LivelinessPolicy.MANUAL_BY_TOPIC,
            "weight": 1,
        },
        {
            "durability": DurabilityPolicy.TRANSIENT_LOCAL,
            "history": HistoryPolicy.KEEP_LAST,
            "liveliness": LivelinessPolicy.AUTOMATIC,
            "weight": 1,
        },
        {
            "durability": DurabilityPolicy.TRANSIENT_LOCAL,
            "history": HistoryPolicy.KEEP_LAST,
            "liveliness": LivelinessPolicy.MANUAL_BY_TOPIC,
            "weight": 1,
        },
        {
            "durability": DurabilityPolicy.TRANSIENT_LOCAL,
            "history": HistoryPolicy.KEEP_ALL,
            "liveliness": LivelinessPolicy.AUTOMATIC,
            "weight": 1,
        },
        {
            "durability": DurabilityPolicy.TRANSIENT_LOCAL,
            "history": HistoryPolicy.KEEP_ALL,
            "liveliness": LivelinessPolicy.MANUAL_BY_TOPIC,
            "weight": 1,
        },
    ]

    def __init__(self) -> None:
        self._initialize_combinations()

    def _initialize_combinations(self) -> None:
        """
        Convert QOS_SETTINGS into a list of (profile, weight).
        """
        self.combinations = []
        for cfg in self.QOS_SETTINGS:
            profile = QoSProfile(
                depth=10,
                durability=cfg["durability"],
                history=cfg["history"],
                liveliness=cfg["liveliness"],
            )
            self.combinations.append({
                "profile": profile,
                "weight": cfg["weight"],
            })

    def update_qos(self) -> None:
        """
        Randomly pick one QoSProfile (by weight) and store it as self.qos.
        """
        weights = [c["weight"] for c in self.combinations]
        choice = random.choices(self.combinations, weights=weights, k=1)[0]
        self.qos = choice["profile"]

    def get_qos(self) -> QoSProfile:
        """
        Return the last selected QoSProfile, or raise if none selected yet.
        """
        if not hasattr(self, 'qos'):
            raise RuntimeError("QoS not yet selected: call update_qos() first")
        return self.qos