#!/usr/bin/env python3
import json, math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def load_config():
    config_path = Path(get_package_share_directory('tsyp_simulation')) / 'config' / 'scenario.yaml'
    with open(config_path, 'r', encoding='utf-8') as f:
        import yaml
        return yaml.safe_load(f)

def dist(a, b):
    return math.hypot(float(a[0])-float(b[0]), float(a[1])-float(b[1]))

def clamp(v, lo, hi): return max(lo, min(hi, v))
def rx_topic(node_id): return f'/network/rx/{node_id.replace("-", "_")}'
def dumps(payload): return json.dumps(payload, separators=(',', ':'), sort_keys=True)
def rssi(distance, ref, path_loss): return ref - path_loss * max(0.0, distance)

def packet_base(packet_id, message_type, creator_id, source_id, destination_id, payload, tx_time, protocol=''):
    from tsyp_simulation.msg import NetworkPacket
    p = NetworkPacket()
    p.packet_id=packet_id; p.message_type=message_type; p.creator_id=creator_id
    p.source_id=source_id; p.destination_id=destination_id; p.payload_json=dumps(payload)
    p.tx_time=tx_time; p.ttl_s=120.0; p.hop_count=0; p.protocol=protocol
    return p
