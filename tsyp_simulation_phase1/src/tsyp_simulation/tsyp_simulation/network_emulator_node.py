#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from tsyp_simulation.msg import NetworkPacket, NetworkHop, BeaconState
from tsyp_simulation.common import load_config, dist, rssi, clamp, rx_topic

class NetworkEmulator(Node):
    """Embedded wireless emulator.

    Topology is intentionally asymmetric by role:
      * inside: Writer -> local beacon -> ... -> Beacon #1 -> Gateway (ESP-NOW)
      * outside: Gateway <-> Command Post / Executor (LoRa)
      * return: Writer <-> Gateway is only allowed for WRITER_RETURN/ACK messages.

    The Outside Network Area is therefore represented by the GATEWAY node itself.
    """
    def __init__(self):
        super().__init__('network_emulator_node')
        cfg=load_config(); self.cfg=cfg['network']
        w=cfg['writer']; world=cfg['world']
        self.nodes={'WRITER-01':tuple(w['start']), 'GATEWAY':tuple(world['entrance_gateway'][:2]),
                    'COMMAND_POST':tuple(world['command_post'][:2]), 'EXECUTOR-01':tuple(world['executor_start'][:2])}
        self.beacons={}
        self.active_beacons=set()
        for beacon in w['beacons']:
            beacon_id=f"BEACON-{beacon['number']}"
            self.beacons[beacon['number']] = (beacon_id, (beacon['x'], beacon['y']))
            self.nodes[beacon_id] = (beacon['x'], beacon['y'])
            if beacon['number'] == 1:
                self.active_beacons.add(1)
        self.rng=random.Random(42)
        self.hop_pub=self.create_publisher(NetworkHop,'/network/hops',200)
        self.state_pub=self.create_publisher(BeaconState,'/network/beacon_state',100)
        self.sub=self.create_subscription(NetworkPacket,'/network/tx',self.on_packet,200)
        self.state_sub=self.create_subscription(BeaconState,'/network/beacon_state',self.on_beacon_state,100)
        self.position_sub=self.create_subscription(Point,'/network/writer_position',self.on_writer_position,20)
        self.rx_names=['GATEWAY','WRITER-01','COMMAND_POST','EXECUTOR-01']+[f'BEACON-{n}' for n in range(1,13)]
        self.out_pubs={name:self.create_publisher(NetworkPacket,rx_topic(name),50) for name in self.rx_names}
        self.get_logger().info('Network emulator: Outside Network Area = GATEWAY; ESP-NOW beacon chain + LoRa outside links')

    def on_beacon_state(self,msg):
        if not msg.deployed: return
        self.beacons[msg.beacon_number]=(msg.beacon_id,(msg.x,msg.y))
        self.nodes[msg.beacon_id]=(msg.x,msg.y)
        self.active_beacons.add(msg.beacon_number)

    def on_writer_position(self,msg):
        self.nodes['WRITER-01']=(float(msg.x), float(msg.y))

    def beacon_chain_from(self,n):
        return [f'BEACON-{i}' for i in range(int(n),0,-1)
                if i in self.active_beacons]

    def route(self,p):
        s,d=p.source_id,p.destination_id
        if s=='WRITER-01' and d.startswith('BEACON-'):
            n=int(d.split('-')[1]); chain=self.beacon_chain_from(n)
            return ['WRITER-01']+chain+['GATEWAY']
        # A gateway message is never routed through a beacon.
        if s=='GATEWAY' and d in ('COMMAND_POST','EXECUTOR-01'):
            return ['GATEWAY',d]
        if s=='EXECUTOR-01' and d=='GATEWAY':
            return ['EXECUTOR-01','GATEWAY']
        # Only the Writer's explicit return packet may use the direct return link.
        if s=='WRITER-01' and d=='GATEWAY' and p.message_type in ('WRITER_RETURN','GATEWAY_ACK'):
            return ['WRITER-01','GATEWAY']
        if s=='GATEWAY' and d=='WRITER-01' and p.message_type=='GATEWAY_ACK':
            return ['GATEWAY','WRITER-01']
        return [s,d]

    def link_profile(self,protocol):
        if protocol=='ESP-NOW': return self.cfg['esp_now']
        if protocol=='RETURN-LINK': return self.cfg['return_link']
        return self.cfg['lora']

    def on_packet(self,p):
        route=self.route(p)
        if len(route)<2: return
        if p.message_type in ('WRITER_RETURN','GATEWAY_ACK') and 'BEACON' not in route:
            protocol='RETURN-LINK'
        elif p.destination_id in ('COMMAND_POST','EXECUTOR-01') or p.source_id in ('GATEWAY','EXECUTOR-01'):
            protocol='LoRa'
        else:
            protocol='ESP-NOW'
        profile=self.link_profile(protocol)
        for i,(a,b) in enumerate(zip(route,route[1:]),start=1):
            if a not in self.nodes or b not in self.nodes:
                self.emit(p,protocol,a,b,i,len(route)-1,0,-999,0,1,False,'UNKNOWN_NODE'); return
            d=dist(self.nodes[a],self.nodes[b])
            if d>profile['max_range_m']:
                self.emit(p,protocol,a,b,i,len(route)-1,d,-999,0,1,False,'OUT_OF_RANGE'); return
            rr=rssi(d,profile['reference_rssi_dbm'],profile['path_loss_db_per_m'])
            drop=clamp(profile['base_drop_probability']+max(0.0,(-rr-70.0))/500.0,0.0,0.95)
            latency=max(1.0,profile['base_latency_ms']+self.rng.uniform(-profile['jitter_ms'],profile['jitter_ms']))
            ok=self.rng.random()>=drop
            self.emit(p,protocol,a,b,i,len(route)-1,d,rr,latency,drop,ok,'DELIVERED' if ok else 'RANDOM_DROP')
            if not ok: return
            # Each beacon is an actual receiving/storing node, not merely a visual hop.
            if b.startswith('BEACON-') and b == p.destination_id and b in self.out_pubs:
                self.out_pubs[b].publish(p)
        p.hop_count=len(route)-1
        final_destination = route[-1]
        if final_destination in self.out_pubs:
            self.out_pubs[final_destination].publish(p)

    def emit(self,p,protocol,a,b,idx,total,d,rr,latency,drop,delivered,reason):
        h=NetworkHop(); h.packet_id=p.packet_id; h.protocol=protocol; h.source_id=a; h.destination_id=b
        h.hop_index=idx; h.hop_total=total
        h.distance_m=float(d); h.rssi_dbm=float(rr); h.latency_ms=float(latency)
        h.drop_probability=float(drop); h.delivered=delivered; h.reason=reason; h.payload_json=p.payload_json
        self.hop_pub.publish(h)
        self.get_logger().info(f'[{protocol} Hop {idx}/{total}] {a} -> {b} | RSSI {rr:.1f} dBm | {latency:.0f} ms | {reason}')

def main():
    rclpy.init(); n=NetworkEmulator(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
