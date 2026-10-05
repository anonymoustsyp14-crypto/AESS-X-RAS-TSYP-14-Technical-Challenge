#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from tsyp_simulation.msg import NetworkPacket, BeaconState
from tsyp_simulation.common import load_config, rx_topic

class BeaconManager(Node):
    """Models the persistent memory of each deployed physical beacon."""
    def __init__(self):
        super().__init__('beacon_manager_node')
        self.cfg=load_config(); self.states={}
        self.pub=self.create_publisher(BeaconState,'/network/beacon_state',100)
        self.state_sub=self.create_subscription(BeaconState,'/network/beacon_state',self.on_state,100)
        self.subs=[]
        for b in self.cfg['writer']['beacons']:
            n=b['number']; self.states[n]={'x':b['x'],'y':b['y'],'events':[]}
            self.subs.append(self.create_subscription(NetworkPacket,rx_topic(f'BEACON-{n}'),lambda m,n=n:self.on_packet(n,m),50))
        self.get_logger().info('Hardcoded beacon route armed; #1 is the entrance gateway beacon')

    def on_state(self, msg):
        if msg.deployed and msg.beacon_number in self.states:
            self.states[msg.beacon_number]['x']=float(msg.x)
            self.states[msg.beacon_number]['y']=float(msg.y)
            try:
                data=json.loads(msg.state_json)
                self.states[msg.beacon_number].update(data)
            except (TypeError, ValueError):
                pass

    def deploy(self,n):
        b=next(x for x in self.cfg['writer']['beacons'] if x['number']==n)
        msg=BeaconState(); msg.beacon_id=f'BEACON-{n}'; msg.beacon_number=n; msg.x=b['x']; msg.y=b['y']; msg.z=.16; msg.deployed=True
        msg.previous_beacon_id=f'BEACON-{n+1}' if n<12 else ''; msg.next_beacon_id=f'BEACON-{n-1}' if n>1 else 'GATEWAY'
        msg.event_ids_json=json.dumps(self.states[n]['events']); msg.navigation_role='ENTRANCE' if n==1 else 'NAVIGATION_BREADCRUMB'
        msg.state_json=json.dumps(self.states[n],sort_keys=True); self.pub.publish(msg)

    def on_packet(self,n,p):
        try: payload=json.loads(p.payload_json)
        except Exception: payload={}
        eid=payload.get('event_id')
        if eid and eid not in self.states[n]['events']: self.states[n]['events'].append(eid)
        msg=BeaconState(); msg.beacon_id=f'BEACON-{n}'; msg.beacon_number=n
        msg.x=self.states[n]['x']
        msg.y=self.states[n]['y']; msg.z=.08; msg.deployed=True
        msg.previous_beacon_id=f'BEACON-{n+1}' if n<12 else ''
        msg.next_beacon_id=f'BEACON-{n-1}' if n>1 else 'GATEWAY'
        msg.event_ids_json=json.dumps(self.states[n]['events'])
        msg.navigation_role='EVENT_BEACON' if self.states[n]['events'] else ('ENTRANCE' if n==1 else 'NAVIGATION_BEACON')
        msg.state_json=json.dumps(self.states[n],sort_keys=True)
        self.pub.publish(msg)
        self.get_logger().info(f'Beacon #{n} stored memory: event={eid or "none"}; forwarded toward #1')

def main():
    rclpy.init(); n=BeaconManager(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
