#!/usr/bin/env python3
import json, uuid
import rclpy
from rclpy.node import Node
from tsyp_simulation.msg import NetworkPacket, BeaconState
from tsyp_simulation.common import load_config, packet_base, rx_topic

class OutsideNetworkGateway(Node):
    """The Outside Network Area is this node itself.

    It is the sole bridge between the danger zone and the outside entities.
    While the Writer is inside, event knowledge reaches this node through Beacon #1.
    When the Writer physically returns to the entrance, a direct return link is used.
    """
    def __init__(self):
        super().__init__('gateway_node'); self.id='GATEWAY'; self.cfg=load_config()
        self.tx=self.create_publisher(NetworkPacket,'/network/tx',100)
        self.rx=self.create_subscription(NetworkPacket,rx_topic('GATEWAY'),self.on_packet,100)
        self.states={}
        self.seen_packets=set()
        self.seen_events=set()
        self.pending_events=[]
        self.active_event=None
        self.states_sub=self.create_subscription(BeaconState,'/network/beacon_state',self.on_state,100)
        self.get_logger().info('OUTSIDE NETWORK AREA = GATEWAY | waiting for Beacon #1 / Writer return')

    def on_state(self,m):
        if m.deployed: self.states[m.beacon_number]=m

    def send(self,typ,dest,payload,creator='GATEWAY'):
        self.tx.publish(packet_base(str(uuid.uuid4()),typ,creator,'GATEWAY',dest,payload,self.get_clock().now().nanoseconds/1e9))

    def mission_from_memory(self,payload):
        event=payload
        coords=event.get('coordinates_local',{})
        # Navigation is beacon-derived: the executor is briefed with the deployed breadcrumb chain.
        deployed=sorted([n for n in self.states
                         if self.states[n].deployed and
                         self.states[n].navigation_role != 'EVENT_BEACON'])
        chain=[{'beacon_number':n,'x':self.states[n].x,'y':self.states[n].y} for n in deployed]
        return {'alert_type':'EVENT_CONFIRMED','event':event,'navigation':{
            'entry_beacon':1,'beacon_route':[n for n in deployed],'deployed_beacons':chain,
            'target_event':{'x':coords.get('x'), 'y':coords.get('y')},
            'instruction':'Enter at Beacon #1 and follow deployed beacon breadcrumbs toward the event.'},
            'gateway_id':self.id,'source':'BEACON-1'}

    def dispatch_event(self,payload):
        if self.active_event is not None:
            self.pending_events.append(payload)
            self.get_logger().info(
                f'Gateway queued {payload.get("event_type")}; '
                'Executor is servicing the previous event')
            return
        brief=self.mission_from_memory(payload)
        self.active_event=payload.get('event_id')
        # Both outside destinations talk ONLY to the Gateway. They never address each other.
        self.send('LORA_COMMAND_ALERT','COMMAND_POST',brief,creator=payload.get('creator_id',self.id))
        self.send('LORA_EXECUTOR_MISSION','EXECUTOR-01',brief,creator=payload.get('creator_id',self.id))
        self.get_logger().info(f'Gateway dispatch: {payload.get("event_type")} -> Command Post + Executor via LoRa')

    def on_packet(self,p):
        if p.packet_id in self.seen_packets:
            return
        self.seen_packets.add(p.packet_id)
        try: payload=json.loads(p.payload_json)
        except Exception: payload={}
        if p.message_type=='EVENT_DISCOVERY':
            event_id=payload.get('event_id')
            if event_id in self.seen_events:
                return
            self.seen_events.add(event_id)
            self.get_logger().info(f'Gateway received EVENT_DISCOVERY through Beacon #1: {event_id}')
            self.dispatch_event(payload)
        elif p.message_type=='WRITER_RETURN':
            self.get_logger().info('Writer returned to entrance: direct Writer <-> Gateway synchronization active')
            self.send('GATEWAY_ACK','WRITER-01',{'ack':'WRITER_RETURN_RECEIVED','gateway_id':self.id,'known_events':payload.get('known_events',[])})
        elif p.message_type=='EXECUTOR_REPORT':
            self.send('LORA_EXECUTOR_REPORT','COMMAND_POST',payload,creator=p.creator_id)
            self.get_logger().info('Executor report forwarded to Command Post through Gateway only')
            if payload.get('event_id') == self.active_event:
                self.active_event=None
                if self.pending_events:
                    self.dispatch_event(self.pending_events.pop(0))

def main():
    rclpy.init(); n=OutsideNetworkGateway(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
