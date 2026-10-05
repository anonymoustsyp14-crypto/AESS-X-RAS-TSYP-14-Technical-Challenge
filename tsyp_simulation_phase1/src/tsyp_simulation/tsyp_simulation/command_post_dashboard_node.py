#!/usr/bin/env python3
import json, os
import rclpy
from rclpy.node import Node
from tsyp_simulation.msg import NetworkHop, NetworkPacket
from tsyp_simulation.common import rx_topic

class Dashboard(Node):
    """Human-in-the-loop endpoint. It is downstream of the Gateway only."""
    def __init__(self):
        super().__init__('command_post_dashboard_node')
        self.create_subscription(NetworkHop,'/network/hops',self.hop,200)
        self.create_subscription(NetworkPacket,rx_topic('COMMAND_POST'),self.alert,100)
        self.lines=[]; self.events=[]; self.create_timer(1.0,self.render)

    def add(self,line): self.lines=(self.lines+[line])[-16:]
    def hop(self,m):
        self.add(f'[{m.protocol} Hop {m.hop_index}/{m.hop_total}] {m.source_id} -> {m.destination_id} | RSSI {m.rssi_dbm:.1f} dBm | {m.latency_ms:.0f} ms | {m.reason}')

    def alert(self,p):
        try: b=json.loads(p.payload_json)
        except Exception: b={}
        e=b.get('event',{})
        if p.message_type=='LORA_COMMAND_ALERT':
            self.events.append(e.get('event_id','?'))
            self.add(f'ALERT VIA GATEWAY / LoRa | {e.get("event_id")} | {e.get("event_type")} | stored={e.get("stored_at_beacon")}')
            self.add('PAYLOAD: '+json.dumps(e,sort_keys=True))
            self.add('NAV: '+json.dumps(b.get('navigation',{}),sort_keys=True))
        elif p.message_type=='LORA_EXECUTOR_REPORT':
            self.add('EXECUTOR REPORT VIA GATEWAY / LoRa: '+json.dumps(b,sort_keys=True))

    def render(self):
        os.system('clear')
        print('='*118); print(' LIVING MAP — COMMAND POST '.center(118)); print('='*118)
        print('OUTSIDE NETWORK AREA = GATEWAY | Command Post has NO direct link to Writer or Executor')
        print('INSIDE: Writer -> local beacon -> ... -> Beacon #1 -> Gateway | OUTSIDE: Gateway -> LoRa -> endpoints')
        print('-'*118)
        for line in self.lines: print(line)
        print('-'*118); print('Detected/validated events:', ', '.join(self.events) or 'none')
        print('Human-in-the-loop: inspect evidence here; any dispatch/return path remains Gateway-mediated.')

def main():
    rclpy.init(); n=Dashboard(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
