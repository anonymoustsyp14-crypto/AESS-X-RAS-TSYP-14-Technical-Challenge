#!/usr/bin/env python3
import math, subprocess, time, uuid, json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tsyp_simulation.msg import NetworkPacket, BeaconState
from tsyp_simulation.common import load_config, packet_base, dist

class WriterRobot(Node):
    """Writer: drop #1 at the entrance, then leave a breadcrumb chain #2..#5.

    Events are written to the deepest beacon currently available. The beacon forwards
    that memory toward #1, and #1 is the only inside node connected to the Gateway.
    The Writer never contacts the Gateway while inside. A direct return link is used
    only after the Writer physically comes back to the entrance.
    """
    def __init__(self):
        super().__init__('writer_robot_node'); cfg=load_config(); w=cfg['writer']
        self.cfg=cfg; self.id=w['id']; self.speed=w['speed_mps']; self.beacons={b['number']:b for b in w['beacons']}
        self.events=cfg['events']; self.cmd=self.create_publisher(Twist,'/model/writer_robot/cmd_vel',20)
        self.tx=self.create_publisher(NetworkPacket,'/network/tx',100); self.state_pub=self.create_publisher(BeaconState,'/network/beacon_state',100)
        self.position_pub=self.create_publisher(Point,'/network/writer_position',10)
        self.start_x,self.start_y=w['start']; self.origin_x,self.origin_y=self.start_x,self.start_y
        self.x,self.y=w['start']; self.heading=0.0; self.odom_ready=False; self.odom_calibrated=False
        self.odom_ref_x=0.0; self.odom_ref_y=0.0; self.heading_ref=0.0
        self.wp_i=0; self.last=time.monotonic(); self.last_progress_log=self.last
        self.return_sent=False; self.done=False
        self.deployed=[]; self.detected=[]; self.front_clearance=3.0
        self.left_clearance=3.0; self.right_clearance=3.0
        self.last_beacon_position=(self.x, self.y); self.next_beacon_number=2
        self.beacon_spacing=float(w.get('beacon_spacing_m', 2.0))
        self.max_beacons=int(w.get('max_beacons', 12))
        self.wps=w['waypoints']; self.drop_map={int(k): int(v) for k, v in w['beacon_drop_after_waypoint'].items()}
        self.odom_sub=self.create_subscription(Odometry, '/model/writer_robot/odom', self.on_odom, 20)
        self.scan_sub=self.create_subscription(LaserScan, '/model/writer_robot/scan', self.on_scan, 20)
        self.timer=self.create_timer(.1,self.tick)
        self.get_logger().info('WRITER waiting for Waffle odometry before dropping Beacon #1')

    def on_odom(self, msg):
        odom_x=float(msg.pose.pose.position.x); odom_y=float(msg.pose.pose.position.y)
        q=msg.pose.pose.orientation
        yaw=math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
        if not self.odom_calibrated:
            self.odom_ref_x=odom_x; self.odom_ref_y=odom_y
            self.heading_ref=yaw
            self.odom_calibrated=True
        self.x=self.start_x+(odom_x-self.odom_ref_x)
        self.y=self.start_y+(odom_y-self.odom_ref_y)
        self.heading=math.atan2(math.sin(yaw-self.heading_ref), math.cos(yaw-self.heading_ref))
        self.odom_ready=True
        self.position_pub.publish(self._position_message())

    def _position_message(self):
        from geometry_msgs.msg import Point
        position=Point(); position.x=self.x; position.y=self.y
        return position

    def on_scan(self, msg):
        ranges = list(msg.ranges)
        if not ranges:
            return
        center = len(ranges) // 2
        window = max(2, len(ranges) // 36)

        def clearance(start, end):
            values = [r for r in ranges[max(0, start):min(len(ranges), end)]
                      if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
            return sorted(values)[len(values) // 2] if values else msg.range_max

        self.front_clearance = clearance(center - window, center + window + 1)
        self.left_clearance = clearance(center + 2 * window, center + 8 * window + 1)
        self.right_clearance = clearance(center - 8 * window, center - 2 * window + 1)

    def drop_beacon(self,n, x=None, y=None, role='NAVIGATION_BEACON'):
        if n in self.deployed: return
        if n not in self.beacons:
            self.beacons[n]={'number':n, 'x':self.x, 'y':self.y}
        b=self.beacons[n]
        # Place the beacon just behind the chassis so the newly dropped object
        # cannot be mistaken for a front obstacle by the lidar.
        if x is None:
            x=self.x-0.35*math.cos(self.heading)
        if y is None:
            y=self.y-0.35*math.sin(self.heading)
        name=f'beacon_{n}'
        color='0.85 0.08 0.03 1' if role == 'EVENT_BEACON' else '0.04 0.18 0.8 1'
        sdf=f'''<sdf version="1.9"><model name="{name}"><static>true</static><link name="link"><pose>0 0 0.08 0 0 0</pose><visual name="body"><geometry><cylinder><radius>0.08</radius><length>0.16</length></cylinder></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse><emissive>0.01 0.04 0.25 1</emissive></material></visual><visual name="top"><pose>0 0 0.11 0 0 0</pose><geometry><cylinder><radius>0.05</radius><length>0.04</length></cylinder></geometry><material><ambient>0.95 0.55 0.02 1</ambient><diffuse>1 0.65 0.03 1</diffuse><emissive>0.18 0.08 0 1</emissive></material></visual></link></model></sdf>'''
        try:
            proc=subprocess.run(
                ['ros2','run','ros_gz_sim','create','-string',sdf,'-name',name,
                 '-x',str(x),'-y',str(y),'-z','0.0'],
                capture_output=True, text=True, timeout=5)
        except subprocess.TimeoutExpired:
            self.get_logger().warning(f'Beacon #{n} visual spawn timed out; continuing with its network state')
            proc=None
        if proc is not None and proc.returncode != 0:
            self.get_logger().error(f'Beacon #{n} spawn failed: {proc.stderr.strip()}')
            return
        self.deployed.append(n); self.deployed.sort()
        self.beacons[n]['x']=x; self.beacons[n]['y']=y
        m=BeaconState(); m.beacon_id=f'BEACON-{n}'; m.beacon_number=n; m.x=x; m.y=y; m.z=.16; m.deployed=True
        m.previous_beacon_id=f'BEACON-{n+1}' if n < self.max_beacons else ''
        m.next_beacon_id=f'BEACON-{n-1}' if n>1 else 'GATEWAY'
        m.event_ids_json='[]'; m.navigation_role='ENTRANCE' if n==1 else role
        m.state_json=json.dumps({'deployed_at':self.get_clock().now().nanoseconds/1e9,
                                 'purpose':'event' if role == 'EVENT_BEACON' else 'navigation'})
        self.state_pub.publish(m); self.get_logger().info(f'PHYSICAL DROP: Beacon #{n} at ({x:.1f},{y:.1f})')
        self.last_beacon_position=(x, y)

    def write_event(self, beacon_number, event):
        b = self.beacons[beacon_number]
        m=BeaconState(); m.beacon_id=f'BEACON-{beacon_number}'; m.beacon_number=beacon_number
        m.x=b['x']; m.y=b['y']; m.z=.08; m.deployed=True
        m.previous_beacon_id=f'BEACON-{beacon_number+1}' if beacon_number < self.max_beacons else ''
        m.next_beacon_id=f'BEACON-{beacon_number-1}' if beacon_number>1 else 'GATEWAY'
        m.event_ids_json=json.dumps([event['id']]); m.navigation_role='EVENT_BEACON'
        m.state_json=json.dumps({'purpose':'event','event_id':event['id'],'event_type':event['type'],
                                 'sensor':event['sensor'],'value':event['value'],
                                 'coordinates_local':{'x':event['x'],'y':event['y']}})
        self.state_pub.publish(m)

    def drop_event_beacon(self, event):
        event_beacon={'FIRE':6, 'VICTIM':7}.get(event['type'])
        if event_beacon is None:
            return None
        # Drop at the writer's measured pose so the first ESP-NOW hop is
        # guaranteed to be in range; the event coordinates remain in metadata.
        self.drop_beacon(event_beacon, self.x, self.y, role='EVENT_BEACON')
        m=BeaconState(); m.beacon_id=f'BEACON-{event_beacon}'; m.beacon_number=event_beacon
        m.x=self.x; m.y=self.y; m.z=.16; m.deployed=True
        m.previous_beacon_id=f'BEACON-{event_beacon+1}' if event_beacon < self.max_beacons else ''
        m.next_beacon_id='BEACON-1'
        m.event_ids_json=json.dumps([event['id']]); m.navigation_role='EVENT_BEACON'
        m.state_json=json.dumps({'purpose':'event','event_id':event['id'],
                                 'event_type':event['type'],'coordinates_local':
                                 {'x':event['x'],'y':event['y']}})
        self.state_pub.publish(m)
        return event_beacon

    def nearest_available_beacon(self):
        return min(self.deployed, key=lambda n:dist((self.x,self.y),(self.beacons[n]['x'],self.beacons[n]['y'])))

    def detect_events(self):
        for e in self.events:
            if e['id'] in self.detected: continue
            if e['type'] not in ('FIRE', 'VICTIM'):
                continue
            if dist((self.x,self.y),(e['x'],e['y']))<0.65:
                self.detected.append(e['id'])
                n=self.drop_event_beacon(e)
                if n is None:
                    continue
                payload={'event_id':e['id'],'event_type':e['type'],'sensor':e['sensor'],'value':e['value'],
                    'coordinates_local':{'x':e['x'],'y':e['y']},'timestamp':self.get_clock().now().nanoseconds/1e9,
                    'creator_id':self.id,'stored_at_beacon':f'BEACON-{n}',
                    'gps_translation':{'lat':36.8065+e['y']*1e-5,'lon':10.1815+e['x']*1e-5}}
                p=packet_base(str(uuid.uuid4()),'EVENT_DISCOVERY',self.id,self.id,f'BEACON-{n}',payload,payload['timestamp'],'ESP-NOW')
                self.tx.publish(p)
                self.get_logger().info(f'EVENT BEACON DROP: {e["id"]} ({e["type"]}) -> Beacon #{n}; event data forwarded toward #1')

    def drop_next_beacon(self):
        while self.next_beacon_number in (6, 7):
            self.next_beacon_number += 1
        if self.next_beacon_number > 5:
            return
        self.drop_beacon(self.next_beacon_number)
        self.next_beacon_number += 1

    def move(self,target,dt):
        dx,dy=target[0]-self.x,target[1]-self.y; d=math.hypot(dx,dy)
        if d<.28: return True
        # Every configured leg is axis-aligned: rotate to its exact cardinal
        # heading, then drive with zero steering until the coordinate is met.
        if abs(dx) >= abs(dy):
            yaw=0.0 if dx >= 0.0 else math.pi
        else:
            yaw=math.pi/2 if dy >= 0.0 else -math.pi/2
        err=math.atan2(math.sin(yaw-self.heading),math.cos(yaw-self.heading))
        t=Twist()
        # This route is deliberately clearance-checked in the scenario file.
        # Lidar avoidance is not used here: allowing it to override a
        # waypoint creates the wandering behavior this deterministic demo is
        # intended to avoid.
        if abs(err) > 0.12:
            t.linear.x=0.0
            t.angular.z=max(-0.65,min(0.65,2.0*err))
        else:
            t.linear.x=min(self.speed, 0.40*d)
            t.angular.z=0.0
        self.cmd.publish(t); return False

    def tick(self):
        dt=min(.2,time.monotonic()-self.last); self.last=time.monotonic()
        if not self.odom_ready:
            self.cmd.publish(Twist())
            return
        if not self.deployed:
            # The entrance anchor is fixed by the scenario, even if the first
            # odometry sample arrives after Gazebo has settled the chassis.
            self.drop_beacon(1, self.start_x, self.start_y)
            self.next_beacon_number=2
        self.detect_events()
        # The coverage route visits every room and corridor; beacons are placed
        # by distance and event need, rather than by a fixed five-drop schedule.
        # Events are placed directly on this hardcoded route. The writer
        # detects them while passing their route checkpoints; it never leaves
        # the predefined path to search for an event.
        targets=self.wps
        if self.wp_i<len(targets):
            target=targets[self.wp_i]
            if time.monotonic()-self.last_progress_log > 5.0:
                self.get_logger().info(
                    f'WRITER step {self.wp_i + 1}/{len(targets)} '
                    f'at ({self.x:.2f},{self.y:.2f}); '
                    f'front={self.front_clearance:.2f}m')
                self.last_progress_log=time.monotonic()
            if self.move(target,dt):
                self.wp_i+=1
                if dist((self.x,self.y), self.last_beacon_position) >= self.beacon_spacing:
                    self.drop_next_beacon()
        else:
            self.cmd.publish(Twist())
            # After exploring, physically return to #1/entrance and only then contact Gateway.
            entrance=(self.beacons[1]['x'],self.beacons[1]['y'])
            if dist((self.x,self.y),entrance)>.12:
                self.move(entrance,dt); return
            if not self.return_sent:
                payload={'return_status':'WRITER_AT_ENTRANCE','known_events':self.detected,'creator_id':self.id,
                         'timestamp':self.get_clock().now().nanoseconds/1e9,'note':'Direct Writer-Gateway communication is used only after physical return.'}
                self.tx.publish(packet_base(str(uuid.uuid4()),'WRITER_RETURN',self.id,self.id,'GATEWAY',payload,payload['timestamp'],'RETURN-LINK'))
                self.return_sent=True; self.done=True; self.get_logger().info('WRITER RETURNED: direct Writer <-> Outside Network Gateway link now active')

def main():
    rclpy.init(); n=WriterRobot(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
