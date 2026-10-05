#!/usr/bin/env python3
import json, math, subprocess, time, uuid
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tsyp_simulation.msg import NetworkPacket
from tsyp_simulation.common import load_config, packet_base, rx_topic

class Executor(Node):
    """Executor receives its mission only from the Gateway and follows beacon breadcrumbs."""
    def __init__(self):
        super().__init__('executor_robot_node'); cfg=load_config(); self.cfg=cfg; self.id=cfg['executor']['id']; self.speed=cfg['executor']['speed_mps']
        self.cmd=self.create_publisher(Twist,'/model/executor_robot/cmd_vel',20); self.tx=self.create_publisher(NetworkPacket,'/network/tx',100)
        self.sub=self.create_subscription(NetworkPacket,rx_topic('EXECUTOR-01'),self.on_mission,100)
        self.odom_sub=self.create_subscription(Odometry,'/model/executor_robot/odom',self.on_odom,20)
        self.scan_sub=self.create_subscription(LaserScan,'/model/executor_robot/scan',self.on_scan,20)
        self.beacons={b['number']:(b['x'],b['y']) for b in cfg['writer']['beacons']}
        self.entrance=self.beacons[1]; self.start_x,self.start_y=cfg['world']['executor_start'][:2]
        self.origin_x,self.origin_y=self.start_x,self.start_y; self.x,self.y=self.origin_x,self.origin_y
        self.odom_ref_x=0.0; self.odom_ref_y=0.0; self.heading_ref=0.0
        self.heading=0.0; self.odom_calibrated=False; self.goals=[]; self.current_event=None; self.last=time.monotonic(); self.last_progress_log=self.last; self.reported=False
        self.front_clearance=3.0; self.left_clearance=3.0; self.right_clearance=3.0
        self.seen_missions=set()
        self.completed_events=set()
        self.timer=self.create_timer(.1,self.tick); self.get_logger().info('Executor waiting OUTSIDE; mission can arrive only from Gateway')

    def on_odom(self, msg):
        odom_x=float(msg.pose.pose.position.x); odom_y=float(msg.pose.pose.position.y)
        q=msg.pose.pose.orientation
        yaw=math.atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z))
        if not self.odom_calibrated:
            self.odom_ref_x=odom_x; self.odom_ref_y=odom_y
            self.heading_ref=yaw
            self.odom_calibrated=True
        self.x=self.start_x+(odom_x-self.odom_ref_x)
        self.y=self.start_y+(odom_y-self.odom_ref_y)
        self.heading=math.atan2(math.sin(yaw-self.heading_ref), math.cos(yaw-self.heading_ref))

    def on_mission(self,p):
        if p.message_type!='LORA_EXECUTOR_MISSION': return
        if p.packet_id in self.seen_missions: return
        self.seen_missions.add(p.packet_id)
        try: brief=json.loads(p.payload_json)
        except Exception: return
        nav=brief.get('navigation',{}); route=nav.get('deployed_beacons',[]); event=nav.get('target_event',{})
        event_id=brief.get('event',{}).get('event_id') or brief.get('event_id')
        if event_id in self.completed_events:
            return
        if self.current_event and self.current_event.get('event_id') == event_id and self.goals:
            return
        # Use the actual deployed breadcrumb order. Do not communicate with Command Post.
        stops=[(float(b['x']),float(b['y']),f'Beacon #{b["beacon_number"]}') for b in sorted(route,key=lambda z:z['beacon_number'])]
        stops += [(float(event['x']),float(event['y']),'EVENT'), (self.entrance[0],self.entrance[1],'ENTRANCE')]
        self.goals=[]
        previous=(self.x, self.y)
        for stop in stops:
            self.goals.extend(self.safe_segment(previous, stop))
            previous=stop[:2]
        self.current_event=brief.get('event',{}); self.reported=False
        self.get_logger().info(f'Gateway mission received: {self.current_event.get("event_type")} | route Beacon #1 -> #2 -> ... -> event')

    def on_scan(self, msg):
        ranges=list(msg.ranges)
        if not ranges:
            return
        center=len(ranges)//2
        window=max(2, len(ranges)//36)
        def clearance(start,end):
            values=[r for r in ranges[max(0,start):min(len(ranges),end)]
                    if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
            return sorted(values)[len(values)//2] if values else msg.range_max
        self.front_clearance=clearance(center-window,center+window+1)
        self.left_clearance=clearance(center+2*window,center+8*window+1)
        self.right_clearance=clearance(center-8*window,center-2*window+1)

    @staticmethod
    def safe_segment(start, stop):
        """Route through the compact building's entrance and partition openings."""
        sx, sy = start
        tx, ty, label = stop
        points=[]
        if sx > 7.5 and tx < 6.7:
            points.append((7.8, 0.0, 'outside corridor'))
            points.append((5.8, 0.0, 'entrance corridor'))
            sx, sy = 5.8, 0.0
        if sy < 0.9 <= ty and (sx < 1.0 or tx < 1.0):
            points.append((1.8, sy, 'partition opening'))
            points.append((1.8, ty, 'partition opening'))
        elif sy < 0.9 <= ty and tx < 3.2:
            points.append((2.5, sy, 'upper-room opening'))
            points.append((2.5, ty, 'upper-room opening'))
        elif sy > 1.8 >= ty and (sx < 1.0 or tx < 1.0):
            points.append((1.8, sy, 'partition opening'))
            points.append((1.8, ty, 'partition opening'))
        if sy < -1.0 <= ty and (sx < 5.0 or tx < 5.0):
            points.append((5.8, sy, 'lower corridor'))
            points.append((5.8, ty, 'lower corridor'))
        elif sy > -0.5 >= ty and (sx < 5.0 or tx < 5.0):
            points.append((5.8, sy, 'lower corridor'))
            points.append((5.8, ty, 'lower corridor'))
        points.append((tx, ty, label))
        return points

    def move(self,target,dt):
        dx,dy=target[0]-self.x,target[1]-self.y; d=math.hypot(dx,dy)
        if d<.16: return True
        yaw=math.atan2(dy,dx); err=math.atan2(math.sin(yaw-self.heading),math.cos(yaw-self.heading))
        t=Twist()
        # The executor follows the prevalidated demonstration route. It does
        # not re-plan from sensors; the route itself is selected around walls.
        if abs(err) > 0.28:
            t.linear.x=0.0
            t.angular.z=max(-.9,min(.9,err*1.8))
        else:
            t.linear.x=self.speed
            t.angular.z=max(-.35,min(.35,err))
        self.cmd.publish(t); return False

    def service_event(self):
        event_type=self.current_event.get('event_type')
        x=float(self.current_event.get('coordinates_local',{}).get('x',
                self.current_event.get('x',0.0)))
        y=float(self.current_event.get('coordinates_local',{}).get('y',
                self.current_event.get('y',0.0)))
        if event_type == 'FIRE':
            command=['ros2','run','ros_gz_sim','delete','-entity','fire_hazard']
            action='removed fire_hazard'
        elif event_type == 'VICTIM':
            sdf=f'''<sdf version="1.9"><model name="first_aid_kit_{self.current_event.get("event_id")}"><static>true</static><pose>{x} {y} 0.18 0 0 0</pose><link name="link"><visual name="kit"><geometry><box><size>0.30 0.20 0.18</size></box></geometry><material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual></link></model></sdf>'''
            command=['ros2','run','ros_gz_sim','create','-string',sdf,
                     '-name',f'first_aid_kit_{self.current_event.get("event_id")}']
            action='dropped first-aid kit'
        else:
            return
        try:
            result=subprocess.run(command,capture_output=True,text=True,timeout=5)
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Executor service action timed out for {event_type}')
            return
        if result.returncode != 0:
            self.get_logger().error(f'Executor service action failed for {event_type}: {result.stderr.strip()}')
            return
        self.get_logger().info(f'Executor serviced {event_type}: {action}')

    def tick(self):
        dt=min(.2,time.monotonic()-self.last); self.last=time.monotonic()
        if not self.goals: self.cmd.publish(Twist()); return
        if time.monotonic()-self.last_progress_log > 5.0:
            self.get_logger().info(
                f'EXECUTOR route progress: {len(self.goals)} stops remaining '
                f'at ({self.x:.2f},{self.y:.2f})')
            self.last_progress_log=time.monotonic()
        target=self.goals[0]
        if self.move(target[:2],dt):
            reached=self.goals.pop(0); self.get_logger().info(f'Executor reached {reached[2]} at ({reached[0]:.1f},{reached[1]:.1f})')
            if reached[2] == 'EVENT':
                self.service_event()
            if not self.goals and not self.reported:
                self.cmd.publish(Twist()); payload={'report_type':'MISSION_EXECUTED','executor_id':self.id,'status':'SERVICED_AND_RETURNED','event_id':self.current_event.get('event_id'),'timestamp':self.get_clock().now().nanoseconds/1e9}
                self.tx.publish(packet_base(str(uuid.uuid4()),'EXECUTOR_REPORT',self.id,self.id,'GATEWAY',payload,payload['timestamp'],'LoRa')); self.reported=True
                self.completed_events.add(self.current_event.get('event_id'))
                self.get_logger().info('Executor report sent to Gateway; Gateway alone forwards it to Command Post')

def main():
    rclpy.init(); n=Executor(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
