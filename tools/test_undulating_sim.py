#!/usr/bin/env python3
"""Audit a generated arena through sensed maps and synchronized control ticks."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from sensor_msgs.msg import PointCloud2
from interfaces.msg import ChassisCommand, RegionStatus


def stamp(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def run(args):
    geometry=json.loads(args.geometry.read_text())
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(); node=Node('undulating_sim_audit')
    state={'odom':None,'clouds':0,'labels7':0,'outside_labels7':0,'paths':0,'clock':None}
    regions={}; commands={}; positions=[]; maps=[]
    def odom(m):
        state['odom']=m
        p=m.pose.pose.position;v=m.twist.twist.linear
        positions.append([stamp(m)*1e-9,p.x,p.y,p.z,math.hypot(v.x,v.y)])
    def terrain(m):
        a=np.asarray(m.data).reshape(m.info.height,m.info.width)
        ys,xs=np.where(a==7)
        wx=m.info.origin.position.x+(xs+.5)*m.info.resolution
        wy=m.info.origin.position.y+(ys+.5)*m.info.resolution
        c,s=math.cos(geometry['angle_rad']),math.sin(geometry['angle_rad'])
        inside=(c*wx+s*wy>=0)&(c*wx+s*wy<=geometry['length_m'])&(np.abs(-s*wx+c*wy)<=geometry['width_m']/2)
        count=int(inside.sum())
        state['outside_labels7']=max(state['outside_labels7'],len(xs)-count)
        state['labels7']=max(count,state['labels7'])
        maps.append({'stamp':stamp(m)*1e-9,'count7':count})
    node.create_subscription(Odometry,'/Odometry',odom,qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid,'/rog_map/terrain_label',terrain,qos_profile_sensor_data)
    node.create_subscription(PointCloud2,'/cloud_registered_full',lambda m:state.update(clouds=state['clouds']+1),qos_profile_sensor_data)
    node.create_subscription(NavPath,'/nav_executor/minco_path',lambda m:state.update(paths=state['paths']+bool(m.poses)),10)
    node.create_subscription(RegionStatus,'/nav_executor/region_status',lambda m:regions.update({stamp(m):m}),10)
    node.create_subscription(ChassisCommand,'/nav_executor/chassis_cmd',lambda m:commands.update({stamp(m):m}),10)
    pub=node.create_publisher(PoseStamped,'/goal_pose',10)
    started=time.monotonic(); error=None; arrived=False; settled=None
    try:
        ready=time.monotonic()+45
        while time.monotonic()<ready and (state['odom'] is None or state['clouds']<15):
            rclpy.spin_once(node,timeout_sec=.05)
        if state['odom'] is None or state['clouds']<15: raise RuntimeError('No odometry/cloud readiness')
        mapped_at=stamp(state['odom'])*1e-9
        mapping_deadline=time.monotonic()+max(20,args.mapping_seconds*10)
        while stamp(state['odom'])*1e-9-mapped_at<args.mapping_seconds:
            if time.monotonic()>mapping_deadline:raise RuntimeError('Simulation clock stalled during mapping')
            rclpy.spin_once(node,timeout_sec=.02)
        goal=PoseStamped();goal.header.frame_id='odom';goal.pose.orientation.w=1.
        if args.approach is not None:
            goal.pose.position.x=args.approach*math.cos(geometry['angle_rad'])
            goal.pose.position.y=args.approach*math.sin(geometry['angle_rad'])
            pub.publish(goal)
            end=time.monotonic()+60; stable=None
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.02)
                p=state['odom'].pose.pose.position;v=state['odom'].twist.twist.linear
                if geometry.get('closed_lane') and -.5<p.x<geometry['length_m']+.5 and abs(p.y)>geometry['width_m']/2-.302:
                    raise RuntimeError('Left the straight driving corridor during approach')
                close=math.hypot(p.x-goal.pose.position.x,p.y-goal.pose.position.y)<.30 and math.hypot(v.x,v.y)<.15
                t=stamp(state['odom'])*1e-9
                if close:
                    if stable is None:stable=t
                    if t-stable>=4:break
                else:stable=None
            else:raise RuntimeError('Approach observation waypoint was not reached')
        goal.pose.position.x,goal.pose.position.y=geometry['goal'];pub.publish(goal)
        deadline=time.monotonic()+args.timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            p=state['odom'].pose.pose.position;v=state['odom'].twist.twist.linear
            longitudinal=math.cos(geometry['angle_rad'])*p.x+math.sin(geometry['angle_rad'])*p.y
            lateral=-math.sin(geometry['angle_rad'])*p.x+math.cos(geometry['angle_rad'])*p.y
            if geometry.get('closed_lane') and -.5<longitudinal<geometry['length_m']+.5 and abs(lateral)>geometry['width_m']/2-.302:
                raise RuntimeError('Left the wave driving corridor; bypass is forbidden')
            close=math.hypot(p.x-geometry['goal'][0],p.y-geometry['goal'][1])<.30 and math.hypot(v.x,v.y)<.15
            t=stamp(state['odom'])*1e-9
            if close:
                if settled is None:settled=t
                if t-settled>=.5:arrived=True;break
            else:settled=None
        if not arrived: raise RuntimeError('Timed out without a valid wave traversal')
    except Exception as exc:
        error=str(exc)
        if state['odom'] is not None:
            hold=PoseStamped();hold.header.frame_id='odom';hold.pose=state['odom'].pose.pose
            pub.publish(hold)
            until=time.monotonic()+.3
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
    paired=[(regions[t],commands[t]) for t in regions.keys()&commands.keys()]
    follow=[(r,c) for r,c in paired if r.terrain_label==7 and c.follow_gimbal]
    unsafe=[(r,c) for r,c in follow if not r.gimbal_heading_ready and math.hypot(c.vx,c.vy)>.02]
    ready_moves=[(r,c) for r,c in follow if r.gimbal_heading_ready and c.mode==7 and math.hypot(c.vx,c.vy)>.02]
    c,s=math.cos(geometry['angle_rad']),math.sin(geometry['angle_rad'])
    ready_on_road=0
    if positions:
        pose=np.asarray(positions)
        for r,command in ready_moves:
            t=stamp(r)*1e-9
            px=np.interp(t,pose[:,0],pose[:,1]);py=np.interp(t,pose[:,0],pose[:,2])
            if 0<=c*px+s*py<=geometry['length_m'] and abs(-s*px+c*py)<geometry['width_m']/2-.302:
                ready_on_road+=1
    wave_bins={int((c*p[1]+s*p[2])/geometry['wavelength_m']) for p in positions
               if 0<=c*p[1]+s*p[2]<geometry['length_m'] and
               abs(-s*p[1]+c*p[2])<geometry['width_m']/2-.302}
    result={'arrived':arrived,'error':error,'clouds':state['clouds'],'max_label7_cells':state['labels7'],
            'mapping_seconds':args.mapping_seconds,
            'max_outside_label7_cells':state['outside_labels7'],
            'paths':state['paths'],'paired_ticks':len(paired),'undulating_follow_ticks':len(follow),
            'ready_mode7_moving_ticks':len(ready_moves),'unready_translation_ticks':len(unsafe),
            'ready_mode7_over_wave_ticks':ready_on_road,
            'traversed_wave_bins':len(wave_bins),
            'wall_seconds':time.monotonic()-started,
            'max_z':max((p[3] for p in positions),default=None),
            'passed':bool(arrived and state['labels7'] and ready_on_road>=10 and len(wave_bins)>=10 and not unsafe and not error)}
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    (args.output/'positions.json').write_text(json.dumps(positions)+'\n')
    (args.output/'terrain_counts.json').write_text(json.dumps(maps)+'\n')
    (args.output/'control.json').write_text(json.dumps([
        {'stamp':stamp(r)*1e-9,'label':r.terrain_label,'mode':c.mode,'phase':r.phase,
         'ready':r.gimbal_heading_ready,'heading_error_deg':r.gimbal_heading_error_deg,
         'follow':c.follow_gimbal,'vx':c.vx,'vy':c.vy} for r,c in sorted(paired,key=lambda p:stamp(p[0]))])+'\n')
    print(json.dumps(result),flush=True)
    node.destroy_node();rclpy.shutdown()
    return result['passed']


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=120)
    parser.add_argument('--mapping-seconds',type=float,default=2,help='simulated mapping time before sending the first goal')
    parser.add_argument('--approach',type=float,help='optional road-axis observation waypoint before the final goal')
    args=parser.parse_args()
    raise SystemExit(0 if run(args) else 1)
