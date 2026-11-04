#!/usr/bin/env python3
"""Build a local Gazebo test arena without modifying the maintained simulator.

Use the generated simulator_path with tools/run_navigation_sim.sh. Height is
peak-to-trough, not amplitude. Mesh is used for BOTH collision and lidar visual.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET


def build(output, simulator, angle, closed_lane=False):
    output = output.resolve()
    resources = output / 'rmu_gazebo_simulator/resource'
    model = resources / 'models/undulating_test'
    model.mkdir(parents=True, exist_ok=True)
    (resources / 'worlds').mkdir(exist_ok=True)
    (output / 'rmu_gazebo_simulator/config').mkdir(exist_ok=True)
    shutil.copyfile(simulator / 'rmu_gazebo_simulator/config/gz_world.yaml',
                    output / 'rmu_gazebo_simulator/config/gz_world.yaml')
    length, wavelength, height, base, width = 2.88, .24, .07, .04, 1.4
    vertices = []
    def triangle(a, b, c):
        vertices.append((*a, *b, *c))
    # A 5 mm longitudinal mesh resolves each period with 48 segments.
    for i in range(576):
        x0, x1 = i * .005, (i + 1) * .005
        z0 = base + height * .5 * (1 - math.cos(2 * math.pi * x0 / wavelength))
        z1 = base + height * .5 * (1 - math.cos(2 * math.pi * x1 / wavelength))
        a, b, c, d = (x0,-width/2,z0), (x1,-width/2,z1), (x1,width/2,z1), (x0,width/2,z0)
        triangle(a,b,c); triangle(a,c,d)
        for y, reverse in [(-width/2, False),(width/2, True)]:
            p,q,r,s=(x0,y,z0),(x1,y,z1),(x1,y,.01),(x0,y,.01)
            if reverse: p,q,r,s=s,r,q,p
            triangle(p,q,r);triangle(p,r,s)
    for x in [0.,length]:
        a,b,c,d=(x,-width/2,.01),(x,width/2,.01),(x,width/2,base),(x,-width/2,base)
        triangle(a,b,c);triangle(a,c,d)
    triangle((0,-width/2,.01),(length,width/2,.01),(length,-width/2,.01))
    triangle((0,-width/2,.01),(0,width/2,.01),(length,width/2,.01))
    with (model / 'road.stl').open('wb') as stream:
        stream.write(b'240mm wavelength 70mm peak-trough'.ljust(80,b'\0'))
        stream.write(struct.pack('<I',len(vertices)))
        for triangle_vertices in vertices:
            stream.write(struct.pack('<12fH',0,0,0,*triangle_vertices,0))
    sdf = ET.Element('sdf',version='1.9')
    arena = ET.SubElement(sdf,'model',name='undulating_test')
    ET.SubElement(arena,'static').text='true'
    link=ET.SubElement(arena,'link',name='road')
    for kind in ['collision','visual']:
        item=ET.SubElement(link,kind,name=kind)
        mesh=ET.SubElement(ET.SubElement(item,'geometry'),'mesh')
        ET.SubElement(mesh,'uri').text='model://undulating_test/road.stl'
        if kind=='visual':
            material=ET.SubElement(item,'material')
            ET.SubElement(material,'ambient').text='.6 .5 .15 1'
            ET.SubElement(material,'diffuse').text='.6 .5 .15 1'
    rail_y=.74 if closed_lane else .9
    rail_height=1.0 if closed_lane else .4
    for y in [-rail_y,rail_y]:
        rail=ET.SubElement(arena,'link',name='rail_'+str(y))
        ET.SubElement(rail,'pose').text=f'1.44 {y} {base+rail_height/2} 0 0 0'
        for kind in ['collision','visual']:
            box=ET.SubElement(ET.SubElement(ET.SubElement(rail,kind,name=kind),'geometry'),'box')
            ET.SubElement(box,'size').text=f'7 {0.08 if closed_lane else .05} {rail_height}'
    if closed_lane:
        for x in [-1.85,4.7]:
            cap=ET.SubElement(arena,'link',name='end_'+str(x))
            ET.SubElement(cap,'pose').text=f'{x} 0 {base+rail_height/2} 0 0 0'
            for kind in ['collision','visual']:
                box=ET.SubElement(ET.SubElement(ET.SubElement(cap,kind,name=kind),'geometry'),'box')
                ET.SubElement(box,'size').text=f'.08 1.56 {rail_height}'
    ET.ElementTree(sdf).write(model/'model.sdf',encoding='utf-8',xml_declaration=True)
    (model/'model.config').write_text(
        '<?xml version="1.0"?><model><name>undulating_test</name><version>1.0</version>'
        '<sdf version="1.9">model.sdf</sdf><description>240/70 mm test road</description></model>\n')
    tree=ET.parse(simulator/'rmu_gazebo_simulator/resource/worlds/empty_world.sdf')
    world=tree.getroot().find('world')
    ET.SubElement(world.find("model[@name='ground_plane']"),'pose').text=f'0 0 {base} 0 0 0'
    wrapper=ET.SubElement(world,'model',name='test_arena')
    ET.SubElement(wrapper,'static').text='true'
    ET.SubElement(wrapper,'pose').text=f'0 0 0 0 0 {angle}'
    ET.SubElement(ET.SubElement(wrapper,'include'),'uri').text='model://undulating_test'
    tree.write(resources/'worlds/rmuc_2025_world.sdf',encoding='utf-8',xml_declaration=True)
    c,s=math.cos(angle),math.sin(angle)
    manifest={'wavelength_m':wavelength,'peak_to_trough_m':height,'base_z':base,
              'length_m':length,'width_m':width,'mesh_step_m':.005,'angle_rad':angle,
              'spawn':[-1.2*c,-1.2*s,.32,angle],'goal':[4.08*c,4.08*s],
              'simulator_path':str(output),'triangles':len(vertices),'closed_lane':closed_lane}
    (output/'geometry.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--simulator',type=Path,default=Path('/home/jsy/Projects/navigation/simulation/rmu-gazebo-simulator'))
    parser.add_argument('--angle-deg',type=float,default=0)
    parser.add_argument('--closed-lane',action='store_true',help='physical lane boundaries prevent bypassing the wave surface')
    args=parser.parse_args()
    print(json.dumps(build(args.output,args.simulator,math.radians(args.angle_deg),args.closed_lane),indent=2))
