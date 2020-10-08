#!/usr/bin/env python
# encoding: utf-8

import xml.etree.ElementTree as ET
import sys
import math

inpath = sys.argv[1]

if (inpath.endswith('.sdf')):
    print('<?xml version="1.0" encoding="UTF-8" standalone="no"?>')
    print('<svg>')
    tree = ET.parse(inpath)
    root = tree.getroot()
    for include in root.iter('include'):
        if (include.find('uri').text == 'model://fhw-adas_guidebox'):
            identifier = include.find('name').text.replace('fhw-adas_guidebox','rect')
            pose = include.find('pose').text.split()
            print(pose, file=sys.stderr)
            [x, y, z, roll, pitch, yaw] = map(float, pose)
            print(f'<rect id="{identifier}" width="1" height="1" transform="matrix({math.cos(yaw):.6},{math.sin(yaw):.6},{2*-math.sin(yaw):.6},{2*math.cos(yaw):.6},{x},{y})" />')
    print('</svg>')
elif (inpath.endswith('.svg')):
    ns_array = {
        'svg': 'http://www.w3.org/2000/svg', 
        'xlink': 'http://www.w3.org/1999/xlink'
    }
    tree = ET.parse(inpath)
    root = tree.getroot()
    for rect in root.findall('svg:rect', ns_array):
        #print(rect, file=sys.stderr)
        name = rect.get('id').replace('rect','fhw-adas_guidebox')
        transform = rect.get('transform')
        [a, b, c, d, e, f] = map(float, transform.strip('matrix()').split(','))
        print(f'''    <include>
      <name>{name}</name>
      <uri>model://fhw-adas_guidebox</uri>
      <pose>{e} {f} 1 0 0 {math.atan2(b, a):.6}</pose>
    </include>''')
else:
    print('Neither svg nor sdf input.', file=sys.stderr)
