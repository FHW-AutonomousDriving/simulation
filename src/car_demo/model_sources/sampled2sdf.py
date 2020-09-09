import sys
import re

SCALE_FACTOR = 0.1
SDF_HEIGHT = 0.004
TEMPLATE = '''<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='%s'>
    <static>1</static>
    <link name='link'>
      <gravity>0</gravity>
      <visual name='visual'>
        <geometry>%s
        </geometry>
        <material>
          <lighting>1</lighting>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/White</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''

re_path_data = re.compile(r'(?<=\bd=")[^"]+')
title = sys.argv[1]
with open(title+'_sampled.svg') as svgdata:
  polyline_xml = ""
  path_datae = re_path_data.findall(svgdata.read())
  for path_data in path_datae:
    cmds = []
    tokens = iter(path_data.split())
    for token in tokens:
      if token in ['M','H','V','L']:
        cmds.append((token, tuple([float(e)*SCALE_FACTOR for e in next(tokens).split(',')])))
      elif token in ['Z']:
        pass
      else:
        raise RuntimeError(f'Unknown token {token}.')
        
    linecmds = []
    prev_coord = None
    for cmd in cmds:
      token, coord = cmd
      if token == 'H':
        coord = (coord[0],prev_coord[1])
      elif token == 'V':
        coord = (prev_coord[0],coord[0])
      linecmds.append((token, coord))
      prev_coord = coord

    coords = [cmd[1] for cmd in linecmds]
    polyline_xml += '\n          <polyline>\n'
    polyline_xml += f'            <height>{SDF_HEIGHT}</height>\n'
    for c in coords:
      polyline_xml += f'            <point>{c[1]:.3f} {c[0]:.3f}</point>\n'
    polyline_xml += '          </polyline>'
  print(TEMPLATE%('fhw-adas_'+title,polyline_xml))
