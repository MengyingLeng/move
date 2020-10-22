from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper
import xml.etree.ElementTree as ET

stream = file('robot_calibration.yaml', 'r')
data = load(stream, Loader=Loader)
data = data['kinematics']

def modifyBody(body, translation, rotation):
    body[1].text = ' '.join(map(str, translation))
    body[2].text = ' '.join(map(str, rotation))

tree = ET.parse('ur5e.default.xml')
root = tree.getroot()

# modify link1
k_shoulder = data['shoulder']
t_1 = [k_shoulder['x'], k_shoulder['y'], k_shoulder['z']]
r_1 = [1, 0, 0, 90]
modifyBody(root[1], t_1, r_1)

# modify link2
t_1 = [data['forearm']['x'], 0, 0]
r_1 = [1, 0, 0, 0]
modifyBody(root[3], t_1, r_1)

# modify link3
t_1 = [data['wrist_1']['x'], 0, 0]
r_1 = [1, 0, 0, 0]
modifyBody(root[5], t_1, r_1)

# modify link4
t_1 = [0, 0, data['wrist_1']['z']]
r_1 = [1, 0, 0, 90]
modifyBody(root[7], t_1, r_1)

# modify link5
t_1 = [0, 0, -data['wrist_2']['y']]
r_1 = [-1, 0, 0, 90]
modifyBody(root[9], t_1, r_1)

# modify link6
t_1 = [0, 0, data['wrist_3']['y']]
r_1 = [1, 0, 0, 0]
modifyBody(root[11], t_1, r_1)


tree.write('test.xml')