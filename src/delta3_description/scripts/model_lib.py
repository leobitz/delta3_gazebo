import numpy as np
import math

def get_unit_vector(vector):
    if len(vector.shape) == 1:
        norm = np.linalg.norm(vector)
        unit = np.divide(vector, norm)
        return unit
    else:
        norms = np.linalg.norm(vector, axis=1, keepdims=True)
        units = np.divide(vector, norms)
        return units

def rotatez(v, degree):
    rad = np.radians(degree)
    m = np.array([
        [np.cos(rad), -np.sin(rad), 0],
        [np.sin(rad), np.cos(rad), 0],
        [0,         0,            1]
    ])
    return np.dot(m, v)

def rotatey(v, degree):
    rad = np.radians(degree)
    m = np.array([
        [np.cos(rad),0, np.sin(rad)],
        [0,         1,            0],
        [-np.sin(rad),0, np.cos(rad)],
    ])
    return np.dot(m, v)

def rotatex(v, degree):
    rad = np.radians(degree)
    m = np.array([
        [1,         0,          0],
        [0, np.cos(rad), -np.sin(rad)],
        [0, np.sin(rad), np.cos(rad)],
        
    ])
    return np.dot(m, v)
    
def get_angle(v1, v2):
    norm1 = np.linalg.norm(v1)
    norm2 = np.linalg.norm(v2)
    dot = np.dot(v1, v2)
    bottom = norm1 * norm2
    cos_inverse = dot/bottom
    rad = np.arccos(cos_inverse)
    return rad

def get_3D_angles(v):
    angles = []
    for axis in [[1, 0, 0], [0, 1, 0], [0, 0, 1]]:
        rads = get_angle(v, axis)
        angles.append(rads)
    return np.array(angles)

def get_distance(vector):
    if len(vector.shape) == 1:
        norm = np.linalg.norm(vector)
        return norm
    else:
        norms = np.linalg.norm(vector, axis=1, keepdims=True)
        return norms

def get_orientations(vectors):
    orients = []
    for v in vectors:
        orients.append(get_3D_angles(v))
    orients = np.vstack(orients)
    return orients

def create_world(links):
    ls = ""
    for link in links:
        ls += link
    world = """<?xml version="1.0" ?>
<sdf version="1.5">
<model name="delta3">
    <static>false</static>
    {0}
</model>
</sdf>
    """.format(ls)
    return world

def create_link(name, pos, radius, length):
    link = """
    <link name="{8}">
        <pose>{0} {1} {2} {3} {4} {5}</pose>
        <gravity>0</gravity>
        <inertial>
            <mass>0.1</mass>
            <inertia>
                <ixx>0.0005</ixx>
                <iyy>0.0005</iyy>
                <izz>0.0005</izz>
            </inertia>
        </inertial>
        <collision name="col">
            <geometry>
                <cylinder>
                    <radius>{6}</radius>
                    <length>{7}</length>
                  </cylinder>
            </geometry>
        </collision>
        <visual name="visual">
            <geometry>
                <cylinder>
                    <radius>{6}</radius>
                    <length>{7}</length>
                </cylinder>
            </geometry>
          </visual>
    </link>
    """.format(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5], radius, length, name)
    return link

def create_wall(name, pos, radius, length, mass=500):
    link = """
    <link name="{8}">
        <pose>{0} {1} {2} {3} {4} {5}</pose>
        <inertial>
            <mass>500</mass>
        </inertial>
        <collision name="col">
            <geometry>
                <cylinder>
                    <radius>{6}</radius>
                    <length>{7}</length>
                  </cylinder>
            </geometry>
        </collision>
        <visual name="visual">
            <geometry>
                <cylinder>
                    <radius>{6}</radius>
                    <length>{7}</length>
                </cylinder>
            </geometry>
          </visual>
    </link>
    """.format(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5], radius, length, name)
    return link

def create_joint(name, parent, child, pos, axis, upper, lower, jtype="revolute"):
    joint = """<joint type="{13}" name="{14}">
                    <pose>{0} {1} {2} {3} {4} {5}</pose>
                    <child>{6}</child>
                    <parent>{7}</parent>
                        <axis>
                            <dynamics>
                                <friction>1.0</friction>
                            </dynamics>
                            <xyz>{8} {9} {10}</xyz>
                            <limit>
                                <upper>{11}</upper>
                                <lower>{12}</lower>
                            </limit>
                        </axis>
			    </joint>
            """.format(
                pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],
                child, parent,
                axis[0],axis[1],axis[2],
                upper, lower,
                jtype, name
            )
    return joint

def plugin(name, file):
    tag = """
        <plugin name="{0}" filename="{1}" />
        """.format(name, file)
    return tag

def rad(deg):
    return deg * math.pi / 180

def deg(rad):
    return rad * 180/ math.pi
