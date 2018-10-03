import numpy as np
from model_lib import *



a = 5 # side length
h = a * np.sqrt(3) / 2 # height length
delta_height = 3
delta_z = 2
upper_traingle_z = delta_height + delta_z
center = [0, 0, upper_traingle_z] # traingle center
thickness = .05
links = []
plugins = []
# upper taingle vertex positions
v1 = np.add(center, [0, (2*h/3), 0])
v2 = rotatez(v1, 120)
v3 = rotatez(v1, -120)

# upper traingle side positions
vs1 = np.add(center, [0, -(h/3), 0])
vs2 = rotatez(vs1, 120)
vs3 = rotatez(vs1, -120)
v = np.vstack((v1, v2, v3))

dir_vecs = np.stack([v3 - vs1, v1 - vs2, v2 - vs3])

# uper traingle side orientation
v1r = [0, np.radians(90), 0]
v2r = [0, np.radians(90), np.radians(120)]
v3r = [0, np.radians(90), np.radians(-120)]
orients = np.vstack((v1r, v2r, v3r))
vs = np.vstack((vs1, vs2, vs3))

# upper traingle pos = Position + Orientiation
upper_traingle = np.hstack((vs, orients))
links.append(create_link("upper_side1", upper_traingle[0], thickness, a))
links.append(create_link("upper_side2", upper_traingle[1], thickness, a))
links.append(create_link("upper_side3", upper_traingle[2], thickness, a))
side_orients = orients.copy()
upper_ribs = v.copy()
upper_ribs[:, 0:2] = upper_ribs[:, 0:2] / 2
ur1 = [0, np.radians(90), np.radians(90)]
ur2 = [0, np.radians(90), np.radians(30)]
ur3 = [0, np.radians(90), np.radians(-30)]
orients = np.vstack((ur1, ur2, ur3)) 

upper_ribs = np.hstack((upper_ribs, orients))
rib_length = 2*h/3
links.append(create_link("upper_rib1", upper_ribs[0], thickness, (2*h/3)))
links.append(create_link("upper_rib2", upper_ribs[1], thickness, (2*h/3)))
links.append(create_link("upper_rib3", upper_ribs[2], thickness, (2*h/3)))


r = 1
arm_length = r
harm_v = vs.copy() 
harm_v[:, 0:2] = 0
harm_v[:, 0:2] += vs[:, 0:2] + get_unit_vector(vs[:, 0:2]) * .5 * r
v1r = [np.radians(90), np.radians(90), 0]
v2r = [np.radians(90), np.radians(90), np.radians(120)]
v3r = [np.radians(90), np.radians(90), np.radians(-120)]
orients = np.vstack((v1r, v2r, v3r)) 
upper_arms = np.hstack((harm_v, orients))
lower_orient = orients.copy()
links.append(create_link("upper_arm1", upper_arms[0], thickness, r, mass=.01))
links.append(create_link("upper_arm2", upper_arms[1], thickness, r, mass=.01))
links.append(create_link("upper_arm3", upper_arms[2], thickness, r, mass=.01))
# r = 0.8

harm_v[:, 0:2] +=  get_unit_vector(vs[:, 0:2]) * .5 * r
upper_arms = np.hstack((harm_v, side_orients))
r = .5
links.append(create_link("hor_upper_arm1", upper_arms[0], thickness, r, mass=.01))
links.append(create_link("hor_upper_arm2", upper_arms[1], thickness, r, mass=.01))
links.append(create_link("hor_upper_arm3", upper_arms[2], thickness, r, mass=.01))
r = 1
lower_vertexes = vs * 0.2
varms = vs - lower_vertexes

la = 2 # side length
h = la * np.sqrt(3) / 2 # height length
center = [0, 0, delta_z] # traingle center

# upper taingle vertex positions
lv1 = np.array([0, (2*h/3), 0])
lv2 = rotatez(lv1, 120)
lv3 = rotatez(lv1, -120)
lv = np.vstack((lv1, lv2, lv3)) + center

# upper traingle side positions
vs1 = np.add(center, [0, -(h/3), 0])
vs2 = rotatez(vs1, 120)
vs3 = rotatez(vs1, -120)

# uper traingle side orientation
v1r = [0, np.radians(90), 0]
v2r = [0, np.radians(90), np.radians(120)]
v3r = [0, np.radians(90), np.radians(-120)]
orients = np.vstack((v1r, v2r, v3r))
lvs = np.vstack((vs1, vs2, vs3))

# upper traingle pos = Position + Orientiation
lower_traingle = np.hstack((lvs, orients))
la = .5
links.append(create_link("lower_side1", lower_traingle[0], thickness, la, mass=.01))
links.append(create_link("lower_side2", lower_traingle[1], thickness, la, mass=.01))
links.append(create_link("lower_side3", lower_traingle[2], thickness, la, mass=.01))

# lower_ribs = lv.copy()
# lower_ribs[:, 0:2] = lower_ribs[:, 0:2] / 2
# ur1 = [0, np.radians(90), np.radians(90)]
# ur2 = [0, np.radians(90), np.radians(30)]
# ur3 = [0, np.radians(90), np.radians(-30)]
# orients = np.vstack((ur1, ur2, ur3))
llvs = lvs.copy()
llvs[:, 0:2] = llvs[:, 0:2]/2 
lower_ribs = np.hstack((llvs, lower_orient))

lower_rib_length = 2*h/3
lower_rib_len = h / 3
links.append(create_link("lower_rib1", lower_ribs[0], thickness, (h/3),mass=.01))
links.append(create_link("lower_rib2", lower_ribs[1], thickness, (h/3), mass=.01))
links.append(create_link("lower_rib3", lower_ribs[2], thickness, (h/3), mass=.01))

upper_arm_end = vs[:, 0:3].copy()
upper_arm_end[:, 0:2]  += get_unit_vector(vs[:, 0:2])  * r
lower_sides_middle = lvs
vertical_diff = (upper_arm_end - lower_sides_middle)

rad_dif1 = get_angle(vertical_diff[0], [0, 0, 1])
rad_dif2 = get_angle(vertical_diff[1], [0, 0, 1])
rad_dif3 = get_angle(vertical_diff[2], [0, 0, 1])

vertical_arm_pos = vertical_diff * .5  + lvs
va = np.linalg.norm(vertical_diff[0])

v1r = [rad_dif1, 0, 0]
v2r = [rad_dif2, 0, np.radians(120)]
v3r = [rad_dif3, 0, np.radians(-120)]
orients = np.vstack((v1r, v2r, v3r)) 
dis = .1 * get_distance(dir_vecs).flatten()

vertical_arm_ = vertical_arm_pos + (dir_vecs * .1)
vertical_arm = np.hstack((vertical_arm_, orients))

links.append(create_link("vertical_arm1_left", vertical_arm[0], thickness, va, mass=.01))
links.append(create_link("vertical_arm2_left", vertical_arm[1], thickness, va, mass=.01))
links.append(create_link("vertical_arm3_left", vertical_arm[2], thickness, va, mass=.01))
vertical_arm_ = vertical_arm_pos - (dir_vecs * .1)
vertical_arm = np.hstack((vertical_arm_, orients))

links.append(create_link("vertical_arm1_right", vertical_arm[0], thickness, va, mass=.01))
links.append(create_link("vertical_arm2_right", vertical_arm[1], thickness, va, mass=.01))
links.append(create_link("vertical_arm3_right", vertical_arm[2], thickness, va, mass=.01))


joints = []

v1r = [0, np.radians(90), 0]
v2r = [0, np.radians(90), np.radians(120)]
v3r = [0, np.radians(90), np.radians(-120)]
orients = np.vstack((v1r, v2r, v3r))

upper_traingle = np.hstack((vs, orients))

holder_length = 1
u_traingle_holder = create_link("upper_triangle_holder", [0, 0, (delta_height + delta_z + holder_length/2), 0, 0, 0], thickness, holder_length)
links.append(u_traingle_holder)

lower_holder_length = .2
l_traingle_holder = create_link("lower_triangle_holder", [0, 0, (delta_z - lower_holder_length/2), 0, 0, 0], thickness, lower_holder_length, mass=.01)
links.append(l_traingle_holder)


horizontal_holder_length = 2 * rib_length 
horizontal_holder = create_link("horizontal_holder", [0, rib_length, (delta_height + delta_z + holder_length), np.radians(90), 0, 0], thickness, horizontal_holder_length, inertial=1.0)
links.append(horizontal_holder)

wall_hight = .5
wall_thickness = rib_length/2
wall = create_wall("wall", [0, rib_length * 2, wall_hight/2, 0, 0, 0], wall_thickness, wall_hight)
links.append(wall)

main_holder_length = holder_length + delta_height + delta_z - wall_hight
main_holder = create_link("main_holder", [0, rib_length * 2, main_holder_length/2 + wall_hight, 0, 0, 0], .2, main_holder_length, inertial=1.0)
links.append(main_holder)
camera_z = (delta_z - lower_holder_length/2) - lower_holder_length/2 - .05
camera = camera_link("camera_link", [0, 0, camera_z, 0, np.radians(90), 0], .05, .1)
links.append(camera)
joints.append(create_joint("upper_s2_s1", "upper_side1", "upper_side2", [0, 0, a/2, 0, 0, 0], [0, 0, 1], 0, 0))
joints.append(create_joint("upper_s2_s3", "upper_side3", "upper_side2", [0, 0, -a/2, 0, 0, 0], [0, 0, 1], 0, 0))
joints.append(create_joint("upper_s3_s1", "upper_side1", "upper_side3", [0, 0, a/2, 0, 0, 0], [0, 0, 1], 0, 0))
a = la
# joints.append(create_joint("lower_s2_s1", "lower_side1", "lower_side2", [0, 0, a/2, 0, 0, 0], [0, 0, 1], 0, 0))
# joints.append(create_joint("lower_s2_s3", "lower_side3", "lower_side2", [0, 0, -a/2, 0, 0, 0], [0, 0, 1], 0, 0))
# joints.append(create_joint("lower_s3_s1", "lower_side1", "lower_side3", [0, 0, a/2, 0, 0, 0], [0, 0, 1], 0, 0))

joints.append(create_joint("upper_arm_a1_s1", "upper_side1", "upper_arm1", [0, 0, -arm_length/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90)))
joints.append(create_joint("upper_arm_a2_s2", "upper_side2", "upper_arm2", [0, 0, -arm_length/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90)))
joints.append(create_joint("upper_arm_a3_s3", "upper_side3", "upper_arm3", [0, 0, -arm_length/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90)))

joints.append(create_joint("upper_arm_hor_a1_s1", "hor_upper_arm1", "upper_arm1", [0, 0, arm_length/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90)))
joints.append(create_joint("upper_arm_hor_a2_s2", "hor_upper_arm2", "upper_arm2", [0, 0, arm_length/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90)))
joints.append(create_joint("upper_arm_hor_a3_s3", "hor_upper_arm3", "upper_arm3", [0, 0, arm_length/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90)))

# joints.append(create_joint("upper_arm_a1_v1", "vertical_arm1", "upper_arm1", [0, 0, arm_length/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90)))
# joints.append(create_joint("upper_arm_a2_v2", "vertical_arm2", "upper_arm2", [0, 0, arm_length/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90)))
# joints.append(create_joint("upper_arm_a3_v3", "vertical_arm3", "upper_arm3", [0, 0, arm_length/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90)))

joints.append(create_joint("vertical_arm_a1_v1_right", "hor_upper_arm1", "vertical_arm1_right", [0, 0, va/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90), jtype='ball'))
joints.append(create_joint("vertical_arm_a2_v2_right", "hor_upper_arm2", "vertical_arm2_right", [0, 0, va/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90), jtype='ball'))
joints.append(create_joint("vertical_arm_a3_v3_right", "hor_upper_arm3", "vertical_arm3_right", [0, 0, va/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90), jtype='ball'))

joints.append(create_joint("vertical_arm_a1_v1_left", "hor_upper_arm1", "vertical_arm1_left", [0, 0, va/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90), jtype='ball'))
joints.append(create_joint("vertical_arm_a2_v2_left", "hor_upper_arm2", "vertical_arm2_left", [0, 0, va/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90), jtype='ball'))
joints.append(create_joint("vertical_arm_a3_v3_left", "hor_upper_arm3", "vertical_arm3_left", [0, 0, va/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90), jtype='ball'))

joints.append(create_joint("lvertical_arm_a1_v1_right", "lower_side1", "vertical_arm1_right", [0, 0, -va/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90), jtype='ball'))
joints.append(create_joint("lvertical_arm_a2_v2_right", "lower_side2", "vertical_arm2_right", [0, 0, -va/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90), jtype='ball'))
joints.append(create_joint("lvertical_arm_a3_v3_right", "lower_side3", "vertical_arm3_right", [0, 0, -va/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90), jtype='ball'))

joints.append(create_joint("lvertical_arm_a1_v1_left", "lower_side1", "vertical_arm1_left", [0, 0, -va/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90), jtype='ball'))
joints.append(create_joint("lvertical_arm_a2_v2_left", "lower_side2", "vertical_arm2_left", [0, 0, -va/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90), jtype='ball'))
joints.append(create_joint("lvertical_arm_a3_v3_left", "lower_side3", "vertical_arm3_left", [0, 0, -va/2, 0, 0, 0], [0, 1, 0], rad(90), -rad(90), jtype='ball'))


joints.append(create_joint("upper_rib1_s1", "upper_side1", "upper_rib1", [0, 0, rib_length/2, 0, 0, 0], [1, 0, 0], 0, 0))
joints.append(create_joint("upper_rib2_s2", "upper_side2", "upper_rib2", [0, 0, -rib_length/2, 0, 0, 0], [1, 0, 0], 0, 0))
joints.append(create_joint("upper_rib3_s3", "upper_side3", "upper_rib3", [0, 0, rib_length/2, 0, 0, 0], [1, 0, 0], 0, 0))

joints.append(create_joint("upper_rib1_holder", "upper_triangle_holder", "upper_rib1", [0, 0, -rib_length/2, 0, 0, 0], [1, 0, 0], 0, 0))
joints.append(create_joint("upper_rib2_holder", "upper_triangle_holder", "upper_rib2", [0, 0, rib_length/2, 0, 0, 0], [1, 0, 0], 0, 0))
joints.append(create_joint("upper_rib3_holder", "upper_triangle_holder", "upper_rib3", [0, 0, -rib_length/2, 0, 0, 0], [1, 0, 0], 0, 0))

joints.append(create_joint("lower_rib1_s1", "lower_rib1", "lower_side1", [0, 0, 0, 0, 0, 0], [0, 1, 0], 0, 0))
joints.append(create_joint("lower_rib2_s2", "lower_rib2", "lower_side2", [0, 0, 0, 0, 0, 0], [0, 1, 0], 0, 0))
joints.append(create_joint("lower_rib3_s3", "lower_rib3", "lower_side3", [0, 0, 0, 0, 0, 0], [0, 1, 0], 0, 0))

joints.append(create_joint("lower_rib1_holder", "lower_triangle_holder", "lower_rib1", [0, 0, -lower_rib_len/2, 0, 0, 0], [1, 0, 0], 0, 0))
joints.append(create_joint("lower_rib2_holder", "lower_triangle_holder", "lower_rib2", [0, 0, -lower_rib_len/2, 0, 0, 0], [1, 0, 0], 0, 0))
joints.append(create_joint("lower_rib3_holder", "lower_triangle_holder", "lower_rib3", [0, 0, -lower_rib_len/2, 0, 0, 0], [1, 0, 0], 0, 0))


joints.append(create_joint("lower_rib12", "lower_rib1", "lower_rib1", [0, 0, -lower_rib_len/2, 0, 0, 0], [1, 0, 0], 0, 0))
joints.append(create_joint("lower_rib32", "lower_rib3", "lower_rib2", [0, 0, -lower_rib_len/2, 0, 0, 0], [1, 0, 0], 0, 0))
joints.append(create_joint("lower_rib13", "lower_rib1", "lower_rib3", [0, 0, -lower_rib_len/2, 0, 0, 0], [1, 0, 0], 0, 0))


joints.append(create_joint("u_tria_v_h_holder", "horizontal_holder", "upper_triangle_holder", [0, 0, holder_length/2, 0, 0, 0], [1, 0, 0], 0, 0))
joints.append(create_joint("h_v_holder", "main_holder", "horizontal_holder", [0, 0, -horizontal_holder_length/2, 0, 0, 0], [1, 0, 0], 0, 0))
joints.append(create_joint("v_wall_holder", "wall", "main_holder", [0, 0, -main_holder_length/2, 0, 0, 0], [1, 0, 0], 0, 0))
joints.append(create_joint("camera_joint", "lower_triangle_holder", "camera_link", [0, 0, 0, 0, 0, 0], [1, 0, 0], 0, 0))

plugins.append(plugin("arm_control", "libdelta3_gazebo.so"))

links.extend(joints)
links.extend(plugins)
world = create_world(links)
open('../models/model.sdf', 'w').write(world)

