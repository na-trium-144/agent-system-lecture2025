import lxml.builder
import math

E = lxml.builder.ElementMaker()

color = "0.8 0.8 0.8"
alpha = " 0.7"
leg_size_1 = 0.2
leg_size_2 = 0.25
leg_mass_1 = 1.0
leg_mass_2 = 2.0
leg_mass_3 = 0.1
longleg_size = 0.25
longleg_mass = 0.5
longleg_num = 12
leg_positions = [
    "0.1 0.08",
    "0.1 -0.08",
    "-0.15 0.08",
    "-0.15 -0.08",
]
base_mass = max(1, 20 - (leg_mass_1 + leg_mass_2 + leg_mass_3) * len(leg_positions) - longleg_mass * longleg_num)
base_x = 0.4
base_y = 0.3
base_z = 0.2

base_link = E.link(
    E.inertial(
        E.origin(xyz="0 0 0", rpy="0 0 0"),
        E.mass(value=str(base_mass)),
        E.inertia(
            ixx=str(base_mass * (base_y**2 + base_z**2) / 12),
            iyy=str(base_mass * (base_x**2 + base_z**2) / 12),
            izz=str(base_mass * (base_x**2 + base_y**2) / 12),
            ixy="0",
            ixz="0",
            iyz="0",
        ),
    ),
    E.visual(
        E.geometry(E.box(size=f"{base_x} {base_y} {base_z}")),
        E.material(
            E.color(rgba=color + alpha),
            name="a",
        ),
    ),
    name="base_link",
)
head_elements = [
    E.link(
        E.inertial(
            E.origin(xyz="0 0 0", rpy="0 0 0"),
            E.mass(value="0.001"),
            E.inertia(ixx="0",iyy="0",izz="0",ixy="0",ixz="0",iyz="0"),
        ),
        E.visual(
            E.geometry(E.box(size="0.2 0.2 0.2")),
            E.material(
                E.color(rgba=color + alpha),
                name="a",
            ),
            E.origin(xyz="0 0 0.1", rpy="0 0 0"),
        ),
        name="head_link",
    ),
    E.joint(
        E.parent(link=f"base_link"),
        E.child(link=f"head_link"),
        E.origin(xyz=f"{base_x / 2 + 0.1 - 0.05} 0 {base_z / 2 - 0.05}", rpy="0 0 0"),
        name="head",
        type="fixed",
    ),
    E.link(
        E.inertial(
            E.origin(xyz="0 0 0", rpy="0 0 0"),
            E.mass(value="0.001"),
            E.inertia(ixx="0",iyy="0",izz="0",ixy="0",ixz="0",iyz="0"),
        ),
        E.visual(
            E.geometry(E.box(size="0.1 0.1 0.05")),
            E.material(
                E.color(rgba=color + alpha),
                name="a",
            ),
            E.origin(xyz="0.05 0 0.025", rpy="0 0 0"),
        ),
        name="nose_link",
    ),
    E.joint(
        E.parent(link=f"head_link"),
        E.child(link=f"nose_link"),
        E.origin(xyz=f"{0.1 - 0.03} 0 0", rpy="0 0 0"),
        name="nose",
        type="fixed",
    ),
    E.link(
        E.inertial(
            E.origin(xyz="0 0 0", rpy="0 0 0"),
            E.mass(value="0.001"),
            E.inertia(ixx="0",iyy="0",izz="0",ixy="0",ixz="0",iyz="0"),
        ),
        E.visual(
            E.geometry(E.box(size="0.05 0.1 0.02")),
            E.material(
                E.color(rgba="0 0 0 0.5"),
                name="a",
            ),
            E.origin(xyz="-0.025 0 -0.01", rpy="0 0 0"),
        ),
        name="nose_tip_link",
    ),
    E.joint(
        E.parent(link=f"nose_link"),
        E.child(link=f"nose_tip_link"),
        E.origin(xyz=f"0.1 0 0.05", rpy="0 0 0"),
        name="nose_tip",
        type="fixed",
    ),
    E.link(
        E.inertial(
            E.origin(xyz="0 0 0", rpy="0 0 0"),
            E.mass(value="0.001"),
            E.inertia(ixx="0",iyy="0",izz="0",ixy="0",ixz="0",iyz="0"),
        ),
        E.visual(
            E.geometry(E.box(size="0.05 0.1 0.1")),
            E.material(
                E.color(rgba=color + alpha),
                name="a",
            ),
            E.origin(xyz="0 0.05 0.05", rpy="0 0 0"),
        ),
        name="ear_left_link",
    ),
    E.joint(
        E.parent(link=f"head_link"),
        E.child(link=f"ear_left_link"),
        E.origin(xyz=f"-0.02 {0.2 / 2 - 0.05} {0.2 - 0.05}", rpy="0 0 0"),
        name="ear_left",
        type="fixed",
    ),
    E.link(
        E.inertial(
            E.origin(xyz="0 0 0", rpy="0 0 0"),
            E.mass(value="0.001"),
            E.inertia(ixx="0",iyy="0",izz="0",ixy="0",ixz="0",iyz="0"),
        ),
        E.visual(
            E.geometry(E.box(size="0.05 0.1 0.1")),
            E.material(
                E.color(rgba=color + alpha),
                name="a",
            ),
            E.origin(xyz="0 -0.05 0.05", rpy="0 0 0"),
        ),
        name="ear_right_link",
    ),
    E.joint(
        E.parent(link=f"head_link"),
        E.child(link=f"ear_right_link"),
        E.origin(xyz=f"-0.02 {-0.2 / 2 + 0.05} {0.2 - 0.05}", rpy="0 0 0"),
        name="ear_right",
        type="fixed",
    ),
    E.link(
        E.inertial(
            E.origin(xyz="0 0 0", rpy="0 0 0"),
            E.mass(value="0.001"),
            E.inertia(ixx="0",iyy="0",izz="0",ixy="0",ixz="0",iyz="0"),
        ),
        E.visual(
            E.geometry(E.box(size="0.1 0.1 0.05")),
            E.material(
                E.color(rgba=color + alpha),
                name="a",
            ),
            E.origin(xyz="-0.05 0 0", rpy="0 0 0"),
        ),
        name="tail_link",
    ),
    E.joint(
        E.parent(link=f"base_link"),
        E.child(link=f"tail_link"),
        E.origin(xyz=f"{-base_x / 2 + 0.05} 0 {base_z / 2}", rpy="0 0 0"),
        name="tail",
        type="fixed",
    ),
]

eyes_length = 0.02
eyes_radius = 0.02
eyes_distance = 0.1

head_elements += [
    E.link(
        E.inertial(
            E.origin(xyz="0 0 0", rpy="0 0 0"),
            E.mass(value="0.001"),
            E.inertia(ixx="0",iyy="0",izz="0",ixy="0",ixz="0",iyz="0"),
        ),
        E.visual(
            E.geometry(E.cylinder(length=str(eyes_length), radius=str(eyes_radius))),
            E.material(
                E.color(rgba="0 0 0 0.5"),
                name="a",
            ),
            E.origin(xyz="0 0 0", rpy="0 0 0"),
        ),
        name="eye_left_link",
    ),
    E.joint(
        E.parent(link=f"head_link"),
        E.child(link=f"eye_left_link"),
        E.origin(xyz=f"0.1 {-eyes_distance/2} 0.1", rpy=f"0 {math.pi / 2} 0"),
        name="eye_left",
        type="fixed",
    ),
    E.link(
        E.inertial(
            E.origin(xyz="0 0 0", rpy="0 0 0"),
            E.mass(value="0.001"),
            E.inertia(ixx="0",iyy="0",izz="0",ixy="0",ixz="0",iyz="0"),
        ),
        E.visual(
            E.geometry(E.cylinder(length=str(eyes_length), radius=str(eyes_radius))),
            E.material(
                E.color(rgba="0 0 0 0.5"),
                name="a",
            ),
            E.origin(xyz="0 0 0", rpy="0 0 0"),
        ),
        name="eye_right_link",
    ),
    E.joint(
        E.parent(link=f"head_link"),
        E.child(link=f"eye_right_link"),
        E.origin(xyz=f"0.1 {eyes_distance/2} 0.1", rpy=f"0 {math.pi / 2} 0"),
        name="eye_right",
        type="fixed",
    ),
]

leg_elements = []
for i, p in enumerate(leg_positions):
    leg_elements += [
        E.link(
            E.inertial(
                E.origin(xyz=f"0 0 {-leg_size_1/2}", rpy="0 0 0"),
                E.mass(value=str(leg_mass_1)),
                E.inertia(
                    ixx=str(leg_mass_1 * (0.05**2 + leg_size_1**2) / 12),
                    iyy=str(leg_mass_1 * (0.05**2 + leg_size_1**2) / 12),
                    izz=str(leg_mass_1 * (0.05**2 + 0.05**2) / 12),
                    ixy="0",
                    ixz="0",
                    iyz="0",
                ),
            ),
            E.visual(
                E.geometry(E.box(size=f"0.05 0.05 {leg_size_1}")),
                E.origin(xyz=f"0 0 {-leg_size_1/2}", rpy="0 0 0"),
                E.material(
                    E.color(rgba=color + " 1"),
                    name="a",
                ),
            ),
            name=f"leg{i}_link_1",
        ),
        E.link(
            E.inertial(
                E.origin(xyz=f"0 0 {-leg_size_2/2}", rpy="0 0 0"),
                E.mass(value=str(leg_mass_2)),
                E.inertia(
                    ixx=str(leg_mass_2 * (0.05**2 + leg_size_2**2) / 12),
                    iyy=str(leg_mass_2 * (0.05**2 + leg_size_2**2) / 12),
                    izz=str(leg_mass_2 * (0.05**2 + 0.05**2) / 12),
                    ixy="0",
                    ixz="0",
                    iyz="0",
                ),
            ),
            E.visual(
                E.geometry(E.box(size=f"0.05 0.05 {leg_size_2}")),
                E.origin(xyz=f"0 0 {-leg_size_2/2}", rpy="0 0 0"),
                E.material(
                    E.color(rgba=color + " 1"),
                    name="a",
                ),
            ),
            E.collision(
                E.geometry(E.box(size=f"0.05 0.05 {leg_size_2}")),
                E.origin(xyz=f"0 0 {-leg_size_2/2}", rpy="0 0 0"),
            ),
            name=f"leg{i}_link_2",
        ),
        E.link(
            E.inertial(
                E.origin(xyz=f"0 0 0", rpy="0 0 0"),
                E.mass(value=str(leg_mass_3)),
                E.inertia(
                    ixx=str(leg_mass_3 * (0.05**2 + 0.05**2) / 12),
                    iyy=str(leg_mass_3 * (0.05**2 + 0.05**2) / 12),
                    izz=str(leg_mass_3 * (0.05**2 + 0.05**2) / 12),
                    ixy="0",
                    ixz="0",
                    iyz="0",
                ),
            ),
            E.visual(
                E.geometry(E.cylinder(radius="0.03", length="0.05")),
                E.origin(xyz=f"0 0 0", rpy=f"{math.pi/2} 0 0"),
                E.material(
                    E.color(rgba=color + " 1"),
                    name="a",
                ),
            ),
            E.collision(
                E.geometry(E.cylinder(radius="0.03", length="0.05")),
                E.origin(xyz=f"0 0 0", rpy=f"{math.pi/2} 0 0"),
            ),
            name=f"leg{i}_link_3",
        ),
        E.link(
            E.inertial(
                E.origin(xyz=f"0 0 0", rpy="0 0 0"),
                E.mass(value="0.001"),
                E.inertia(ixx="0", iyy="0", izz="0", ixy="0", ixz="0", iyz="0"),
            ),
            E.visual(
                E.geometry(E.cylinder(radius="0.025", length="0.05")),
                E.origin(xyz=f"0 0 0", rpy=f"{math.pi/2} 0 0"),
                E.material(
                    E.color(rgba=color + " 1"),
                    name="a",
                ),
            ),
            name=f"leg{i}_link_4",
        ),
        E.joint(
            E.parent(link=f"base_link"),
            E.child(link=f"leg{i}_link_1"),
            E.origin(xyz=p + " 0.05", rpy="0 0 0"),
            E.axis(xyz="0 1 0"),
            name=f"leg{i}_joint_1",
            type="continuous",
        ),
        E.joint(
            E.parent(link=f"leg{i}_link_1"),
            E.child(link=f"leg{i}_link_2"),
            E.origin(xyz=f"0 0 {-leg_size_1}", rpy="0 0 0"),
            E.axis(xyz="0 1 0"),
            name=f"leg{i}_joint_2",
            type="continuous",
        ),
        E.joint(
            E.parent(link=f"leg{i}_link_2"),
            E.child(link=f"leg{i}_link_3"),
            E.origin(xyz=f"0 0 {-leg_size_2}", rpy="0 0 0"),
            name=f"leg{i}_joint_3",
            type="fixed",
        ),
        E.joint(
            E.parent(link=f"leg{i}_link_2"),
            E.child(link=f"leg{i}_link_4"),
            E.origin(xyz=f"0 0 0", rpy="0 0 0"),
            name=f"leg{i}_joint_4",
            type="fixed",
        ),
    ]

longleg_elements = []
for i in range(longleg_num):
    leg_elements += [
        E.link(
            E.inertial(
                E.origin(xyz=f"0 0 {-longleg_size/2}", rpy="0 0 0"),
                E.mass(value=str(longleg_mass)),
                E.inertia(
                    ixx=str(longleg_mass * (0.05**2 + longleg_size**2) / 12),
                    iyy=str(longleg_mass * (0.05**2 + longleg_size**2) / 12),
                    izz=str(longleg_mass * (0.05**2 + 0.05**2) / 12),
                    ixy="0",
                    ixz="0",
                    iyz="0",
                ),
            ),
            E.visual(
                E.geometry(E.box(size=f"0.05 0.05 {longleg_size}")),
                E.origin(xyz=f"0 0 {-longleg_size/2}", rpy="0 0 0"),
                E.material(
                    E.color(rgba=color + " 1"),
                    name="a",
                ),
            ),
            E.collision(
                E.geometry(E.box(size=f"0.05 0.05 {longleg_size}")),
                E.origin(xyz=f"0 0 {-longleg_size/2}", rpy="0 0 0"),
            ) if i == longleg_num - 1 else "",
            name=f"longleg_link_{i}",
        ),
        E.joint(
            E.parent(link=f"base_link" if i == 0 else f"longleg_link_{i-1}"),
            E.child(link=f"longleg_link_{i}"),
            E.origin(xyz=f"{longleg_size/2} 0 0" if i == 0 else f"0 0 {-longleg_size}", rpy="0 0 0"),
            E.axis(xyz="0 1 0"),
            name=f"longleg_joint_{i}",
            type="continuous",
        ),
    ]

urdf = E.robot(
    base_link,
    *head_elements,
    *leg_elements,
    *longleg_elements,
    name="framy",
)

with open("framy.urdf", "wb") as f:
    f.write(lxml.etree.tostring(urdf, pretty_print=True))
