import lxml.builder
import math

E = lxml.builder.ElementMaker()

color="0.8 0.7 0.5"
leg_size_1 = 0.2
leg_mass_1 = 1
leg_size_2 = 0.2
leg_mass_2 = 1
leg_positions = [
    "0.15 0.1",
    "0.15 -0.1",
    "-0.2 0.1",
    "-0.2 -0.1",
]
base_mass = 20 - (leg_mass_1 + leg_mass_2) * len(leg_positions)
base_x = 0.5
base_y = 0.4
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
        E.geometry(
            E.box(size=f"{base_x} {base_y} {base_z}")
        ),
        E.material(
            E.color(rgba=color + " 0.5"),
            name="a",
        ),
    ),
    name="base_link",
)
head_elements = [
    E.link(
        E.visual(
            E.geometry(
                E.box(size="0.1 0.1 0.1")
            ),
            E.material(
                E.color(rgba=color + " 0.5"),
                name="a",
            ),
            E.origin(xyz="0 0 0.05", rpy="0 0 0"),
        ),
        name="neck_link",
    ),
    E.joint(
        E.parent(link=f"base_link"),
        E.child(link=f"neck_link"),
        E.origin(xyz="0.2 0 0.1", rpy="0 0 0"),
        name="neck",
        type="fixed",
    ),
    E.link(
        E.visual(
            E.geometry(
                E.box(size="0.2 0.25 0.15")
            ),
            E.material(
                E.color(rgba=color + " 0.5"),
                name="a",
            ),
            E.origin(xyz="0 0 0.075", rpy="0 0 0"),
        ),
        name="head_link",
    ),
    E.joint(
        E.parent(link=f"neck_link"),
        E.child(link=f"head_link"),
        E.origin(xyz="0 0 0.1", rpy="0 0 0"),
        name="head",
        type="fixed",
    ),
]

eyes_length = 0.02
eyes_radius = 0.03
eyes_distance = 0.16

head_elements += [
    E.link(
        E.visual(
            E.geometry(
                E.cylinder(length=str(eyes_length), radius=str(eyes_radius))
            ),
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
        E.origin(xyz=f"0.1 {-eyes_distance/2} 0.075", rpy=f"0 {math.pi / 2} 0"),
        name="eye_left",
        type="fixed",
    ),
    E.link(
        E.visual(
            E.geometry(
                E.cylinder(length=str(eyes_length), radius=str(eyes_radius))
            ),
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
        E.origin(xyz=f"0.1 {eyes_distance/2} 0.075", rpy=f"0 {math.pi / 2} 0"),
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
                E.geometry(
                    E.box(size=f"0.05 0.05 {leg_size_1}")
                ),
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
                E.geometry(
                    E.box(size=f"0.05 0.05 {leg_size_2}")
                ),
                E.origin(xyz=f"0 0 {-leg_size_2/2}", rpy="0 0 0"),
                E.material(
                    E.color(rgba=color + " 1"),
                    name="a",
                ),
            ),
            E.collision(
                E.geometry(
                    E.box(size=f"0.05 0.05 {leg_size_2}")
                ),
                E.origin(xyz=f"0 0 {-leg_size_2/2}", rpy="0 0 0"),
            ),
            name=f"leg{i}_link_2",
        ),
        E.joint(
            E.parent(link=f"base_link"),
            E.child(link=f"leg{i}_link_1"),
            E.origin(xyz=p + " 0", rpy="0 0 0"),
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
    ]

urdf = E.robot(
    base_link,
    *head_elements,
    *leg_elements,
    name="hoge",
)

with open("hoge.urdf", "wb") as f:
    f.write(lxml.etree.tostring(urdf, pretty_print=True))
