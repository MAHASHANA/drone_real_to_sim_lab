#!/usr/bin/env python3
"""Generate a simple URDF for a 7-inch FPV frame.

The model uses primitives instead of CAD meshes so the same part list can be
used for depth-camera segmentation, pose matching, and simulator assembly.
"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = PROJECT_ROOT / "configs" / "frame_config.json"
GENERATED_DIR = PROJECT_ROOT / "generated"
URDF_PATH = GENERATED_DIR / "7in_fpv_frame.urdf"
MANIFEST_PATH = GENERATED_DIR / "part_manifest.json"


def indent(elem: ET.Element, level: int = 0) -> None:
    pad = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        for child in elem:
            indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = pad


def vec(values: tuple[float, float, float] | list[float]) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def add_inertial(link: ET.Element, mass: float, inertia: tuple[float, float, float]) -> None:
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{mass:.6g}")
    ixx, iyy, izz = inertia
    ET.SubElement(
        inertial,
        "inertia",
        ixx=f"{ixx:.6g}",
        ixy="0",
        ixz="0",
        iyy=f"{iyy:.6g}",
        iyz="0",
        izz=f"{izz:.6g}",
    )


def box_inertia(mass: float, size: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = size
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def cylinder_inertia(mass: float, radius: float, height: float) -> tuple[float, float, float]:
    ixy = mass * (3.0 * radius * radius + height * height) / 12.0
    izz = 0.5 * mass * radius * radius
    return (ixy, ixy, izz)


def add_materials(robot: ET.Element) -> None:
    materials = {
        "carbon_dark": "0.015 0.018 0.020 1",
        "carbon_edge": "0.08 0.10 0.11 1",
        "aluminum": "0.55 0.57 0.60 1",
        "motor_pad": "0.08 0.08 0.08 1",
    }
    for name, rgba in materials.items():
        material = ET.SubElement(robot, "material", name=name)
        ET.SubElement(material, "color", rgba=rgba)


def add_box_link(
    robot: ET.Element,
    name: str,
    size: tuple[float, float, float],
    mass: float,
    material: str,
) -> None:
    link = ET.SubElement(robot, "link", name=name)
    add_inertial(link, mass, box_inertia(mass, size))
    for tag in ("visual", "collision"):
        elem = ET.SubElement(link, tag)
        ET.SubElement(elem, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = ET.SubElement(elem, "geometry")
        ET.SubElement(geometry, "box", size=vec(size))
        if tag == "visual":
            ET.SubElement(elem, "material", name=material)


def add_cylinder_link(
    robot: ET.Element,
    name: str,
    radius: float,
    height: float,
    mass: float,
    material: str,
) -> None:
    link = ET.SubElement(robot, "link", name=name)
    add_inertial(link, mass, cylinder_inertia(mass, radius, height))
    for tag in ("visual", "collision"):
        elem = ET.SubElement(link, tag)
        ET.SubElement(elem, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = ET.SubElement(elem, "geometry")
        ET.SubElement(geometry, "cylinder", radius=f"{radius:.6g}", length=f"{height:.6g}")
        if tag == "visual":
            ET.SubElement(elem, "material", name=material)


def add_fixed_joint(
    robot: ET.Element,
    name: str,
    child: str,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    parent: str = "base_link",
) -> None:
    joint = ET.SubElement(robot, "joint", name=name, type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", xyz=vec(xyz), rpy=vec(rpy))


def main() -> None:
    cfg = json.loads(CONFIG_PATH.read_text())
    GENERATED_DIR.mkdir(exist_ok=True)

    robot = ET.Element("robot", name=cfg["robot_name"])
    add_materials(robot)

    bottom = cfg["bottom_plate"]
    bottom_size = tuple(bottom["size_m"])
    add_box_link(robot, "base_link", bottom_size, bottom["mass_kg"], "carbon_dark")

    parts = [
        {
            "name": "bottom_plate",
            "link": "base_link",
            "shape": "box",
            "size_m": list(bottom_size),
            "pose_xyz_m": [0.0, 0.0, bottom["z_m"]],
            "pose_rpy_rad": [0.0, 0.0, 0.0],
            "mass_kg": bottom["mass_kg"],
        }
    ]

    top = cfg["top_plate"]
    top_size = tuple(top["size_m"])
    add_box_link(robot, "top_plate", top_size, top["mass_kg"], "carbon_edge")
    add_fixed_joint(robot, "top_plate_joint", "top_plate", (0.0, 0.0, top["z_m"]))
    parts.append(
        {
            "name": "top_plate",
            "link": "top_plate",
            "shape": "box",
            "size_m": list(top_size),
            "pose_xyz_m": [0.0, 0.0, top["z_m"]],
            "pose_rpy_rad": [0.0, 0.0, 0.0],
            "mass_kg": top["mass_kg"],
        }
    )

    wheelbase = cfg["wheelbase_m"]
    if "motor_span_x_m" in cfg and "motor_span_y_m" in cfg:
        motor_x = cfg["motor_span_x_m"] / 2.0
        motor_y = cfg["motor_span_y_m"] / 2.0
        derived_wheelbase = math.hypot(cfg["motor_span_x_m"], cfg["motor_span_y_m"])
        if abs(derived_wheelbase - wheelbase) > 0.003:
            raise ValueError(
                "Configured motor spans do not match wheelbase: "
                f"{derived_wheelbase:.3f} m vs {wheelbase:.3f} m"
            )
    else:
        motor_radius = wheelbase / 2.0
        motor_x = motor_radius / math.sqrt(2.0)
        motor_y = motor_radius / math.sqrt(2.0)
    arm_length = cfg["arm_length_m"]
    arm_size = (arm_length, cfg["arm_width_m"], cfg["arm_thickness_m"])
    arm_mass = cfg["arm_mass_kg"]
    arm_z = 0.0
    mount = cfg["motor_mount"]
    motor_mount_z = cfg["arm_thickness_m"] / 2.0 + mount["thickness_m"] / 2.0

    arms = [
        ("front_right", motor_x, motor_y),
        ("front_left", -motor_x, motor_y),
        ("rear_left", -motor_x, -motor_y),
        ("rear_right", motor_x, -motor_y),
    ]
    arms = [(prefix, mx, my, math.atan2(my, mx)) for prefix, mx, my in arms]
    for prefix, mx, my, yaw in arms:
        arm_link = f"{prefix}_arm"
        add_box_link(robot, arm_link, arm_size, arm_mass, "carbon_dark")
        arm_center = (mx / 2.0, my / 2.0, arm_z)
        add_fixed_joint(robot, f"{arm_link}_joint", arm_link, arm_center, (0.0, 0.0, yaw))
        parts.append(
            {
                "name": arm_link,
                "link": arm_link,
                "shape": "box",
                "size_m": list(arm_size),
                "pose_xyz_m": list(arm_center),
                "pose_rpy_rad": [0.0, 0.0, yaw],
                "mass_kg": arm_mass,
            }
        )

        pad_link = f"motor_mount_{prefix}"
        add_cylinder_link(
            robot,
            pad_link,
            mount["radius_m"],
            mount["thickness_m"],
            mount["mass_kg"],
            "motor_pad",
        )
        add_fixed_joint(robot, f"{pad_link}_joint", pad_link, (mx, my, motor_mount_z))
        parts.append(
            {
                "name": pad_link,
                "link": pad_link,
                "shape": "cylinder",
                "radius_m": mount["radius_m"],
                "height_m": mount["thickness_m"],
                "pose_xyz_m": [mx, my, motor_mount_z],
                "pose_rpy_rad": [0.0, 0.0, 0.0],
                "mass_kg": mount["mass_kg"],
            }
        )

    standoff = cfg["stack_standoff"]
    half_spacing = standoff["spacing_m"] / 2.0
    standoff_z = standoff["height_m"] / 2.0
    for sx in (-half_spacing, half_spacing):
        for sy in (-half_spacing, half_spacing):
            name = f"stack_standoff_{'front' if sy > 0 else 'rear'}_{'right' if sx > 0 else 'left'}"
            add_cylinder_link(
                robot,
                name,
                standoff["radius_m"],
                standoff["height_m"],
                standoff["mass_kg"],
                "aluminum",
            )
            add_fixed_joint(robot, f"{name}_joint", name, (sx, sy, standoff_z))
            parts.append(
                {
                    "name": name,
                    "link": name,
                    "shape": "cylinder",
                    "radius_m": standoff["radius_m"],
                    "height_m": standoff["height_m"],
                    "pose_xyz_m": [sx, sy, standoff_z],
                    "pose_rpy_rad": [0.0, 0.0, 0.0],
                    "mass_kg": standoff["mass_kg"],
                }
            )

    indent(robot)
    tree = ET.ElementTree(robot)
    ET.register_namespace("", "")
    tree.write(URDF_PATH, encoding="utf-8", xml_declaration=True)
    MANIFEST_PATH.write_text(json.dumps({"parts": parts}, indent=2) + "\n")

    print(f"Wrote {URDF_PATH}")
    print(f"Wrote {MANIFEST_PATH}")
    print(
        "Motor centers: "
        f"x=+/-{motor_x:.4f} m, y=+/-{motor_y:.4f} m, "
        f"wheelbase {math.hypot(2.0 * motor_x, 2.0 * motor_y):.3f} m"
    )


if __name__ == "__main__":
    main()
