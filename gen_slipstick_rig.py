# slip_stick_urdf.py
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Sequence, Tuple, Union


def _prettify(elem: ET.Element, level: int = 0) -> None:
    indent = "  "
    i = "\n" + level * indent
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + indent
        for child in elem:
            _prettify(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = i
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


leg_col = [58, 161, 236, 1.0]
toe_col = [255, 149, 56, 1.0]
sol_col = [255, 220, 0, 1.0]
mas_col = [56, 56, 56, 0.75]

RGBA_LEG = (leg_col[0]/255, leg_col[1]/255, leg_col[2]/255, leg_col[3])
RGBA_TOE_HEEL = (toe_col[0]/255, toe_col[1]/255, toe_col[2]/255, toe_col[3])
RGBA_SOLE = (sol_col[0]/255, sol_col[1]/255, sol_col[2]/255, sol_col[3])
RGBA_MASS = (mas_col[0]/255, mas_col[1]/255, mas_col[2]/255, mas_col[3])


@dataclass(frozen=True)
class HydroRigidConfig:
    mesh_resolution_hint: float
    mu_static: float
    mu_dynamic: float
    dissipation: float


@dataclass(frozen=True)
class HydroSoftConfig:
    hydroelastic_modulus: float
    mesh_resolution_hint: float
    mu_static: float
    mu_dynamic: float
    dissipation: float


def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _box_inertia_about_com(m: float, size_xyz: Sequence[float]) -> Tuple[float, float, float]:
    x, y, z = size_xyz
    ixx = (m / 12.0) * (y * y + z * z)
    iyy = (m / 12.0) * (x * x + z * z)
    izz = (m / 12.0) * (x * x + y * y)
    return ixx, iyy, izz


def _sphere_inertia_about_com(m: float, r: float) -> Tuple[float, float, float]:
    i = (2.0 / 5.0) * m * r * r
    return i, i, i


def _add_material(parent: ET.Element, name: str, rgba: Tuple[float, float, float, float]) -> None:
    mat = ET.SubElement(parent, "material", name=name)
    ET.SubElement(mat, "color", rgba=f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}")


def _add_visual_box(
    link: ET.Element,
    size_xyz: Sequence[float],
    xyz: Sequence[float],
    rpy: Sequence[float],
    material_name: str
) -> None:
    vis = ET.SubElement(link, "visual")
    ET.SubElement(vis, "origin", xyz=f"{xyz[0]} {xyz[1]} {xyz[2]}", rpy=f"{rpy[0]} {rpy[1]} {rpy[2]}")
    geom = ET.SubElement(vis, "geometry")
    ET.SubElement(geom, "box", size=f"{size_xyz[0]} {size_xyz[1]} {size_xyz[2]}")
    ET.SubElement(vis, "material", name=material_name)


def _add_visual_sphere(
    link: ET.Element,
    radius: float,
    xyz: Sequence[float],
    rpy: Sequence[float],
    material_name: str
) -> None:
    vis = ET.SubElement(link, "visual")
    ET.SubElement(vis, "origin", xyz=f"{xyz[0]} {xyz[1]} {xyz[2]}", rpy=f"{rpy[0]} {rpy[1]} {rpy[2]}")
    geom = ET.SubElement(vis, "geometry")
    ET.SubElement(geom, "sphere", radius=f"{radius}")
    ET.SubElement(vis, "material", name=material_name)


def _add_visual_cylinder(
    link: ET.Element,
    radius: float,
    length: float,
    xyz: Sequence[float],
    rpy: Sequence[float],
    material_name: str
) -> None:
    vis = ET.SubElement(link, "visual")
    ET.SubElement(vis, "origin", xyz=f"{xyz[0]} {xyz[1]} {xyz[2]}", rpy=f"{rpy[0]} {rpy[1]} {rpy[2]}")
    geom = ET.SubElement(vis, "geometry")
    ET.SubElement(geom, "cylinder", radius=f"{radius}", length=f"{length}")
    ET.SubElement(vis, "material", name=material_name)


def _add_collision_box_with_hydro(
    link: ET.Element,
    size_xyz: Sequence[float],
    xyz: Sequence[float],
    rpy: Sequence[float],
    hydro: Union[HydroRigidConfig, HydroSoftConfig]
) -> None:
    col = ET.SubElement(link, "collision")
    ET.SubElement(col, "origin", xyz=f"{xyz[0]} {xyz[1]} {xyz[2]}", rpy=f"{rpy[0]} {rpy[1]} {rpy[2]}")
    geom = ET.SubElement(col, "geometry")
    ET.SubElement(geom, "box", size=f"{size_xyz[0]} {size_xyz[1]} {size_xyz[2]}")

    prox = ET.SubElement(col, "{http://drake.mit.edu}proximity_properties")

    if isinstance(hydro, HydroRigidConfig):
        ET.SubElement(prox, "{http://drake.mit.edu}rigid_hydroelastic")
        ET.SubElement(prox, "{http://drake.mit.edu}mesh_resolution_hint", value=f"{hydro.mesh_resolution_hint}")
        ET.SubElement(prox, "{http://drake.mit.edu}mu_static", value=f"{hydro.mu_static}")
        ET.SubElement(prox, "{http://drake.mit.edu}mu_dynamic", value=f"{hydro.mu_dynamic}")
        ET.SubElement(prox, "{http://drake.mit.edu}hunt_crossley_dissipation", value=f"{hydro.dissipation}")
    else:
        ET.SubElement(prox, "{http://drake.mit.edu}compliant_hydroelastic")
        ET.SubElement(prox, "{http://drake.mit.edu}hydroelastic_modulus", value=f"{hydro.hydroelastic_modulus}")
        ET.SubElement(prox, "{http://drake.mit.edu}mesh_resolution_hint", value=f"{hydro.mesh_resolution_hint}")
        ET.SubElement(prox, "{http://drake.mit.edu}mu_static", value=f"{hydro.mu_static}")
        ET.SubElement(prox, "{http://drake.mit.edu}mu_dynamic", value=f"{hydro.mu_dynamic}")
        ET.SubElement(prox, "{http://drake.mit.edu}hunt_crossley_dissipation", value=f"{hydro.dissipation}")


def write_slip_stick_urdf(
    rig_params: Dict
) -> Path:
    output_path = rig_params.get("output_path", "slip_stick_rig.urdf")
    foot_offset     = rig_params.get("foot_offset")
    foot_size       = rig_params.get("foot_size")
    toe_size        = rig_params.get("toe_size")
    sole_protrusion = rig_params.get("sole_protrusion")
    leg_length      = rig_params.get("leg_length")
    leg_limits      = rig_params.get("leg_limits")
    ankle_limit     = rig_params.get("ankle_limit")
    mass            = rig_params.get("mass")
    hydro_hard     = rig_params.get("hard")
    hydro_soft     = rig_params.get("soft")
    hydro_medium   = rig_params.get("medium")
    hook_size       = rig_params.get("hook_size")
    out = Path(output_path)

    if len(foot_offset) != 3 or len(foot_size) != 3:
        raise ValueError("foot_offset and foot_size must be length-3 sequences [x,y,z].")
    if len(leg_limits) != 2 or len(leg_limits[0]) != 2 or len(leg_limits[1]) != 2:
        raise ValueError("leg_limits must be [[xmin,zmin],[xmax,zmax]].")
    if len(ankle_limit) != 2:
        raise ValueError("ankle_limit must be [a,b] degrees.")
    if foot_size[0] <= 2.0 * toe_size:
        raise ValueError("foot_size[0] must be > 2*toe_size so the sole has positive length.")
    if leg_length <= 0:
        raise ValueError("leg_length must be > 0.")
    if mass <= 0:
        raise ValueError("mass must be > 0.")

    foot_offset_x, foot_offset_y, foot_offset_z = foot_offset
    foot_x, foot_y, foot_z_toeheel = foot_size
    hook_x, hook_z = hook_size
    hook_y = foot_y  # hook has same y-width as foot
    hook_boxsize = (hook_x, hook_y, hook_z)

    # Foot geometry in COM frame (COM frame origin is at the foot COM)
    x_toe_center = (foot_x / 2.0) - (toe_size / 2.0)
    toeheel_size = (toe_size, foot_y, foot_z_toeheel)
    sole_size = (foot_x - 2.0 * toe_size, foot_y, foot_z_toeheel + 2.0 * sole_protrusion)

    # Foot link frame is at ankle. COM in ankle frame is -foot_offset.
    ankle_to_com = (-foot_offset_x, -foot_offset_y, -foot_offset_z)
    toe_pos = (ankle_to_com[0] + x_toe_center, ankle_to_com[1], ankle_to_com[2])
    heel_pos = (ankle_to_com[0] - x_toe_center, ankle_to_com[1], ankle_to_com[2])
    sole_pos = (ankle_to_com[0], ankle_to_com[1], ankle_to_com[2])

    hook_xpos = - x_toe_center - (toe_size + hook_x) / 2.0
    hook_zpos = (foot_z_toeheel - hook_z) / 2.0
    hook_pos = (ankle_to_com[0] + hook_xpos, ankle_to_com[1], ankle_to_com[2] + hook_zpos)

    # Leg mass/inertia derived from leg_length
    leg_mass = 0.5 * leg_length
    leg_box_size = (0.02, 0.02, leg_length)
    leg_ixx, leg_iyy, leg_izz = _box_inertia_about_com(leg_mass, leg_box_size)

    # Mass sphere size + inertia
    sphere_diameter = 0.02 + mass * 0.018
    sphere_radius = 0.5 * sphere_diameter
    mass_ixx, mass_iyy, mass_izz = _sphere_inertia_about_com(mass, sphere_radius)

    # Foot mass fixed; inertia about foot COM (independent of foot_offset)
    foot_mass = 0.075
    vol_toe = toeheel_size[0] * toeheel_size[1] * toeheel_size[2]
    vol_heel = vol_toe
    vol_sole = sole_size[0] * sole_size[1] * sole_size[2]
    vol_sum = vol_toe + vol_heel + vol_sole
    m_toe = foot_mass * (vol_toe / vol_sum)
    m_heel = foot_mass * (vol_heel / vol_sum)
    m_sole = foot_mass * (vol_sole / vol_sum)

    toe_ixx, toe_iyy, toe_izz = _box_inertia_about_com(m_toe, toeheel_size)
    heel_ixx, heel_iyy, heel_izz = _box_inertia_about_com(m_heel, toeheel_size)
    sole_ixx, sole_iyy, sole_izz = _box_inertia_about_com(m_sole, sole_size)

    # Parallel axis: shift toe/heel from their centers to the foot COM (at x=0 in COM frame)
    # Toe center at +x_toe_center, Heel center at -x_toe_center
    toe_ixx_s = toe_ixx + m_toe * (0.0**2 + 0.0**2)
    toe_iyy_s = toe_iyy + m_toe * (x_toe_center**2 + 0.0**2)
    toe_izz_s = toe_izz + m_toe * (x_toe_center**2 + 0.0**2)

    heel_ixx_s = heel_ixx + m_heel * (0.0**2 + 0.0**2)
    heel_iyy_s = heel_iyy + m_heel * (x_toe_center**2 + 0.0**2)
    heel_izz_s = heel_izz + m_heel * (x_toe_center**2 + 0.0**2)

    foot_ixx = toe_ixx_s + heel_ixx_s + sole_ixx
    foot_iyy = toe_iyy_s + heel_iyy_s + sole_iyy
    foot_izz = toe_izz_s + heel_izz_s + sole_izz

    # Leg limits (z sign flip per your convention)
    xmin, zmin_mag = leg_limits[0]
    xmax, zmax_mag = leg_limits[1]
    z_lower = -zmax_mag
    z_upper = -zmin_mag

    ankle_lower = _deg_to_rad(float(ankle_limit[0]))
    ankle_upper = _deg_to_rad(float(ankle_limit[1]))

    ET.register_namespace("drake", "http://drake.mit.edu")
    robot = ET.Element("robot", name="slip_stick_rig")

    _add_material(robot, "leg_red", RGBA_LEG)
    _add_material(robot, "toe_heel_blue", RGBA_TOE_HEEL)
    _add_material(robot, "sole_green", RGBA_SOLE)
    _add_material(robot, "mass_gray", RGBA_MASS)

    ET.SubElement(robot, "link", name="world")
    ET.SubElement(robot, "link", name="x_stage")
    ET.SubElement(robot, "link", name="z_stage")

    jx = ET.SubElement(robot, "joint", name="x_prismatic", type="prismatic")
    ET.SubElement(jx, "parent", link="world")
    ET.SubElement(jx, "child", link="x_stage")
    ET.SubElement(jx, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(jx, "axis", xyz="1 0 0")
    ET.SubElement(jx, "limit", lower=f"{xmin}", upper=f"{xmax}", effort="1e6", velocity="5.0")

    jz = ET.SubElement(robot, "joint", name="z_prismatic", type="prismatic")
    ET.SubElement(jz, "parent", link="x_stage")
    ET.SubElement(jz, "child", link="z_stage")
    ET.SubElement(jz, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(jz, "axis", xyz="0 0 1")
    ET.SubElement(jz, "limit", lower=f"{z_lower}", upper=f"{z_upper}", effort="1e6", velocity="5.0")

    # Leg link (mass & inertia ONLY from leg geometry)
    leg = ET.SubElement(robot, "link", name="leg")
    inert = ET.SubElement(leg, "inertial")
    ET.SubElement(inert, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inert, "mass", value=f"{leg_mass}")
    ET.SubElement(
        inert, "inertia",
        ixx=f"{leg_ixx}", ixy="0", ixz="0",
        iyy=f"{leg_iyy}", iyz="0",
        izz=f"{leg_izz}"
    )

    _add_visual_box(
        link=leg,
        size_xyz=leg_box_size,
        xyz=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
        material_name="leg_red"
    )
    _add_visual_cylinder(
        link=leg,
        radius=0.01,
        length=0.02,
        xyz=(0.0, 0.0, -leg_length / 2.0),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material_name="leg_red"
    )

    # Place leg so ankle is at world origin when q=0
    j_leg = ET.SubElement(robot, "joint", name="z_to_leg_fixed", type="fixed")
    ET.SubElement(j_leg, "parent", link="z_stage")
    ET.SubElement(j_leg, "child", link="leg")
    ET.SubElement(j_leg, "origin", xyz=f"0 0 {leg_length/2.0}", rpy="0 0 0")

    # Mass sphere link, welded to top of leg
    mass_link = ET.SubElement(robot, "link", name="mass")
    minert = ET.SubElement(mass_link, "inertial")
    ET.SubElement(minert, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(minert, "mass", value=f"{mass}")
    ET.SubElement(
        minert, "inertia",
        ixx=f"{mass_ixx}", ixy="0", ixz="0",
        iyy=f"{mass_iyy}", iyz="0",
        izz=f"{mass_izz}"
    )
    _add_visual_sphere(
        link=mass_link,
        radius=sphere_radius,
        xyz=(0.0, 0.0, 0.0),
        rpy=(0.0, 0.0, 0.0),
        material_name="mass_gray"
    )

    j_mass = ET.SubElement(robot, "joint", name="leg_to_mass_weld", type="fixed")
    ET.SubElement(j_mass, "parent", link="leg")
    ET.SubElement(j_mass, "child", link="mass")
    ET.SubElement(j_mass, "origin", xyz=f"0 0 {leg_length/2.0}", rpy="0 0 0")

    # Foot link
    foot = ET.SubElement(robot, "link", name="foot")
    finert = ET.SubElement(foot, "inertial")
    ET.SubElement(finert, "origin", xyz=f"{-foot_offset_x} {-foot_offset_y} {-foot_offset_z}", rpy="0 0 0")
    ET.SubElement(finert, "mass", value=f"{foot_mass}")
    ET.SubElement(
        finert, "inertia",
        ixx=f"{foot_ixx}", ixy="0", ixz="0",
        iyy=f"{foot_iyy}", iyz="0",
        izz=f"{foot_izz}"
    )

    _add_visual_box(foot, toeheel_size, toe_pos, (0.0, 0.0, 0.0), "toe_heel_blue")
    _add_collision_box_with_hydro(foot, toeheel_size, toe_pos, (0.0, 0.0, 0.0), hydro_hard)

    _add_visual_box(foot, toeheel_size, heel_pos, (0.0, 0.0, 0.0), "toe_heel_blue")
    _add_collision_box_with_hydro(foot, toeheel_size, heel_pos, (0.0, 0.0, 0.0), hydro_hard)

    _add_visual_box(foot, hook_boxsize, hook_pos, (0.0, 0.0, 0.0), "sole_green")
    _add_collision_box_with_hydro(foot, hook_boxsize, hook_pos, (0.0, 0.0, 0.0), hydro_medium)

    _add_visual_box(foot, sole_size, sole_pos, (0.0, 0.0, 0.0), "sole_green")
    _add_collision_box_with_hydro(foot, sole_size, sole_pos, (0.0, 0.0, 0.0), hydro_soft)

    # Ankle joint about y at the bottom of the leg
    ankle = ET.SubElement(robot, "joint", name="ankle_y", type="revolute")
    ET.SubElement(ankle, "parent", link="leg")
    ET.SubElement(ankle, "child", link="foot")
    ET.SubElement(ankle, "origin", xyz=f"0 0 {-leg_length/2.0}", rpy="0 0 0")
    ET.SubElement(ankle, "axis", xyz="0 1 0")
    ET.SubElement(
        ankle, "limit",
        lower=f"{ankle_lower}", upper=f"{ankle_upper}",
        effort="50", velocity="10"
    )

    _prettify(robot)
    ET.ElementTree(robot).write(out, encoding="utf-8", xml_declaration=True)
    return out

if __name__ == "__main__":

    hard = HydroRigidConfig(
        mesh_resolution_hint=0.005,
        mu_static=0.3,
        mu_dynamic=0.2,
        dissipation=1.0,
    )

    medium = HydroSoftConfig(
        hydroelastic_modulus=1e5,
        mesh_resolution_hint=0.005,
        mu_static=2.0,
        mu_dynamic=1.5,
        dissipation=1.0,
    )

    soft = HydroSoftConfig(
        hydroelastic_modulus=5e6,
        mesh_resolution_hint=0.005,
        mu_static=0.9,
        mu_dynamic=0.8,
        dissipation=1.0,
    )

    rig_params = {
        "output_path": "slipstick_rig.urdf",
        "foot_offset": [0.0, 0.0, 0.02],
        "foot_size": [0.12, 0.05, 0.005],
        "toe_size": 0.02,
        "sole_protrusion": 0.001,
        "leg_length": 0.60,
        "leg_limits": [[0.0, 0.0], [1.5, 0.80]],
        "ankle_limit": [-60.0, 60.0],
        "mass": 5.0,
        "soft": soft,
        "hard": hard,
        "medium": medium,
        "hook_size": [0.002, 0.01]  # x-thickness, z-height, with the y-width = foot_size[1]
    }

    write_slip_stick_urdf(rig_params)