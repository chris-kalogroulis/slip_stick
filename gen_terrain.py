from typing import Dict
import xml.etree.ElementTree as ET
import numpy as np

drake_props = {
    "hydroelastic_type": "compliant",
    "hydroelastic_modulus": "5e7",
    "hunt_crossley_dissipation": "1.0",
    "resolution_hint": "0.01",
    "mu_static": "0.9",
    "mu_dynamic": "0.8",
}

def prettify(elem, level=0):
    indent = "  "
    i = "\n" + level * indent
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + indent
        for child in elem:
            prettify(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = i
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


class URDFBuilder:
    def __init__(self, name="generated_robot", drake_props=None):
        self.DRAKE_NS = "drake"
        self.DRAKE_URI = "http://drake.mit.edu"
        ET.register_namespace(self.DRAKE_NS, self.DRAKE_URI)

        self.robot = ET.Element("robot", name=name)

        self.drake_props = drake_props or {
            "hydroelastic_type": "compliant",
            "hydroelastic_modulus": "5e7",
            "hunt_crossley_dissipation": "1.0",
            "resolution_hint": "0.01",
            "mu_static": "1.0",
            "mu_dynamic": "0.8",
        }

    def _add_drake_proximity_properties(self, collision_elem):
        drake_tag = lambda t: f"{{{self.DRAKE_URI}}}{t}"
        prox = ET.SubElement(collision_elem, drake_tag("proximity_properties"))

        hydro_type = (self.drake_props.get("hydroelastic_type") or "compliant").lower()
        if hydro_type not in ("compliant", "rigid"):
            raise ValueError("drake_props['hydroelastic_type'] must be 'compliant' or 'rigid'")
        ET.SubElement(prox, drake_tag(f"{hydro_type}_hydroelastic"))

        ET.SubElement(prox, drake_tag("hydroelastic_modulus"),
                      value=str(self.drake_props["hydroelastic_modulus"]))
        ET.SubElement(prox, drake_tag("hunt_crossley_dissipation"),
                      value=str(self.drake_props["hunt_crossley_dissipation"]))
        ET.SubElement(prox, drake_tag("resolution_hint"),
                      value=str(self.drake_props["resolution_hint"]))
        ET.SubElement(prox, drake_tag("mu_static"),
                      value=str(self.drake_props["mu_static"]))
        ET.SubElement(prox, drake_tag("mu_dynamic"),
                      value=str(self.drake_props["mu_dynamic"]))

    def add_link(self, name):
        return ET.SubElement(self.robot, "link", name=name)

    def add_box_to_link(self, link, geom_name, size, xyz, color):
        visual = ET.SubElement(link, "visual")
        ET.SubElement(visual, "origin", xyz=f"{xyz[0]} {xyz[1]} {xyz[2]}", rpy="0 0 0")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "box", size=f"{size[0]} {size[1]} {size[2]}")
        material = ET.SubElement(visual, "material", name=f"{geom_name}_mat")
        ET.SubElement(material, "color", rgba=f"{color[0]} {color[1]} {color[2]} {color[3]}")

        collision = ET.SubElement(link, "collision")
        ET.SubElement(collision, "origin", xyz=f"{xyz[0]} {xyz[1]} {xyz[2]}", rpy="0 0 0")
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "box", size=f"{size[0]} {size[1]} {size[2]}")
        self._add_drake_proximity_properties(collision)

    def set_link_inertial_box_approx(self, link, mass, size, xyz=(0, 0, 0)):
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz=f"{xyz[0]} {xyz[1]} {xyz[2]}", rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=str(mass))

        x, y, z = size
        ixx = (1/12) * mass * (y**2 + z**2)
        iyy = (1/12) * mass * (x**2 + z**2)
        izz = (1/12) * mass * (x**2 + y**2)

        ET.SubElement(inertial, "inertia",
                      ixx=str(ixx), ixy="0", ixz="0",
                      iyy=str(iyy), iyz="0",
                      izz=str(izz))

    def save(self, filename):
        prettify(self.robot)
        tree = ET.ElementTree(self.robot)
        tree.write(filename, encoding="utf-8", xml_declaration=True)


def _random_spaced_positions_1d(n, x_min, x_max, min_gap, rng, max_tries=20000):
    """
    Sample n positions in [x_min, x_max] such that any two are at least min_gap apart.
    Simple rejection sampling; good for moderate n.
    """
    if n <= 0:
        return np.array([], dtype=float)

    span = x_max - x_min
    if span < 0:
        raise ValueError("x_max must be >= x_min")
    if n == 1:
        return np.array([float(rng.uniform(x_min, x_max))], dtype=float)

    # Necessary condition (not sufficient in all packings, but a good sanity check)
    if min_gap * (n - 1) > span + 1e-12:
        raise ValueError(
            f"Cannot place n={n} points with min_gap={min_gap} inside "
            f"[{x_min}, {x_max}] (span={span}). Reduce n or min_gap."
        )

    positions = []
    tries = 0
    while len(positions) < n and tries < max_tries:
        tries += 1
        x = float(rng.uniform(x_min, x_max))
        if all(abs(x - p) >= min_gap for p in positions):
            positions.append(x)

    if len(positions) < n:
        raise RuntimeError(
            f"Failed to sample {n} non-overlapping positions after {max_tries} tries. "
            f"Try reducing n or min_gap."
        )

    return np.array(sorted(positions), dtype=float)


def generate_terrain(terrain_params: Dict):
    
    base_size = terrain_params.get("base_size", (1.5, 0.5, 0.05))
    base_color = terrain_params.get("base_color", (0.5, 0.5, 0.5, 1.0))
    top_color = terrain_params.get("top_color", (0.4, 0.4, 0.7, 1.0))
    n = terrain_params.get("n", 100)
    top_box_x = terrain_params.get("top_box_x", 0.0025)
    top_box_z = terrain_params.get("top_box_z", 0.0025)
    filename = terrain_params.get("filename", "terr_geom.urdf")
    random = terrain_params.get("random", True)
    seed = terrain_params.get("seed", 0)
    min_gap = terrain_params.get("min_gap", None)

    lr, lg, lb, la = top_color
    Lr, Lg, Lb, La = base_color
    Lx, Ly, Lz = base_size

    builder = URDFBuilder(drake_props=drake_props)
    terrain = builder.add_link("terrain")

    builder.add_box_to_link(
        link=terrain,
        geom_name="base",
        size=(Lx, Ly, Lz),
        xyz=(0, 0, 0),
        color=(Lr, Lg, Lb, La),
    )

    # Valid x-range for bump centers (stay fully on top of base)
    x_min = -Lx / 2 + top_box_x / 2
    x_max =  Lx / 2 - top_box_x / 2

    # Height placement (on top surface)
    z_center = Lz / 2 + top_box_z / 2

    if n == 1:
        x_positions = np.array([0.0], dtype=float) if not random else np.array(
            [float(np.random.default_rng(seed).uniform(x_min, x_max))], dtype=float
        )
    else:
        if random:
            rng = np.random.default_rng(seed)
            if min_gap is None:
                min_gap = top_box_x  # default: no overlap in x (just-touching allowed)
            x_positions = _random_spaced_positions_1d(
                n=n, x_min=x_min, x_max=x_max, min_gap=float(min_gap), rng=rng
            )
        else:
            x_positions = np.linspace(x_min, x_max, n)

    for i, x in enumerate(x_positions):
        builder.add_box_to_link(
            link=terrain,
            geom_name=f"bump_{i}",
            size=(top_box_x, Ly, top_box_z),
            xyz=(float(x), 0.0, z_center),
            color=(lr, lg, lb, la),
        )

    builder.set_link_inertial_box_approx(
        link=terrain,
        mass=1.0,
        size=(Lx, Ly, Lz),
        xyz=(0, 0, 0),
    )

    builder.save(filename)


if __name__ == "__main__":

    terrain_params = {
        "base_size": (1.5, 0.5, 0.05),
        "base_color": (0.5, 0.5, 0.5, 1.0),
        "top_color": (0.4, 0.4, 0.7, 1.0),
        "n": 100,
        "top_box_x": 0.0025,
        "top_box_z": 0.0025,
        "filename": "terr_geom.urdf",
        "random": True,
        "seed": 0,
        "min_gap": None,  # default to no x-overlap between bumps
    }

    generate_terrain(terrain_params)