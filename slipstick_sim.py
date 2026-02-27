import os
from pathlib import Path
from typing import Dict, Any, Optional, Tuple
import time

from gen_terrain import generate_terrain
from gen_slipstick_rig import (
    write_slip_stick_urdf, HydroRigidConfig, HydroSoftConfig
)

import numpy as np
import matplotlib.pyplot as plt

from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant, ContactModel
from pydrake.multibody.tree import RevoluteJoint, PrismaticJoint, RevoluteSpring, LinearSpringDamper
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem, BasicVector
from pydrake.visualization import AddDefaultVisualization
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.systems.primitives import LogVectorOutput


# -----------------------------
# Joint elements
# -----------------------------
def add_ankle_springs_and_dampers(plant: MultibodyPlant, joint_params: Dict):
    j = plant.GetJointByName("ankle_y")

    if isinstance(j, RevoluteJoint):
        j.set_default_damping(joint_params["d"])
        plant.AddForceElement(
            RevoluteSpring(
                joint=j,
                nominal_angle=joint_params["q0"],
                stiffness=joint_params["k"],
            )
        )
    else:
        raise ValueError(f"Expected 'ankle_y' to be a RevoluteJoint, but got {type(j)}")


class _BodySpeedFromState(LeafSystem):
    """
    Input:  plant state x = [q; v]
    Output: scalar speed = || v_WB || (translational, world frame) for a chosen body
    """
    def __init__(self, *, plant: MultibodyPlant, body_name: str, model_instance=None):
        super().__init__()
        self._plant = plant
        self._body = plant.GetBodyByName(body_name) if model_instance is None else plant.GetBodyByName(body_name, model_instance)

        self._nq = plant.num_positions()
        self._nv = plant.num_velocities()
        self._nx = self._nq + self._nv

        self._plant_context = plant.CreateDefaultContext()

        self.DeclareVectorInputPort("x", BasicVector(self._nx))
        self.DeclareVectorOutputPort("speed", BasicVector(1), self._CalcSpeed)

    def _CalcSpeed(self, context, output):
        x = self.get_input_port(0).Eval(context)
        q = x[:self._nq]
        v = x[self._nq:]

        self._plant.SetPositions(self._plant_context, q)
        self._plant.SetVelocities(self._plant_context, v)

        V_WB = self._plant.EvalBodySpatialVelocityInWorld(self._plant_context, self._body)
        speed = float(np.linalg.norm(V_WB.translational()))
        output.SetAtIndex(0, speed)


def add_body_speed_logger(*, builder: DiagramBuilder, plant: MultibodyPlant, body_name: str, model_instance=None):
    speed_sys = builder.AddSystem(_BodySpeedFromState(plant=plant, body_name=body_name, model_instance=model_instance))
    speed_sys.set_name(f"{body_name}_speed")

    builder.Connect(plant.get_state_output_port(), speed_sys.get_input_port(0))

    speed_logger = LogVectorOutput(speed_sys.get_output_port(0), builder)
    speed_logger.set_name(f"{body_name}_speed_logger")
    return speed_logger


def plot_body_speed_logger(*, simulator: Simulator, speed_logger, title: str, ylims=None):
    log = speed_logger.FindLog(simulator.get_context())
    t = log.sample_times()
    speed = log.data().reshape(-1)

    plt.figure()
    plt.plot(t, speed)
    plt.xlabel("time (s)")
    plt.ylabel("speed (m/s)")
    plt.title(title)
    if ylims is not None:
        plt.ylim(ylims)
    plt.grid(True)
    plt.show()


# -----------------------------
# Helpers: final pose + speed
# -----------------------------
def get_body_position_and_speed_at_end(
    *,
    plant: MultibodyPlant,
    plant_context,
    body_name: str,
) -> Tuple[np.ndarray, float]:
    """
    Returns:
      position_WB (3,) in meters
      speed (scalar) in m/s (translational speed of body origin, world frame)
    """
    body = plant.GetBodyByName(body_name)
    X_WB = plant.EvalBodyPoseInWorld(plant_context, body)
    p_WB = X_WB.translation().copy()

    V_WB = plant.EvalBodySpatialVelocityInWorld(plant_context, body)
    speed = float(np.linalg.norm(V_WB.translational()))

    return p_WB, speed


# -----------------------------
# Build scene / diagram
# -----------------------------
def create_scene(
    *,
    urdf_path: str,
    terr_path: str,
    slope: float,
    sim_time_step: float,
    joint_params: Dict,
    terrain_params: Dict,
    rig_params: Dict,
    contact_model: ContactModel,
    visualize_meshcat: bool
):
    """
    Returns:
      diagram, meshcat_or_none, mass_speed_logger_or_none
    """
    urdf_path = Path(urdf_path)
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path.resolve()}")

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)

    parser = Parser(plant)
    parser.AddModels(str(urdf_path))
    terrain = parser.AddModels(str(terr_path))[0]

    slope_rad = np.deg2rad(slope)
    foot_z_offset = rig_params["foot_offset"][2] + rig_params["foot_size"][2]/2
    foot_x_offset = rig_params["foot_size"][0]/2
    h = terrain_params["base_size"][2]/2 + terrain_params["top_box_z"]
    length = terrain_params["base_size"][0]
    d = 0.1
    x_offset = -foot_x_offset + (length * np.cos(slope_rad))/2 - h*np.sin(slope_rad) - d*np.cos(slope_rad)
    z_offset = -foot_z_offset - (length * np.sin(slope_rad))/2 - h*np.cos(slope_rad) + d*np.sin(slope_rad)
    tolerance = 1e-3
    T = RigidTransform(RollPitchYaw(0, slope_rad, 0), [x_offset-tolerance, 0, z_offset-tolerance])

    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("terrain", terrain),
        T
    )

    add_ankle_springs_and_dampers(plant, joint_params)

    plant.set_contact_model(contact_model)
    plant.Finalize()

    meshcat = None
    if visualize_meshcat:
        meshcat = StartMeshcat()
        meshcat.Delete()
        meshcat.DeleteAddedControls()
        AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Keep your logger (optional plotting later)
    mass_speed_logger = add_body_speed_logger(builder=builder, plant=plant, body_name="mass")

    diagram = builder.Build()
    return diagram, meshcat, mass_speed_logger


# -----------------------------
# Run simulation
# -----------------------------
def run_simulate(
    *,
    sim_time_step: float,
    realtime_rate: float,
    test_params: Dict,
    joint_params: Dict,
    terrain_params: Dict,
    rig_params: Dict,
    contact_model: ContactModel,
    test_mode: Optional[bool],
    visualize_meshcat: bool,
    plot_mass_speed: bool,
    update_urdf: bool = True,
):
    """
    Returns:
      (weight_position_W, end_mass_speed, simulator, diagram)
    """

    if update_urdf:
        update_terrain(terrain_params)
        update_rigURDF(rig_params)
    else:
        print("WARNING: not updating URDF or terrain - make sure they are up to date!")

    duration=test_params["duration"]
    x_q0=test_params["x_pos0"]
    x_v0=test_params["x_vel0"]
    z_q0=test_params["z_pos0"]
    z_v0=test_params["z_vel0"]
    slope=test_params["slope"]
    urdf_path = rig_params["output_path"]
    terr_path = terrain_params["filename"]

    if test_mode is None:
        test_mode = True if "TEST_SRCDIR" in os.environ else False
    if test_mode:
        duration = min(duration, 0.1)

    diagram, meshcat, mass_speed_logger = create_scene(
        urdf_path=urdf_path,
        terr_path=terr_path,
        slope=slope,
        sim_time_step=sim_time_step,
        joint_params=joint_params,
        terrain_params=terrain_params,
        rig_params=rig_params,
        contact_model=contact_model,
        visualize_meshcat=visualize_meshcat,
    )

    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_sub = diagram.GetSubsystemByName("plant")
    plant_context = plant_sub.GetMyMutableContextFromRoot(context)

    rail = plant_sub.GetJointByName("x_prismatic")
    rail.set_translation(plant_context, x_q0)
    rail.set_translation_rate(plant_context, x_v0)

    thigh = plant_sub.GetJointByName("z_prismatic")
    thigh.set_translation(plant_context, z_q0)
    thigh.set_translation_rate(plant_context, z_v0)

    simulator.Initialize()
    simulator.set_target_realtime_rate(realtime_rate)

    if visualize_meshcat and meshcat is not None:
        meshcat.StartRecording()

    time.sleep(0.5)

    simulator.AdvanceTo(duration)

    # ---- end-of-sim readout: weight pose + speed ----
    end_mass_pos, end_mass_speed = get_body_position_and_speed_at_end(
        plant=plant_sub,
        plant_context=plant_context,
        body_name="mass",
    )

    if plot_mass_speed:
        plot_body_speed_logger(
            simulator=simulator,
            speed_logger=mass_speed_logger,
            title="mass speed",
            ylims=None,
        )

    if visualize_meshcat and meshcat is not None:
        meshcat.StopRecording()
        meshcat.PublishRecording()

    return end_mass_pos, end_mass_speed, simulator, diagram


# -----------------------------
# Set Parameters
# -----------------------------
ankle_params = {
    "k": 1.0,   # Nm/rad
    "d": 0.5,     # Nms/rad
    "q0": 0.0,    # rad
}

test_params = {
    "duration": 1.0,
    "x_pos0": 0.0,
    "x_vel0": 0.0,
    "z_pos0": 0.0,
    "z_vel0": -2.0,
    "slope": 30,
}

terrain_params = {
    "base_size": (1.5, 0.5, 0.05),
    "base_color": (0.4, 0.4, 0.4, 1.0),
    "top_color": (0.3, 0.3, 0.3, 1.0),
    "n": 100,
    "top_box_x": 0.0025,
    "top_box_z": 0.0025,
    "filename": "terr_geom.urdf",
    "random": True,
    "seed": 0,
    "min_gap": None  # default to no x-overlap between bumps
}

hard = HydroRigidConfig(
    mesh_resolution_hint=0.005,
    mu_static=0.3,
    mu_dynamic=0.2,
    dissipation=1.0,
)

soft = HydroSoftConfig(
    hydroelastic_modulus=5e6,
    mesh_resolution_hint=0.005,
    mu_static=2.0,
    mu_dynamic=1.5,
    dissipation=1.0,
)

rig_params = {
    "output_path": "slipstick_rig.urdf",
    "foot_offset": [0.0, 0.0, 0.02],
    "foot_size": [0.15, 0.05, 0.005],
    "toe_size": 0.02,
    "sole_protrusion": 0.001,
    "leg_length": 0.50,
    "leg_limits": [[0.0, 0.0], [1.2, 1.2]],
    "ankle_limit": [-60.0, 60.0],
    "mass": 5.0,
    "soft": soft,
    "hard": hard
}

def update_rigURDF(rig_params: Dict):
    write_slip_stick_urdf(rig_params)

def update_terrain(terrain_params: Dict):
    generate_terrain(terrain_params)

# -----------------------------
# Simulation cost function (for optimization)
# -----------------------------

def sim_cost(
        test_params=test_params,
        joint_params=ankle_params,
        terrain_params = terrain_params,
        rig_params = rig_params,
        vis = False,
        update_urdf=False
    ):
    end_mass_pos, end_mass_speed, simulator, diagram = run_simulate(
        sim_time_step=1e-4,
        realtime_rate=0.0,          # 0.0 = run as fast as possible (no real-time pacing)
        test_params=test_params,
        joint_params=joint_params,
        terrain_params=terrain_params,
        rig_params=rig_params,
        contact_model=ContactModel.kHydroelasticWithFallback,
        test_mode=None,
        visualize_meshcat=vis,    # <- toggle this
        plot_mass_speed=False, # <- toggle this if you still want the plot
        update_urdf=update_urdf
    )
    return [end_mass_pos, end_mass_speed]

# -----------------------------
# Run simulation in this file once
# -----------------------------
if __name__ == "__main__":
    end_mass_pos, end_mass_speed, simulator, diagram = run_simulate(
        sim_time_step=1e-4,
        realtime_rate=0.0,          # 0.0 = run as fast as possible (no real-time pacing)
        test_params=test_params,
        joint_params=ankle_params,
        terrain_params=terrain_params,
        rig_params=rig_params,
        contact_model=ContactModel.kHydroelasticWithFallback,
        test_mode=None,
        visualize_meshcat=True,    # <- toggle this
        plot_mass_speed=False, # <- toggle this if you still want the plot
    )

    print(f"[END] weight position (world) [m]: {end_mass_pos[0]}")
    print(f"[END] weight speed [m/s]: {end_mass_speed:.6f}")