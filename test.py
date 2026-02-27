from gen_slipstick_rig import (
    write_slip_stick_urdf, HydroRigidConfig, HydroSoftConfig
)

hard = HydroRigidConfig(
    mesh_resolution_hint=0.005,
    mu_static=0.9,
    mu_dynamic=0.8,
    dissipation=1.0,
)

soft = HydroSoftConfig(
    hydroelastic_modulus=5e6,
    mesh_resolution_hint=0.005,
    mu_static=0.9,
    mu_dynamic=0.8,
    dissipation=1.0,
)

write_slip_stick_urdf(
    output_path="slip_stick_rig.urdf",
    foot_offset=[0.0, 0.0, 0.02],      # p_COM->ankle
    foot_size=[0.1, 0.05, 0.005],
    toe_size=0.02,
    sole_protrusion=0.001,
    leg_length=0.30,
    leg_limits=[[0.0, 0.0], [1.5, 0.70]],
    ankle_limit=[-60.0, 60.0],
    mass=2.0,
    hydro_hard=hard,
    hydro_soft=soft,
)