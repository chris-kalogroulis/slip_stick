# List of Rig Params

## Rig Params 1
```python
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
```
### Result files with ^
- results/ssopt_20260227-154144.csv
- results/ssopt_20260227-154317.csv
- results/ssopt_20260227-154816.csv

---

## Rig Params 2
Softer, larger and more distinct sole.
```python
hard = HydroRigidConfig(
    mesh_resolution_hint=0.005,
    mu_static=0.3,
    mu_dynamic=0.2,
    dissipation=1.0,
)

soft = HydroSoftConfig(
    hydroelastic_modulus=7.5e2,
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
    "sole_protrusion": 0.003,
    "leg_length": 0.50,
    "leg_limits": [[0.0, -0.05], [1.2, 1.2]],
    "ankle_limit": [-60.0, 60.0],
    "mass": 5.0,
    "soft": soft,
    "hard": hard
}
```