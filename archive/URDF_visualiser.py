#!/usr/bin/env python3

import os
from pathlib import Path

from pydrake.geometry import StartMeshcat
from pydrake.visualization import ModelVisualizer


def main():
    # Start Meshcat
    meshcat = StartMeshcat()
    print("\nMeshcat URL (open in browser):")
    print(meshcat.web_url())

    # Path to the URDF (in the same directory as this script)

    parent_dir = Path(__file__).parent.parent  # Go up one level to the project root
    urdf_path = parent_dir / "slipstick_rig.urdf"

    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path.resolve()}")

    # Create the model visualizer
    visualizer = ModelVisualizer(meshcat=meshcat)

    # Add the URDF model
    visualizer.parser().AddModels(str(urdf_path))

    # Run once in test environments, interactive otherwise
    test_mode = True if "TEST_SRCDIR" in os.environ else False

    # Start visualization (blocks until you stop it in Meshcat)
    visualizer.Run(loop_once=test_mode)


if __name__ == "__main__":
    main()
