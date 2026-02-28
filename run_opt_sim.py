import csv
from ast import literal_eval

csv_path = "results/ssopt_20260228-140246.csv"  # <-- update this to your actual CSV file path


def gen_test_params(duration, speed, slope):
    return {
        "duration": duration,
        "x_pos0": 0.0,
        "x_vel0": 0.0,
        "z_pos0": 0.0,
        "z_vel0": -speed,
        "slope": slope,
    }


def gen_terrain_params(n, top_box_x, top_box_z, random, seed):
    return {
        "base_size": (1.5, 0.5, 0.05),
        "base_color": (0.4, 0.4, 0.4, 1.0),
        "top_color": (0.3, 0.3, 0.3, 1.0),
        "n": n,
        "top_box_x": top_box_x,
        "top_box_z": top_box_z,
        "filename": "terr_geom.urdf",
        "random": random,
        "seed": seed,
        "min_gap": None,
    }


# Read the CSV file and extract parameters and results of each test into an array of dictionaries called tests
tests = []
with open(csv_path, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        # Convert types
        try:
            duration = float(row.get("duration", 0))
            speed = float(row.get("speed", 0))
            slope = float(row.get("slope", 0))
            n = int(row.get("n", 0))
            top_box_x = float(row.get("top_box_x", 0))
            top_box_z = float(row.get("top_box_z", 0))
            random_flag = row.get("random", "False")
            if random_flag.lower() in ("true", "1", "t"):
                random_val = True
            else:
                random_val = False
            seed = int(row.get("seed", 0))

            ankle_d = float(row.get("ankle_d", 0)) if row.get("ankle_d") not in (None, "") else None
            ankle_k = float(row.get("ankle_k", 0)) if row.get("ankle_k") not in (None, "") else None
            cost = float(row.get("cost", 0)) if row.get("cost") not in (None, "") else None
            time_taken = float(row.get("time_taken", 0)) if row.get("time_taken") not in (None, "") else None

            tests.append({
                "duration": duration,
                "speed": speed,
                "slope": slope,
                "n": n,
                "top_box_x": top_box_x,
                "top_box_z": top_box_z,
                "random": random_val,
                "seed": seed,
                "x0": [ankle_d, ankle_k],
                "cost": cost,
                "time_taken": time_taken,
            })
        except Exception:
            # skip malformed rows
            continue


# Print each test's parameters and results in a readable format
for i, t in enumerate(tests):
    print(f"[{i}] duration={t['duration']}, speed={t['speed']}, slope={t['slope']}, n={t['n']}, top_box_x={t['top_box_x']}, top_box_z={t['top_box_z']}, seed={t['seed']}, x0={t['x0']}, cost={t['cost']}")


# Ask for user input to select a test for visualisation
if not tests:
    raise SystemExit(f"No tests found in {csv_path}")

while True:
    try:
        test_index = int(input(f"Enter the index of the test to visualise (0 to {len(tests)-1}): "))
        if 0 <= test_index < len(tests):
            break
        else:
            print("Index out of range, try again.")
    except ValueError:
        print("Please enter a valid integer index.")


def run_visualisation(test_index):
    test = tests[test_index]
    test_params = gen_test_params(test["duration"]*2, test["speed"], test["slope"])
    terrain_params = gen_terrain_params(test["n"], test["top_box_x"], test["top_box_z"], test["random"], test["seed"])

    # Try to import simulation helpers now so listing works without plotting deps
    try:
        from slipstick_sim import sim_cost, update_terrain
    except ModuleNotFoundError as e:
        print("Cannot run visualisation: missing dependency:", e)
        print("Install required packages (for example: pip install matplotlib) and try again.")
        return

    update_terrain(terrain_params)

    # Print the parameters of the test being visualised in a readable format
    print("Visualising test:")
    print(f"  duration={test['duration']}, speed={test['speed']}, slope={test['slope']}")
    print(f"  terrain: n={test['n']}, top_box_x={test['top_box_x']}, top_box_z={test['top_box_z']}, seed={test['seed']}")
    print(f"  optimized x0={test['x0']}, cost={test['cost']}")


    ankle_params = {
        "k": test["x0"][0], # Nm/rad
        "d": test["x0"][1], # Nms/rad
        "q0": 0.0,    # rad
    }

    sim_cost(
        joint_params=ankle_params,
        test_params=test_params,
        terrain_params=terrain_params,
        vis=True,
        update_urdf=True,
    )


run_visualisation(test_index)