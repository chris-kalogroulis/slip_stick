# optimize.py
import os
import time
import numpy as np
from scipy.optimize import minimize
from slipstick_sim import sim_cost, update_terrain

offer_vis = False

cheeky_vis = False

count = None
start_time = None
test_params = None
max_iter = None
modifier = None
best = {"x": None, "f": np.inf}
powell = None

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
        "min_gap": None
    }

def setup_experiment(duration, speed, slope, n, top_box_x, top_box_z, random, seed):
    test_params = gen_test_params(duration, speed, slope)
    terrain_params = gen_terrain_params(n, top_box_x, top_box_z, random, seed)

    update_terrain(terrain_params)

    return test_params

def run_cost(x):
    global count 
    global start_time
    global test_params
    global max_iter
    global modifier
    global best
    global powell

    ankle_k = max(x[1], 0.1)
    ankle_d = max(x[0], 0.1)

    ankle_params = {
        "k": ankle_k, # Nm/rad
        "d": ankle_d, # Nms/rad
        "q0": 0.0,    # rad
    }

    pos, vel = sim_cost(joint_params=ankle_params, test_params=test_params, vis=cheeky_vis)

    f=pos[0]

    if powell and np.isfinite(f) and f < best["f"]:
        best["f"] = f
        best["x"] = np.array(x, copy=True)
        print("NEW BEST:", best["f"], "at", best["x"])

    count = count + 1
    end_time = time.perf_counter()
    duration = end_time - start_time
    frac_done = count/(max_iter*modifier)
    time_left = (duration/frac_done) - duration

    print(f"x: {x}    cost: {f}    %: {int(frac_done*100)} tleft: {int(time_left)}")

    return f
    
def run_optimization(x0, duration, speed, slope, n, top_box_x, top_box_z, random, seed, max_iterations, use_powell):
    global count
    global start_time
    global test_params
    global max_iter
    global modifier
    global best
    global powell
    
    best = {"x": None, "f": np.inf}
    count = 0
    modifier = 1.7
    max_iter = max_iterations
    powell = use_powell
    test_params = setup_experiment(
        duration=duration,
        speed=speed,
        slope=slope,
        n=n,
        top_box_x=top_box_x,
        top_box_z=top_box_z,
        random=random,
        seed=seed
    )

    start_time = time.perf_counter()

    if not powell:
        res = minimize(run_cost, x0, method="Nelder-Mead",
                    options={"maxiter": max_iter, "xatol": 1e-3, "fatol": 1e-6})
    else:
        bounds = [(0.1, 10000.0), (0.1, 1000.0)]  # bounds for ankle_d and ankle_k
        res = minimize(lambda x: run_cost(x),
                    x0,
                    method="Powell",     # good derivative-free baseline
                    bounds=bounds,
                    options={"maxiter": max_iter})
        print("best seen:", best["x"], best["f"])

    print("#### RESULTS ####")
    print("scipy returned:", res.x, res.fun)

    deep_dur = 1  # seconds
    freq = 1040  # Hz
    os.system('play -nq -t alsa synth {} sine {} vol 0.5'.format(deep_dur, freq))

    return res

def proceed_check():
    while True:
        # .lower() handles "Y" or "y"
        choice = input("Do you want to proceed? (y/n): ").lower().strip()
        
        if choice == 'y':
            return True
        elif choice == 'n':
            print("Operation cancelled.")
            return False
        else:
            print("Invalid input. Please enter 'y' or 'n'.")

def run_sim_vis(res):
    ankle_params = {
        "k": res.x[0], # Nm/rad
        "d": res.x[1], # Nms/rad
        "q0": 0.0,    # rad
    }

    sim_cost(joint_params=ankle_params, test_params=test_params, vis=True)

if __name__ == "__main__":
    tests = [
        {
            "x0": [5.0, 1.0],
            "duration": 1.0,
            "speed": 3.0,
            "slope": 40,
            "n": 100,
            "top_box_x": 0.0025,
            "top_box_z": 0.0025,
            "seed": 0,
            "max_iterations": 100,
        },
        {
            "x0": [25.0, 5.0],
            "duration": 1.0,
            "speed": 3.0,
            "slope": 40,
            "n": 100,
            "top_box_x": 0.0025,
            "top_box_z": 0.0025,
            "seed": 0,
            "max_iterations": 100,
        },
        {
            "x0": [100.0, 15.0],
            "duration": 1.0,
            "speed": 3.0,
            "slope": 40,
            "n": 100,
            "top_box_x": 0.0025,
            "top_box_z": 0.0025,
            "seed": 0,
            "max_iterations": 100,
        },
        {
            "x0": [5.0, 1.0],
            "duration": 1.0,
            "speed": 5.0,
            "slope": 30,
            "n": 100,
            "top_box_x": 0.0025,
            "top_box_z": 0.0025,
            "seed": 0,
            "max_iterations": 100,
        },
        {
            "x0": [25.0, 5.0],
            "duration": 1.0,
            "speed": 5.0,
            "slope": 30,
            "n": 100,
            "top_box_x": 0.0025,
            "top_box_z": 0.0025,
            "seed": 0,
            "max_iterations": 100,
        },
                {
            "x0": [100.0, 15.0],
            "duration": 1.0,
            "speed": 5.0,
            "slope": 30,
            "n": 100,
            "top_box_x": 0.0025,
            "top_box_z": 0.0025,
            "seed": 0,
            "max_iterations": 100,
        },
    ]

    # Prepare a single CSV file for this run and write header
    os.makedirs("results", exist_ok=True)
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    out_path = f"results/ssopt_{timestamp}.csv"
    f = open(out_path, "w")
    f.write("duration,speed,slope,n,top_box_x,top_box_z,random,seed,max_iterations,use_powell,ankle_k,ankle_d,cost,time_taken\n")

    for test in tests:
        res = run_optimization(
            x0=test["x0"],
            duration=test["duration"],
            speed=test["speed"],
            slope=test["slope"],
            n=test["n"],
            top_box_x=test["top_box_x"],
            top_box_z=test["top_box_z"],
            random=True,
            seed=test["seed"],
            max_iterations=test["max_iterations"],
            use_powell=False
        )
        # Write results per-test into a single CSV file created once per run
        # (file is opened below before the loop)
        f.write(f"{test['duration']},{test['speed']},{test['slope']},{test['n']},{test['top_box_x']},{test['top_box_z']},{True},{test['seed']},{test['max_iterations']},{False},{res.x[0]},{res.x[1]},{res.fun},{time.perf_counter() - start_time}\n")

    # Close CSV file
    try:
        f.close()
    except Exception:
        pass

    # Beep multiple times to indicate completion
    for _ in range(3):
        os.system('play -nq -t alsa synth 0.2 sine 880 vol 0.5')
        time.sleep(0.3)