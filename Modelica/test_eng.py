import os
import re
import subprocess
import numpy as np
import pandas as pd

# ==========================================
# 1. CONFIGURATION & SETUP
# ==========================================
WEIGHTS_FILE = "path_to_weights_training.txt"
PACKAGE_MO_FILE = "parameters.mo"
RUN_SCRIPT = "run.sh"

TARGET_REACH_THRESHOLD = 0.5     # [m]
HARD_CRASH_THRESHOLD = 1.0       # [m]
CRASH_Z_THRESHOLD = 0.1          # [m]

if os.path.exists(WEIGHTS_FILE):
    with open(WEIGHTS_FILE, "r") as f:
        theta = np.array(re.findall(r"[-+]?\d*\.\d+(?:[eE][-+]?\d+)?", f.read()), dtype=float)
else:
    raise FileNotFoundError(f"Weights file {WEIGHTS_FILE} not found!")

# ==========================================
# 2. DEFINITION OF PARAMETERS TO TEST
# ==========================================
ACTIVE_DRONES_N = [3, 5, 9]

SPAWN_DISTANCES = [
    {"name": "Close", "dist": (10.0, 15.0), "z_diff": (-5.0, 5.0), "hero_z": (2.0, 10.0)},
    {"name": "Medium", "dist": (30.0, 45.0), "z_diff": (-15.0, 15.0), "hero_z": (5.0, 25.0)},
    {"name": "Extreme", "dist": (70.0, 100.0), "z_diff": (-25.0, 30.0), "hero_z": (15.0, 50.0)}
]

VIRTUAL_SIM_TIMES = [60.0, 120.0, float('inf')]
RUNS_PER_CONFIG = 50 

rng = np.random.default_rng(42)

# ==========================================
# 3. HELPER FUNCTIONS FOR RECOMPILATION
# ==========================================
def update_and_recompile(total_n):
    print(f"\n[BUILD] Updating Modelica package with N = {total_n}...")
    
    with open(PACKAGE_MO_FILE, "r") as file:
        content = file.read()
    
    new_content, num_subs = re.subn(r"(constant\s+Integer\s+N\s*=\s*)\d+", rf"\g<1>{total_n}", content)
    
    if num_subs == 0:
        raise ValueError(f"Could not find 'N' declaration in file {PACKAGE_MO_FILE}. Check Modelica syntax.")
    
    with open(PACKAGE_MO_FILE, "w") as file:
        file.write(new_content)
        
    print(f"[BUILD] Running {RUN_SCRIPT}...")
    try:
        subprocess.run([RUN_SCRIPT], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("[BUILD] Recompilation completed successfully.\n")
    except subprocess.CalledProcessError:
        raise RuntimeError(f"Error while executing {RUN_SCRIPT}.")

# ==========================================
# 4. SIMULATION LOOP
# ==========================================
for n_obs in ACTIVE_DRONES_N:
    total_n = n_obs + 1
    update_and_recompile(total_n)
    
   
    current_n_results = []
    
    for spawn in SPAWN_DISTANCES:
        config_name = f"N_Tot:{total_n} | Dist:{spawn['name']}"
        print(f"--- Running Physical Tests: {config_name} ---")
        
        raw_run_results = []
        
        for run in range(RUNS_PER_CONFIG):
            override_lines = []
            
            current_stop_time = 6000.0 
            override_lines.append("stop_on_reach=true\n")
            
            # Setup AI weights
            for i, val in enumerate(theta):
                override_lines.append(f"drones[1].ctr.theta_weights[{i+1}]={val:.10f}\n")
            override_lines.append("drones[1].ctr.is_AI=true\n")
            override_lines.append("p.target_vr[1]=0.0\np.target_vtheta[1]=1.57\np.target_vphi[1]=0.0\n")

            # Dynamic Spawn Hero
            hero_x = rng.uniform(-20.0, 20.0)
            hero_y = rng.uniform(-20.0, 20.0)
            hero_z = rng.uniform(*spawn["hero_z"])
            
            override_lines.append(f"drones[1].plant.x_init={hero_x:.4f}\n")
            override_lines.append(f"drones[1].plant.y_init={hero_y:.4f}\n")
            override_lines.append(f"drones[1].plant.z_init={hero_z:.4f}\n")

            # Target Spawn
            t_angle = rng.uniform(0, 2 * np.pi)
            t_dist = rng.uniform(*spawn["dist"])
            target_x = hero_x + t_dist * np.cos(t_angle)
            target_y = hero_y + t_dist * np.sin(t_angle)
            target_z = max(1.0, hero_z + rng.uniform(*spawn["z_diff"]))
            
            override_lines.append(f"p.hero_target_x={target_x:.4f}\n")
            override_lines.append(f"p.hero_target_y={target_y:.4f}\n")
            override_lines.append(f"p.hero_target_z={target_z:.4f}\n")

            # Obstacles spawn
            for d in range(2, total_n + 1):
                obs_r = rng.uniform(2.0, t_dist * 0.8)
                obs_angle = rng.uniform(0, 2 * np.pi)
                override_lines.append(f"drones[{d}].ctr.is_AI=false\n")
                override_lines.append(f"drones[{d}].plant.x_init={hero_x + obs_r * np.cos(obs_angle):.4f}\n")
                override_lines.append(f"drones[{d}].plant.y_init={hero_y + obs_r * np.sin(obs_angle):.4f}\n")
                override_lines.append(f"drones[{d}].plant.z_init={rng.uniform(2.0, target_z + 5.0):.4f}\n")
                override_lines.append(f"p.target_vr[{d}]={rng.uniform(2.5, 3.0):.4f}\n")
                override_lines.append(f"p.target_vtheta[{d}]={rng.uniform(np.pi/4, 3*np.pi/4):.4f}\n")
                override_lines.append(f"p.target_vphi[{d}]={rng.uniform(-np.pi, np.pi):.4f}\n")

            override_file = f"paper_override_{run}.txt"
            res_file = f"paper_res_{run}.csv"
            with open(override_file, "w") as f:
                f.writelines(override_lines)

            # Modelica Simulation
            cmd = [
                "./MultiUAVSystem", f"-overrideFile={override_file}",
                "-startTime=0", f"-stopTime={current_stop_time}", "-stepSize=0.02",
                f"-r={res_file}", "-outputFormat=csv", "-noEventEmit",
                "-variableFilter=.*plant\\.(x|y|z)|.*ctr\\.(Trustx|Trusty|Trustz)"
            ]
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            # Single Run Analysis
            run_success = False
            run_time = float('inf')
            
            if os.path.exists(res_file):
                df = pd.read_csv(res_file)
                df = df[df["time"] > 0.05] if "time" in df.columns else df
                time_arr = df["time"].to_numpy() if "time" in df.columns else np.arange(len(df)) * 0.02
                
                hx, hy, hz = df['drones[1].plant.x'].to_numpy(), df['drones[1].plant.y'].to_numpy(), df['drones[1].plant.z'].to_numpy()
                
                crashed = False
                for j in range(2, total_n + 1):
                    dist_j = np.sqrt((df[f"drones[{j}].plant.x"] - hx)**2 + (
