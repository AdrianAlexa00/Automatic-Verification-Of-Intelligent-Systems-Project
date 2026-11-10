
import numpy as np
import subprocess
import pandas as pd
import os
import concurrent.futures
import re

# ==========================================
# 1. SETUP & HYPERPARAMETERS
# ==========================================
WEIGHTS_OUTPUT_FILE = "AI_pilot.txt"
MODELICA_SNIPPET_FILE = "weights_training.txt"
NUM_PARAMS = 595
TOTAL_PERTURBATIONS = 32

M_G = 5.0 * 9.81           # Weight force [N] = 49.05 N
P_HOVER = 750.0            # Hovering power [W]
P_AVIONICS = 15.0          # Onboard avionics power [W]
E_TOTAL_JOULES = 1278720.0 # 6S 16000mAh battery energy [J]

HARD_CRASH_THRESHOLD = 0.6  # [m] Physical collision radius
WARNING_THRESHOLD    = 2.0  # [m] Proximity penalty radius

grid_H = 9 
grid_V = 3 
GRID_SIZE = 27 


# Load or Initialize Weights
if os.path.exists("best_theta_weights_595.txt"):
    with open("best_theta_weights_595.txt", "r") as f:
        content = f.read()
    numbers = re.findall(r"[-+]?\d*\.\d+(?:[eE][-+]?\d+)?", content)
    theta = np.array(numbers, dtype=float)
    assert len(theta) == NUM_PARAMS, f"Expected {NUM_PARAMS} weights, got {len(theta)}"
    print(f"Loaded {len(theta)} weights from best_theta_weights_595.txt.\n")
else:
    print(f"Initializing standard random weights ({NUM_PARAMS} params)...")
    np.random.seed(42)
    theta = np.random.randn(NUM_PARAMS) * 0.02

# ES + Adam Algorithm Hyperparameters
alpha = 0.015         # Learning rate
sigma = 0.020         # Perturbation standard deviation
weight_decay = 0.001  # L2 Regularization
patience = 30
epochs = 200
patience_counter = 0
n_workers = 8         # Parallel processes
N = 5                 # Number of Modelica drones
SIM_TIME = 60.0       # Simulation time (s)
MAX_SPEED_LIMIT = 3.0 # Speed limit m/s

BENCHMARK_SEEDS = [101, 202, 303, 505]
TARGET_REACHED_THRESHOLD = 0.5


# ==========================================
# 2. SCENARIO GENERATION (CURRICULUM)
# ==========================================
def get_env_config(seed, N, max_dist, max_z_diff):
    rng = np.random.default_rng(seed)
    env_lines = []
    
    # 1. TARGET AND HERO POSITION
    t_radius = rng.uniform(5.0, max_dist)
    t_angle = rng.uniform(0, 2 * np.pi)
    
    hero_target_x = t_radius * np.cos(t_angle)
    hero_target_y = t_radius * np.sin(t_angle)
    hero_target_z = rng.uniform(max(1.0, 5.0 - max_z_diff), 5.0 + max_z_diff)
    
    env_lines.append(f"p.hero_target_x={hero_target_x:.4f}\n")
    env_lines.append(f"p.hero_target_y={hero_target_y:.4f}\n")
    env_lines.append(f"p.hero_target_z={hero_target_z:.4f}\n")
    
    env_lines.append("drones[1].ctr.is_AI=true\n")
    env_lines.append("drones[1].plant.x_init=0.0\n")
    env_lines.append("drones[1].plant.y_init=0.0\n")
    env_lines.append("drones[1].plant.z_init=5.0\n")
    
    hero_start = np.array([0.0, 0.0, 5.0])
    hero_target = np.array([hero_target_x, hero_target_y, hero_target_z])

    # 2. DYNAMIC OBSTACLE DRONES (ID 2..N)
    for drone_id in range(2, N + 1):
        env_lines.append(f"drones[{drone_id}].ctr.is_AI=false\n")
        
        a_val = rng.uniform(0.25, 0.75)
        corridor_point = hero_start + a_val * (hero_target - hero_start)
        
        obs_x = corridor_point[0] + rng.uniform(-2.0, 2.0)
        obs_y = corridor_point[1] + rng.uniform(-2.0, 2.0)
        obs_z = max(1.5, corridor_point[2] + rng.uniform(-1.0, 1.0))
        
        if np.linalg.norm(np.array([obs_x, obs_y, obs_z]) - hero_start) < 3.0:
            obs_x += 3.0
            obs_y += 3.0

        # UPDATED SPEED PARAMETERS
        base_speed = rng.uniform(0.5, 1.0)
        vr_base = rng.uniform(0.3, base_speed)
        vtheta_base = np.pi / 2.0 + rng.uniform(-0.2, 0.2)
        angle_to_hero = np.arctan2(hero_start[1] - obs_y, hero_start[0] - obs_x)
        vphi_base = angle_to_hero + rng.uniform(-0.5, 0.5)

        env_lines.append(f"p.target_vr[{drone_id}]={vr_base:.4f}\n")
        env_lines.append(f"p.target_vtheta[{drone_id}]={vtheta_base:.4f}\n")
        env_lines.append(f"p.target_vphi[{drone_id}]={vphi_base:.4f}\n")
        
        env_lines.append(f"drones[{drone_id}].plant.x_init={obs_x:.4f}\n")
        env_lines.append(f"drones[{drone_id}].plant.y_init={obs_y:.4f}\n")
        env_lines.append(f"drones[{drone_id}].plant.z_init={obs_z:.4f}\n")

    return env_lines, (hero_target_x, hero_target_y, hero_target_z)


# ==========================================
# 3. WORKER SIMULATION EVALUATION
# ==========================================
def run_simulation(worker_id, perturbed_theta, env_lines, hero_target, override_filename, result_filename):
    override_lines = [f"drones[1].ctr.theta_weights[{i+1}]={val:.10f}\n" for i, val in enumerate(perturbed_theta)]
    override_lines.extend(env_lines)
        
    with open(override_filename, "w") as f:
        f.writelines(override_lines)
    
    cmd = [
        "./MultiUAVSystem", 
        f"-overrideFile={override_filename}",
        "-startTime=0",
        f"-stopTime={SIM_TIME}",
        "-stepSize=0.02",
        f"-r={result_filename}",
        "-noEventEmit",
        "-variableFilter=.*plant\.(x|y|z|Vx|Vy|Vz)|.*ctr\.(Trustx|Trusty|Trustz|grid)", 
        "-outputFormat=csv"
    ]

    stats = {
        'final_dist': 999.0,
        'final_dx': 0.0,
        'final_dy': 0.0,
        'final_dz': 0.0,
        'max_speed': 0.0,
        'avg_speed': 0.0,
        'crashed': True,
        'min_z': 0.0,
        'min_dist_other': 999.0,
        'crashes_count': 0,
        'hit_drones': [],
        'ground_crashed': True,
        'target': hero_target,
        'time_to_target': SIM_TIME,
        'energy_joules': 0.0,
        'soc_used_pct': 0.0
    }

    try:
        subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
        df = pd.read_csv(result_filename)
        df.columns = df.columns.str.strip().str.replace('"', '')

        tx, ty, tz = hero_target
        hx, hy, hz = df['drones[1].plant.x'], df['drones[1].plant.y'], df['drones[1].plant.z']
        hVx, hVy, hVz = df['drones[1].plant.Vx'], df['drones[1].plant.Vy'], df['drones[1].plant.Vz']
        time_arr = df['time'].to_numpy()
        hz_arr = hz.to_numpy()
        Tr_x = df['drones[1].ctr.Trustx'].to_numpy()
        Tr_y = df['drones[1].ctr.Trusty'].to_numpy()
        Tr_z = df['drones[1].ctr.Trustz'].to_numpy()
        hero_pos = np.column_stack((hx, hy, hz))

        # Safe extraction of Grid column (handles both 'grid' and 'drones[1].ctr.grid')
        grid_cols = [c for c in df.columns if 'grid' in c.lower()]
        if grid_cols:
            grid_col_name = grid_cols[0]
            grid_matrix = np.array(df[grid_col_name].tolist()) if df[grid_col_name].dtype == object else df[[grid_col_name]].values
            sensor_active_per_step = (grid_matrix > 0.0).any(axis=1)
        else:
            sensor_active_per_step = np.zeros(len(df), dtype=bool)

        crashes = 0
        hit_drones = []
        min_distance_global = 999.0

        # ==============================================================================
        # PROXIMITY PENALTY CALCULATION FILTERED BY GRID + COLLISION CHECK
        # ==============================================================================
        WARNING_THRESHOLD = 3.5     # Safety warning distance in meters
        HARD_CRASH_THRESHOLD = 0.6  # Physical collision distance [m]
        proximity_warning_penalty = 0.0

        for k in range(2, N + 1):
            other_pos = df[[f'drones[{k}].plant.x', f'drones[{k}].plant.y', f'drones[{k}].plant.z']].values
            rel_pos = other_pos - hero_pos
            real_dists = np.linalg.norm(rel_pos, axis=1)

            min_d = float(np.min(real_dists))
            if min_d < min_distance_global:
                min_distance_global = min_d

            # Physical impact check
            if min_d < HARD_CRASH_THRESHOLD:
                crashes += 1
                hit_drones.append(k)

            # Pre-warning penalty (triggers ONLY if obstacle is detected by grid)
            close_and_detected = sensor_active_per_step & (real_dists < WARNING_THRESHOLD)
            if np.any(close_and_detected):
                overlap = WARNING_THRESHOLD - real_dists[close_and_detected]
                proximity_warning_penalty += float(np.sum(overlap ** 2) * 20.0)

        stats['min_dist_other'] = min_distance_global
        stats['crashes_count'] = crashes
        stats['hit_drones'] = hit_drones
        stats['crashed'] = (crashes > 0)

        # Ground crash check
        crash_mask = (hz_arr <= 0.5)
        if crash_mask.any():
            stats['ground_crashed'] = True
            first_crash_idx = np.argmax(crash_mask)
            crash_time = float(time_arr[first_crash_idx])
            time_left = SIM_TIME - crash_time
            return -10000.0 - (time_left * 100.0), stats

        stats['ground_crashed'] = False

        speed_3d = np.sqrt(hVx**2 + hVy**2 + hVz**2).to_numpy()
        dist_to_target = np.sqrt((hx - tx)**2 + (hy - ty)**2 + (hz - tz)**2).to_numpy()

        hx_fin, hy_fin, hz_fin = float(hx.iloc[-1]), float(hy.iloc[-1]), float(hz.iloc[-1])
        stats['final_dist'] = float(dist_to_target[-1])
        stats['final_dx']   = hx_fin - tx
        stats['final_dy']   = hy_fin - ty
        stats['final_dz']   = hz_fin - tz
        stats['max_speed']  = float(np.max(speed_3d))
        stats['avg_speed']  = float(np.mean(speed_3d))
        stats['min_z']      = float(np.min(hz_arr))

        final_dist = float(dist_to_target[-1])
        avg_dist = float(np.mean(dist_to_target))

        target_reached_mask = dist_to_target <= TARGET_REACHED_THRESHOLD
        thrust_mag = np.sqrt(Tr_x**2 + Tr_y**2 + Tr_z**2)
        power_inst = P_HOVER * (thrust_mag / M_G)**1.5 + P_AVIONICS

        if np.any(target_reached_mask):
            first_reach_idx = np.argmax(target_reached_mask)
            time_to_target = float(time_arr[first_reach_idx])
            power_inst_mission = power_inst[:first_reach_idx + 1]
        else:
            time_to_target = float(SIM_TIME)
            power_inst_mission = power_inst

        energy_joules = float(np.sum(power_inst_mission) * 0.02)
        soc_used_pct = float((energy_joules / E_TOTAL_JOULES) * 100.0)

        stats['time_to_target'] = time_to_target
        stats['energy_joules']  = energy_joules
        stats['soc_used_pct']   = soc_used_pct

        # Fitness components calculation
        dist_penalty = (avg_dist * 100.0) + (final_dist * 400.0)
        proximity_bonus = 1000.0 / (1.0 + final_dist)
        hover_weight = np.exp(-dist_to_target / 0.5)
        hover_stop_penalty = np.mean(hover_weight * (speed_3d**2)) * 150.0
        overspeed = np.maximum(0.0, speed_3d - MAX_SPEED_LIMIT)
        overspeed_penalty = np.mean(overspeed ** 2) * 100.0
        time_penalty = time_to_target * 30.0
        energy_penalty = soc_used_pct * 20.0

        if crashes > 0:
            fitness = -(5000.0 * crashes + SIM_TIME * 30.0 + stats['final_dist'] * 100.0)
        else:
            fitness = proximity_bonus - (dist_penalty + overspeed_penalty) - hover_stop_penalty - energy_penalty - time_penalty - proximity_warning_penalty

        return fitness, stats

    except Exception as e:
        return -20000.0, stats


# ==========================================
# 4. BENCHMARK EVALUATION WITH DETAILED REPORT
# ==========================================
def evaluate_benchmark(current_theta, benchmark_seeds, N, max_dist, max_z_diff):
    total_score = 0.0
    all_stats = []
    
    for seed in benchmark_seeds:
        env_lines, hero_target = get_env_config(seed, N, max_dist=max_dist, max_z_diff=max_z_diff)
        score, stats = run_simulation(999, current_theta, env_lines, hero_target, "override_bench.txt", "res_bench.csv")
        total_score += score
        all_stats.append(stats)
        
    avg_score = total_score / len(benchmark_seeds)
    avg_final_dist = np.mean([s['final_dist'] for s in all_stats])
    avg_dx = np.mean([s['final_dx'] for s in all_stats])
    avg_dy = np.mean([s['final_dy'] for s in all_stats])
    avg_dz = np.mean([s['final_dz'] for s in all_stats])
    avg_max_speed = np.mean([s['max_speed'] for s in all_stats])
    avg_speed = np.mean([s['avg_speed'] for s in all_stats])
    avg_time_target = np.mean([s['time_to_target'] for s in all_stats])
    avg_energy_kj = np.mean([s['energy_joules'] for s in all_stats]) / 1000.0
    avg_soc_pct = np.mean([s['soc_used_pct'] for s in all_stats])
    total_crashes = sum([1 for s in all_stats if s['ground_crashed'] or s['crashes_count'] > 0])

    print(f"  ┌── [BENCHMARK STATUS - Spawn Max: {max_dist:.1f}m]")
    print(f"  │ Average Score        : {avg_score:.2f}")
    print(f"  │ Final 3D Distance    : {avg_final_dist:.2f} m")
    print(f"  │ Avg Target Offset    : ΔX: {avg_dx:+.2f}m | ΔY: {avg_dy:+.2f}m | ΔZ: {avg_dz:+.2f}m")
    print(f"  │ Avg/Max Speed        : {avg_speed:.2f} m/s / {avg_max_speed:.2f} m/s")
    print(f"  │ Avg Time to Target   : {avg_time_target:.2f} s / {SIM_TIME:.1f} s")
    print(f"  │ Consumed Energy      : {avg_energy_kj:.2f} kJ ({avg_soc_pct:.2f}% Battery)")
    print(f"  │ Crashed Drones       : {total_crashes}/{len(benchmark_seeds)}")
    print(f"  ├── [SEED BY SEED DETAIL]")
    
    for idx, (seed, s) in enumerate(zip(benchmark_seeds, all_stats)):
        t_str = f"[{s['target'][0]:.1f}, {s['target'][1]:.1f}, {s['target'][2]:.1f}]"
        # Detailed impact status string generation
        if s['ground_crashed']:
            status_str = "💥 GROUND CRASH"
        elif len(s['hit_drones']) > 0:
            drones_str = ", ".join([f"Drone {d_id}" for d_id in s['hit_drones']])
            status_str = f"💥 COLLISION -> {drones_str}"
        else:
            status_str = "✅ FLIGHT OK"

        print(f"  │  Seed {seed} | Target: {t_str} | Final Dist: {s['final_dist']:.2f}m | MinDist Obst: {s['min_dist_other']:.2f}m | Status: {status_str}")

    print(f"  └──────────────────────────────────────────────────")
    
    return avg_score, avg_final_dist, total_crashes


# ==========================================
# 5. MAIN TRAINING LOOP (ALGORITHM 1 ES)
# ==========================================
if __name__ == '__main__':
    print(f"--- PHASE 1: Free Flight Training Algorithm 1 ({NUM_PARAMS} params) ---")
    
    current_max_dist = 5.0
    current_max_z_diff = 1.0
    TARGET_REACHED_THRESHOLD = 0.5
    
    best_score, best_final_dist, _ = evaluate_benchmark(theta, BENCHMARK_SEEDS, N, max_dist=current_max_dist, max_z_diff=current_max_z_diff)
    best_theta = np.copy(theta)
    
    print("=" * 60)
    print(f" INITIAL BASELINE BENCHMARK SCORE: {best_score:.2f}")
    print("=" * 60 + "\n")
    
    m = np.zeros(NUM_PARAMS)
    v = np.zeros(NUM_PARAMS)
    beta1, beta2 = 0.9, 0.999
    epsilon_adam = 1e-8
    
    for t in range(epochs):
        gen_seed = np.random.randint(1000, 999999)
        env_lines, hero_target = get_env_config(gen_seed, N, max_dist=current_max_dist, max_z_diff=current_max_z_diff)
        
        num_pairs = TOTAL_PERTURBATIONS // 2
        epsilon_half = np.random.randn(num_pairs, NUM_PARAMS)
        epsilon_all = np.vstack([epsilon_half, -epsilon_half])
        
        perturbed_thetas = [theta + sigma * eps for eps in epsilon_all]
        results = []
        
        for batch_start in range(0, TOTAL_PERTURBATIONS, n_workers):
            batch_thetas = perturbed_thetas[batch_start:batch_start + n_workers]
            with concurrent.futures.ProcessPoolExecutor(max_workers=n_workers) as executor:
                futures = []
                for i, p_theta in enumerate(batch_thetas):
                    worker_id = batch_start + i
                    override_fn = f"override_w{worker_id}.txt"
                    result_fn = f"res_w{worker_id}.csv"
                    futures.append(executor.submit(run_simulation, worker_id, p_theta, env_lines, hero_target, override_fn, result_fn))
                results.extend([f.result() for f in futures])

        F = np.array([float(res[0]) for res in results], dtype=float)
        worker_dists = np.array([float(res[1]['final_dist']) for res in results], dtype=float)
        worker_dz = np.array([float(res[1]['final_dz']) for res in results], dtype=float)

        # Event mapping for hit drones/ground for the current generation
        crash_events = []
        for w_idx, res in enumerate(results):
            st = res[1]
            if st['ground_crashed']:
                crash_events.append(f"W{w_idx}: Ground")
            elif len(st['hit_drones']) > 0:
                crash_events.append(f"W{w_idx}: Drones{st['hit_drones']}")

        crash_report_str = ", ".join(crash_events) if crash_events else "No Collisions"

        print(f"--- Gen {t} (Spawn Max: {current_max_dist:.1f}m) ---")
        print(f"  │ Worker Scores : {np.round(F, 1)}")
        print(f"  │ Worker Dist 3D: {np.round(worker_dists, 2)} m (Avg ΔZ: {np.mean(worker_dz):+.2f}m)")
        print(f"  │ Collision Events: {crash_report_str}")

        # Fitness Shaping / Ranking
        ranks = np.argsort(np.argsort(F))
        A = (ranks / (TOTAL_PERTURBATIONS - 1)) - 0.5 
        
        # ES + Adam Gradient Update
        gradient_estimate = np.dot(A, epsilon_all) / (TOTAL_PERTURBATIONS * sigma)
        g = gradient_estimate - (weight_decay * theta)

        m = beta1 * m + (1 - beta1) * g
        v = beta2 * v + (1 - beta2) * (g ** 2)
        m_hat = m / (1 - beta1 ** (t + 1))
        v_hat = v / (1 - beta2 ** (t + 1))
        
        theta = theta + alpha * m_hat / (np.sqrt(v_hat) + epsilon_adam)
        
        # Benchmark Evaluation
        eval_score, avg_final_dist, crashes = evaluate_benchmark(theta, BENCHMARK_SEEDS, N, max_dist=current_max_dist, max_z_diff=current_max_z_diff)
        print(f"Benchmark Score: {eval_score:.2f} | Best: {best_score:.2f}")
        
        # Curriculum Learning Loop
        if avg_final_dist <= TARGET_REACHED_THRESHOLD and crashes == 0:
            if current_max_dist < 25.0:
                current_max_dist = min(25.0, current_max_dist + 2.0)
                current_max_z_diff = min(4.5, current_max_z_diff + 1.0)
                print(f"\n🎯 TARGET REACHED! (Avg dist {avg_final_dist:.2f}m <= {TARGET_REACHED_THRESHOLD}m)")
                print(f"   -> NEW LEVEL: Max Spawn Dist = {current_max_dist:.1f}m | Max Z Diff = +/- {current_max_z_diff:.1f}m\n")
                
                best_score, _, _ = evaluate_benchmark(theta, BENCHMARK_SEEDS, N, max_dist=current_max_dist, max_z_diff=current_max_z_diff)
                patience_counter = 0
                continue

        if eval_score > best_score:
            print(f"  -> NEW BEST BENCHMARK! Improved from {best_score:.2f} to {eval_score:.2f}")
            best_score = eval_score
            best_theta = np.copy(theta)
            patience_counter = 0

            np.savetxt(WEIGHTS_OUTPUT_FILE, best_theta, delimiter=",")
            np.savetxt("best_theta_weights_595.txt", best_theta, delimiter=",")
        else:
            patience_counter += 1
            print(f"  -> Patience: {patience_counter}/{patience}")

        if eval_score < -5000.0 and patience_counter >= 15:
            print("  ⚠️ Drift detected: Rollback to best weights!")
            theta = np.copy(best_theta)
            m = np.zeros(NUM_PARAMS)
            v = np.zeros(NUM_PARAMS)
            patience_counter = 0

        if patience_counter >= patience:
            print("\nPhase 1 Training Complete!")
            break

    # Final saving
    modelica_snippet = "parameter Real theta_weights[595] = {\n  " + ", ".join([f"{val:.10f}" for val in best_theta]) + "\n};"
    with open(MODELICA_SNIPPET_FILE, "w") as f:
        f.write(modelica_snippet)
