# F1TENTH Frenet MPC — Race Report

A Frenet-frame Model Predictive Controller for F1TENTH autonomous racing, solved in real time with [acados](https://github.com/acados/acados) SQP-RTI.

---

## Setup

### 1. Install acados

```bash
cd ~
git clone https://github.com/acados/acados.git
cd acados && git submodule update --recursive --init
mkdir build && cd build && cmake .. && make -j$(nproc) && make install
pip3 install ~/acados/interfaces/acados_template
export ACADOS_SOURCE_DIR=$HOME/acados   # add to ~/.bashrc
```

### 2. Generate the C solver (run once, or after changing the problem)

```bash
cd ~/f1tenth_ws/src/frenet_mpc
rm -rf c_generated_code_frenet
python3 scripts/generate_frenet_solver.py
```

### 3. Build

```bash
cd ~/f1tenth_ws
colcon build --packages-select frenet_mpc --symlink-install
source install/setup.bash
```

### 4. Run

```bash
ros2 launch frenet_mpc frenet_mpc.launch.py \
  centerline_file:=/home/nvidia/f1tenth_ws/src/frenet_mpc/maps/wp-neel-tuned.csv \
  target_speed:=4.0
```

If the car drives the wrong way, add `reverse_centerline:=true`.

---

## Capturing the Raceline

Drive the car manually around the track once (joystick, slow speed) and record the pose:

```bash
ros2 run frenet_mpc record_wp
# Ctrl+C when done → saves to maps/halfmap.csv
```

The script subscribes to `/pf/viz/inferred_pose` (particle filter) and writes a waypoint every time the car moves more than 0.1 m. Each row is `x, y, yaw`.

> **Tip:** Drive slowly and smoothly. Sharp kinks in the recorded line produce spiky curvature estimates, which cause unnecessary braking and can destabilize the solver.

The `maps/` folder contains several CSV files from different recording sessions — they are independent and unrelated to each other. The node only uses the one pointed to by `centerline_file`. The heading column in the CSV is recorded for reference but the node recomputes yaw and curvature internally from the `x, y` positions.

---

## Raceline Tracking — How the MPC Works

### Vehicle model

The controller works in **Frenet coordinates** — instead of tracking `(x, y)`, it tracks lateral offset `e_y` and heading error `e_ψ` from the nearest point on the raceline. This makes the reference always zero and the problem nearly linear.

**States:** `[s, e_y, e_ψ, v, δ]` — arc-length, lateral error, heading error, speed, steering angle  
**Controls:** `[δ̇, a]` — steering rate, acceleration

**Dynamics:**
```
ṡ     = v·cos(e_ψ) / (1 − κ·e_y)
ė_y   = v·sin(e_ψ)
ė_ψ   = (v/L)·tan(δ) − κ·ṡ
v̇     = a
δ̇_state = δ̇
```

The curvature `κ(s)` is fed as a time-varying parameter at each horizon step — the solver never needs to replan a trajectory.

### Speed profile

At startup the node computes a curvature-adaptive speed profile:

1. **Pointwise limit:** `v = min(sqrt(μg/|κ|), v_target · exp(−gain·|κ|))`
2. **Smoothing:** 4-pass weighted average over neighbors
3. **Feasibility:** backward pass (brake limit) + forward pass (friction-circle acceleration budget)

The result is a physically consistent profile that brakes before corners and accelerates out of them with no per-corner hand-tuning.

> **Known solver limit:** with the current horizon and solver settings, curvatures above roughly **κ ≈ 2.0 1/m (radius ≈ 0.50 m)** cause the SQP to struggle — either MINSTEP or slow convergence. Avoid placing the centerline through turns tighter than ~0.5 m radius, or reduce speed significantly in those sections.

### Solver

- **Horizon:** N = 30 steps × 0.08 s = **2.4 s lookahead**
- **Cost:** `LINEAR_LS` on `[e_y, e_ψ, v, δ, δ̇, a]` — exact Gauss-Newton Hessian, fast convergence
- **Constraints:** hard bounds on `v`, `δ`, `δ̇`; soft bounds on `e_y` and `e_ψ` (slack penalty `1e3`) so localization noise doesn't cause spurious infeasibility
- **QP solver:** `PARTIAL_CONDENSING_HPIPM`
- **Warm-starting:** the previous solution is shifted one step and reused — reduces solve time from ~20 ms (cold) to ~2 ms (warm)

**Infeasibility fallback (3 levels):**  
relax obstacle margins → remove obstacle constraints + reset warm-start → emergency brake

### Key cost weights

| Weight | Value | What it controls |
|--------|-------|-----------------|
| `w_ey` | 35 | Stay on the centerline |
| `w_epsi` | 15 | Keep heading aligned |
| `w_delta_dot` | 65 | Smooth steering (most important for hardware) |
| `w_v` | 3 | Track the speed profile |
| `w_a` | 0.2 | Left low — let the optimizer use acceleration freely |

---

## Obstacle Avoidance & Overtaking

> **Note:** Obstacle avoidance is currently tuned and tested at **4 m/s**. Behavior at higher speeds has not been validated.

### Detection

Every control step, LiDAR points are compared against the static occupancy grid. Any point that lands on a free (non-wall) cell and passes a wall-proximity filter is treated as a dynamic object. Points are clustered by proximity (`obs_cluster_dist = 0.20 m`, minimum `obs_min_cluster_pts = 5` points); the largest cluster centroid is used as the obstacle measurement.

### Tracking

A 4-state Kalman filter in Frenet coordinates `[s, e_y, ṡ, ė_y]` tracks the opponent across steps. The filter handles track wraparound and drops the track after 8 consecutive missed detections.

### Passing side selection

```
gap_left  = (obs_ey − margin) − track_left_bound
gap_right = track_right_bound − (obs_ey + margin)

pick whichever gap > car_width, prefer the larger one
```

Once a side is chosen, it is held for at least `obs_side_commit_time = 0.5 s` to prevent oscillation between sides.

### Corridor constraint

For each horizon step `k`, the obstacle's predicted position is extrapolated using its Kalman-filter velocity. If that position overlaps with the ego trajectory, the MPC's `e_y` bound is tightened to the chosen passing corridor. The cost reference is simultaneously shifted to the corridor center, so the optimizer doesn't fight conflicting objectives.

After the obstacle disappears (tracker goes inactive), the corridor resets and the car returns to the centerline naturally.

---

## Tuning

### Speed

Start at `target_speed = 2.0` and increase by 0.5 m/s per lap. Watch the `e_y` field in `/frenet_mpc/state` — keep it below 0.3 m at steady state.

| Parameter | Effect |
|-----------|--------|
| `curvature_speed_gain` | Higher → slower in corners. Start at 1.8 |
| `v_min_corner` | Floor speed anywhere on track |
| `max_brake_decel` | Increase to brake harder before corners |

### MPC weights

| Symptom | Fix |
|---------|-----|
| Oscillating/snaking on straights | Raise `w_delta_dot` |
| Cutting corners | Raise `w_ey` |
| Jerky steering on hardware | Raise `w_delta_dot` (was the most impactful single knob for us) |
| Doesn't track speed | Raise `w_v` |

### Debug topics

```bash
ros2 topic echo /frenet_mpc/state        # [s, e_y, e_ψ, v, κ, δ₀, _, δ_cmd, v_cmd, solve_ms, side, obs_s, obs_ey]
ros2 topic echo /frenet_mpc/solve_time_ms
```

Visualization (RViz / Foxglove):

| Topic | Content |
|-------|---------|
| `/frenet_mpc/predicted_trajectory` | MPC prediction (cyan spheres) |
| `/frenet_mpc/reference_trajectory` | Raceline reference (orange) |
| `/frenet_mpc/corridor` | Active corridor bounds (green/red lines) |
| `/frenet_mpc/obstacle` | Tracked opponent + PASS L/R label |

---

## What We Learned

**What worked:**

- **Frenet frame** — transforming the problem to curvilinear coordinates made the reference always zero and the Hessian nearly constant. This was the single best design decision.
- **Soft constraints on `e_y` / `e_ψ`** — hard bounds caused infeasibility whenever localization noise pushed the state slightly outside. Making them soft eliminated spurious emergency brakes.
- **High `w_delta_dot` (65)** — the physical servo has backlash. Heavily penalizing steering rate eliminated chatter that no amount of PID tuning on the VESC side could fix.
- **Three-level infeasibility fallback** — in tight scenarios where the car couldn't pass, it gracefully fell back to following rather than stopping mid-race.

**What didn't work as expected:**

- **Map-based track width estimation** — ray-casting from the centerline into the occupancy grid was supposed to give automatic corridor bounds. In practice, map resolution and thick wall representations made the bounds too conservative by ~0.2 m. `track_wall_margin` is a tunable workaround.
- **High-speed obstacle detection** — above ~5 m/s, asynchronous LiDAR and pose callbacks meant the map-diff check occasionally used a stale ego pose, misclassifying wall grazes as dynamic objects.
- **Tight corner convergence** — the solver cannot reliably handle curvatures above ~2.0 1/m. At those turns the SQP hits MINSTEP and the fallback logic takes over, which introduces a brief delay. The fix is to ensure the centerline avoids such tight radii.
