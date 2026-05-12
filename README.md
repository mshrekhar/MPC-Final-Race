# F1TENTH Frenet MPC

Frenet-frame Model Predictive Controller for F1TENTH autonomous racing, solved in real time with [acados](https://github.com/acados/acados) SQP-RTI.

---

## Team

| Name | Email |
|------|-------|
| Milan Manoj | milanmnj@seas.upenn.edu |
| Manasi Shrekhar | mshrek@seas.upenn.edu |
| Eric Ouyang | eouyang@seas.upenn.edu |
| Jackson Wang | yxjacksn@seas.upenn.edu |

---

## Demo

[FRENET MPC ON CAR](https://youtube.com/shorts/HqMdWvJWUbM)

---

## Racing Strategy

We use a **Frenet-frame MPC** — instead of tracking `(x, y)` Cartesian waypoints, we track lateral offset `e_y` and heading error `e_ψ` from a pre-recorded centerline. This means the reference is always zero and the optimization problem stays nearly linear across the prediction horizon.

**States:** `[s, e_y, e_ψ, v, δ]` — arc-length, lateral error, heading error, speed, steering angle  
**Controls:** `[δ̇, a]` — steering rate, acceleration

The curvature `κ(s)` of the raceline is fed as a time-varying parameter at each horizon step, so the solver never needs to replan a full trajectory — it only needs to know the shape of the road ahead.

**Speed profile** is computed at startup from the centerline:
1. Pointwise limit: `min(sqrt(μg/|κ|), v_target · exp(−gain·|κ|))`
2. 4-pass Gaussian smoothing over neighbors
3. Forward/backward feasibility pass using a friction-circle acceleration budget

This gives automatic braking before corners and acceleration out of them without hand-tuning per-corner speeds.

**Solver settings:**
- Horizon: N = 30 steps × 0.08 s = **2.4 s lookahead**
- Cost: `LINEAR_LS` on `[e_y, e_ψ, v, δ, δ̇, a]`
- Soft constraints on `e_y` and `e_ψ` (slack penalty 1e3) — prevents infeasibility from localization noise
- Hard constraints on `v`, `δ`, `δ̇`
- QP solver: `PARTIAL_CONDENSING_HPIPM`
- Warm-started every step: ~2 ms solve time on Jetson Orin

---

## Obstacle Avoidance

> Currently tuned and validated at **4 m/s**. Behavior at higher speeds has not been fully tested.

**Detection:** Each LiDAR scan is compared against the static occupancy grid. Points that land on free (non-wall) cells pass a wall-proximity filter and are treated as dynamic objects. Points are clustered by proximity (0.20 m radius, minimum 5 points); the largest cluster centroid is the obstacle measurement.

**Tracking:** A 4-state Kalman filter in Frenet coordinates `[s, e_y, ṡ, ė_y]` tracks the opponent across steps and handles track wraparound. The tracker drops after 8 consecutive missed detections.

**Side selection:**
```
gap_left  = (obs_ey − margin) − track_left_bound
gap_right = track_right_bound − (obs_ey + margin)
→ pass on whichever side has more room
```
Once a side is chosen it is held for at least 0.5 s to prevent oscillation.

**Corridor constraint:** For each horizon step the obstacle's position is extrapolated using its Kalman-filter velocity. The MPC's `e_y` bounds are tightened to the chosen passing corridor, and the cost reference is shifted to the corridor center so the optimizer doesn't fight conflicting objectives. When the obstacle disappears the corridor resets and the car returns to the centerline naturally.

---

## Challenges

**Solver infeasibility in tight corridors.** When the opponent was near the centerline with little room on either side, the corridor constraints occasionally made the QP infeasible. We added a three-level fallback: (1) relax obstacle margins by 50%, (2) remove obstacle constraints entirely and reset the warm-start, (3) emergency brake. This eliminated mid-race stops.

**Curvature limit.** With the current horizon and solver settings, curvatures above roughly **κ ≈ 2.0 1/m (radius ≈ 0.50 m)** cause the SQP to hit MINSTEP or converge slowly. We worked around this by ensuring the recorded centerline avoids turns tighter than ~0.5 m radius.

**Map-based track width estimation.** Ray-casting from the centerline into the occupancy grid to compute automatic corridor bounds was more conservative than expected (~0.2 m error) due to map resolution and thick wall representations. We added a tunable `track_wall_margin` parameter to compensate.

**High-speed obstacle detection.** Above ~5 m/s, asynchronous LiDAR and pose callbacks caused occasional stale-pose map-diff checks, misclassifying wall grazes as dynamic objects. We mitigated this by saving and restoring the projection segment index around the detection step.

**Steering chatter on hardware.** The physical servo has backlash that caused oscillation at lower `w_delta_dot` values. Raising the steering rate penalty to 65 (from an initial 10) eliminated the chatter — this was the single most impactful tuning knob on hardware.

---

## Results & What We Would Improve

The car reliably laps at **5 m/s** on the race track with clean centerline tracking (steady-state `e_y` < 0.3 m). Obstacle avoidance works consistently at 4 m/s.

**What we would improve:**
- Re-generate the solver with a tighter Levenberg-Marquardt regularization schedule to push the curvature limit beyond κ = 2.0 1/m and enable higher cornering speeds
- Synchronize LiDAR and pose callbacks with a proper time-stamped buffer for more reliable high-speed obstacle detection
- Replace the fixed-gain speed profile with a lap-time-optimal raceline (e.g. minimum curvature or minimum time) computed offline

---

## Build & Run

### 1. Install acados

```bash
cd ~
git clone https://github.com/acados/acados.git
cd acados && git submodule update --recursive --init
mkdir build && cd build && cmake .. && make -j$(nproc) && make install
pip3 install ~/acados/interfaces/acados_template
export ACADOS_SOURCE_DIR=$HOME/acados   # add to ~/.bashrc
```

### 2. Clone and generate the C solver

```bash
cd ~/f1tenth_ws/src
git clone https://github.com/MLN-MNJ/F1TENTH-Frenet-MPC.git frenet_mpc
cd frenet_mpc
python3 scripts/generate_frenet_solver.py
```

> Re-run this step any time you change `scripts/generate_frenet_solver.py` (horizon, model, constraints, cost structure).

### 3. Build

```bash
cd ~/f1tenth_ws
colcon build --packages-select frenet_mpc --symlink-install
source install/setup.bash
```

### 4. Record a centerline (first time only)

Drive the car manually around the track at low speed:

```bash
ros2 run frenet_mpc record_wp
# Ctrl+C when done → saves to maps/halfmap.csv
```

Subscribes to `/pf/viz/inferred_pose` (particle filter). Each row is `x, y, yaw`. The `maps/` folder contains several CSV files from different recording sessions — they are independent of each other. Point `centerline_file` at whichever one you want to use.

### 5. Launch the EKF odometry stack

The MPC controller subscribes to `/odom_ekf`, which is produced by the [`sensors_bringup`](https://github.com/MLN-MNJ/F1TENTH-EKF/tree/main/sensors_bringup) package. It fuses LiDAR ICP odometry, VESC wheel odometry, and IMU data via `robot_localization`.

```bash
ros2 launch sensors_bringup sensors_bringup.launch.py
```

Keep the car **completely still** for ~5 seconds during gyro bias calibration before driving.

### 6. Run

```bash
ros2 launch frenet_mpc frenet_mpc.launch.py \
  centerline_file:=/home/nvidia/f1tenth_ws/src/frenet_mpc/maps/wp-neel-tuned.csv \
  target_speed:=4.0
```

If the car drives the wrong way: add `reverse_centerline:=true`

### Debug topics

```bash
ros2 topic echo /frenet_mpc/state        # s, e_y, e_ψ, v, κ, δ, _, δ_cmd, v_cmd, solve_ms, side, obs_s, obs_ey
ros2 topic echo /frenet_mpc/solve_time_ms
```

| Visualization topic | Content |
|---|---|
| `/frenet_mpc/predicted_trajectory` | MPC prediction (cyan spheres) |
| `/frenet_mpc/reference_trajectory` | Raceline reference (orange) |
| `/frenet_mpc/corridor` | Active corridor bounds (green/red lines) |
| `/frenet_mpc/obstacle` | Tracked opponent + PASS L/R/BRAKE label |

### Key tuning parameters

| Parameter | Default | Effect |
|---|---|---|
| `target_speed` | 5.0 m/s | Straight-line speed cap |
| `curvature_speed_gain` | 1.8 | Higher → slower in corners |
| `w_delta_dot` | 65 | Steering smoothness — raise if hardware chatters |
| `w_ey` | 35 | Centerline tracking tightness |
| `obs_safety_margin` | 0.15 m | Clearance around opponent |
| `track_wall_margin` | 0.30 m | Clearance from walls |
