# frenet_mpc

Standalone Frenet-frame acados MPC package for F1TENTH racing.

## Files

```text
frenet_mpc/
├── CMakeLists.txt
├── package.xml
├── src/frenet_mpc_node.cpp
├── scripts/generate_frenet_solver.py
├── launch/frenet_mpc.launch.py
├── config/frenet_mpc.yaml
└── maps/centerline.csv
```

## Build

```bash
cd /home/nvidia/f1tenth_ws/src
# copy this folder here as /home/nvidia/f1tenth_ws/src/frenet_mpc

cd /home/nvidia/f1tenth_ws/src/frenet_mpc
rm -rf c_generated_code_frenet
python3 scripts/generate_frenet_solver.py

cd /home/nvidia/f1tenth_ws
colcon build --packages-select frenet_mpc --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch frenet_mpc frenet_mpc.launch.py \
  centerline_file:=/home/nvidia/f1tenth_ws/src/frenet_mpc/maps/centerline.csv \
  target_speed:=2.0
```

If the car drives the wrong way:

```bash
ros2 launch frenet_mpc frenet_mpc.launch.py \
  centerline_file:=/home/nvidia/f1tenth_ws/src/frenet_mpc/maps/centerline.csv \
  target_speed:=2.0 \
  reverse_centerline:=true
```

## Debug topics

```bash
ros2 topic echo /frenet_mpc/state
ros2 topic echo /frenet_mpc/solve_time_ms
ros2 topic echo /drive
```
