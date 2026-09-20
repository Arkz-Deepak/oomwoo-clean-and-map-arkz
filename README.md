# OOMWOO Clean-and-Map (`oomwoo_clean_and_map`)

A clean, self-contained ROS 2 Jazzy package for autonomous coverage sweep and simultaneous mapping (SLAM) of the OOMWOO robot vacuum in a square room simulation.

## Package Architecture

```
oomwoo-clean-and-map-arkz/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package dependencies (rclpy, nav_msgs, slam_toolbox, etc.)
├── README.md                   # Documentation and usage instructions
├── config/
│   └── slam_square_room.yaml   # Tuned SLAM Toolbox parameters (tight heading keyframing, chassis filter)
├── launch/
│   └── clean_and_map_square.launch.py # Single unified launch file
├── scripts/
│   ├── coverage_planner_node.py # Boustrophedon sweep planner & reactive cmd_vel executor
│   └── save_and_analyze_map.py  # Map saver and geometric coverage analyzer
├── urdf/                       # OOMWOO robot definition with corrected physics
│   ├── robot.urdf.xacro
│   ├── params.xacro
│   ├── plugins.xacro
│   ├── inertial.xacro
│   └── materials.xacro
├── worlds/
│   └── square_room.world       # 6x6m enclosed square room simulation world
└── maps/                       # Generated map outputs and analysis
    ├── square_room_map.pgm
    ├── square_room_map.yaml
    └── map_analysis.txt
```

---

## Quick Start

### 1. Build the Package
```bash
cd ~/Projects/oomwoo-clean-and-map-arkz
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### 2. Launch Clean-and-Map Simulation
```bash
# Headless mode (default, low resource consumption)
ros2 launch oomwoo_clean_and_map clean_and_map_square.launch.py

# With RViz visualization
ros2 launch oomwoo_clean_and_map clean_and_map_square.launch.py rviz:=true

# With Gazebo GUI
ros2 launch oomwoo_clean_and_map clean_and_map_square.launch.py headless:=false rviz:=true
```

### 3. Save & Analyze the SLAM Map
```bash
ros2 run oomwoo_clean_and_map save_and_analyze_map.py --duration 30.0
```

---

## Key Features & Physics Corrections

1. **TF Sim-Time Synchronization**: `robot_state_publisher` runs with `use_sim_time: True`, synchronizing transforms with `/clock` simulation time and eliminating time jumps that cause dropped laser scans and ghost walls.
2. **Anti-Slip Differential Velocity**: Joint limits and cruise speeds are calibrated (`v_cruise: 0.25 m/s`, `rotate_speed: 0.4 rad/s`) to prevent differential wheel slip in DART physics.
3. **Chassis Laser Filtering**: `min_laser_range: 0.18 m` ensures the robot's own chassis and bumper returns are not mapped as false obstacles.
4. **Smooth Rotation Keyframing**: `minimum_travel_heading: 0.2 rad` (~11.5°) ensures continuous scan matching during in-place turns.
