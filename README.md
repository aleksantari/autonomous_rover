# autonomous_rover

Open source ROS 2 stack for the JHU Embodied AI Club focused on an autonomous rover platform.
This repository is the foundation for a **behavior-tree-based autonomy stack** that builds
from low-level rover bringup to higher-level navigation and decision-making.

![Image](https://github.com/user-attachments/assets/17efa26b-67da-4155-ac95-bbbc55936c0e)


---

## Project Status (Current State)

This repo is a **work in progress** and currently provides:

- **Rover bringup** for the Waveshare UGV stack (hardware interface + state publication).
- **A minimal Nav2 launch** for planning and control servers.
- Structure to evolve into a full behavior-tree-driven autonomy stack.

Key packages:

- `rover_bringup`: Launches the core UGV nodes, robot state publisher, and localization.
- `rover_nav`: Launches a minimal Nav2 stack (controller, planner, BT navigator, costmaps).

---

## Long-Term Goals

This repository is intended to grow into a full **behavior-tree-based autonomy stack** that supports:

- **Full navigation** (map server, localization, recovery behaviors, and planners).
- **Custom BT nodes** for rover-specific tasks (inspection, patrol, waypoint routines).
- **Simulation + hardware parity** (Gazebo/Isaac sims with matching launch flows).
- **Perception integration** (object detection, semantic mapping, and terrain analysis).
- **Higher-level autonomy** (mission planning, state estimation improvements, teleop fallback).

We welcome contributors who want to help build the system end-to-end—from perception
and planning to mission logic and deployment.

---

## Usage (Current)

### 1. Pull dependencies

```bash
vcs import src < ros2.repos
```

### 2. Build

```bash
colcon build
```

### 3. Launch rover bringup (hardware stack)

```bash
ros2 launch rover_bringup rover.launch.py
```

Optional RViz:

```bash
ros2 launch rover_bringup rover.launch.py use_rviz:=true
```

### 4. Launch Nav2 (minimal)

```bash
ros2 launch rover_nav nav.launch.py
```

> Note: The current Nav2 launch is minimal and does **not** include map server or AMCL yet.
> It assumes localization/odom is already running from bringup.

---

## Video demo

> testing commands via ros2_control.

<video src="https://github.com/user-attachments/assets/98c3db93-e9a5-4a27-add2-950f71cae3e0" width="600" controls></video>

---

## Contributing

If you're a contributor reviewing this repository:

- This is a **foundation repo** with a clear growth path toward full autonomy.
- We are actively building toward a modular behavior-tree architecture.
- Contributions to Nav2 integration, BT nodes, simulation, perception, and testing are welcome.

---

## License

TBD
