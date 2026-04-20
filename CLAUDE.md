# CLAUDE.md — AGV Navigation Project

> This file is read automatically by Claude Code at the start of every session.
> Keep it up to date. Anything here is treated as authoritative context.

---

## 1. Project Overview

This is a small **differential-drive Autonomous guilde vehicle (AGV)** running
ROS 2 with SLAM and Nav2. The robot uses an ESP32 as the low-level motor/sensor
controller and a PC (Ubuntu 24.04 + ROS 2 Jazzy) as the high-level compute node.
Communication between the two is over Wi-Fi using custom UDP packets for
odometry and motor commands, plus a TCP bridge for LiDAR data.

The high-level data flow is:

```
[Encoders + Motors]  ←→  [ESP32]  ←UDP/TCP→  [PC: ROS 2 Jazzy]  ←→  [RPLIDAR A1M8]
                                                      │
                                                      ├── slam_toolbox (mapping mode)
                                                      ├── AMCL          (localization mode)
                                                      └── Nav2          (autonomous navigation)
```

The ESP32 is responsible for closed-loop PI speed control and raw encoder
integration. The PC is responsible for everything above that: odometry
computation, SLAM, localization, planning, and control commands.

---

## 2. Software Stack

| Component           | Version / Choice                    |
|---------------------|-------------------------------------|
| OS                  | Ubuntu 24.04 LTS                    |
| ROS 2 distribution  | **Jazzy Jalisco**                   |
| Simulator           | **Gazebo Harmonic**                 |
| SLAM                | `slam_toolbox` (online async)       |
| Localization        | `nav2_amcl` (DifferentialMotionModel) |
| Navigation          | `nav2` full stack                   |
| LiDAR               | RPLIDAR A1M8 via TCP bridge         |
| MCU firmware        | Arduino / ESP32, custom PI control  |
| Primary language    | Python (ROS 2 nodes), C++ (firmware)|

> Note: earlier project docs mention ROS 2 Kilted + Gazebo Ionic. The actual
> deployment target is **Jazzy + Harmonic**. Ignore references to Kilted/Ionic
> in `project_plan.pdf`.

---

## 3. Robot Geometry — Critical Constants

These constants appear in **three** places and must stay identical everywhere.
A mismatch silently corrupts odometry, which then silently corrupts SLAM.

```
WHEEL_BASE   = 0.285   m   (distance between wheel centers)
WHEEL_RADIUS = 0.0319  m
TICKS_PER_REV = 330.0      (encoder ticks per full wheel revolution)
```

Where they live:

1. `esp32_firmware/` → constants `WHEEL_BASE_M`, `WHEEL_RADIUS_M`, `TICKS_PER_REV`
2. `udp_bridge_node.py` → ROS parameters `wheel_base`, `wheel_radius`, `ticks_per_rev`
3. All launch files that spawn `udp_bridge_node` pass these as parameters

**Rule:** If you change one, you must change them all in the same commit.

---

## 4. TF Tree and Ownership

The TF tree is:

```
map → odom → base_link → laser
                      └→ imu_link  (reserved, not currently used)
base_footprint → base_link
```

Each transform has exactly **one** publisher. Do not add second publishers —
ROS 2 will accept them silently but the result is undefined behaviour.

| Transform             | Published by                                        |
|-----------------------|-----------------------------------------------------|
| `map → odom`          | `slam_toolbox` (SLAM mode) **or** `amcl` (nav mode) |
| `odom → base_link`    | `udp_bridge_node.py` (parameter `publish_tf:=True`) |
| `base_link → laser`   | Static TF publisher in each bringup launch file     |
| `base_footprint → base_link` | Robot state publisher from `agv.urdf`         |

When running SLAM and Nav2 at the same time, **only `slam_toolbox` publishes
`map → odom`**. Do not launch `amcl` simultaneously or the TF tree becomes
inconsistent and navigation collapses.

---

## 5. ESP32 ↔ PC Protocol

This is a **custom protocol** — it is not a ROS-native bridge. Any change
on one side must be mirrored on the other.

### Ports

| Port  | Direction   | Transport | Purpose                               |
|-------|-------------|-----------|---------------------------------------|
| 8889  | PC → ESP32  | UDP       | Motor velocity commands               |
| 8888  | ESP32 → PC  | UDP       | Odometry + debug packets              |
| 20108 | PC ↔ ESP32  | TCP       | Raw RPLIDAR serial bridge             |

### Packet formats

**Motor command (PC → ESP32, port 8889):**
```
s,<right_ticks_per_sec>,<left_ticks_per_sec>\n
```

**Odometry (ESP32 → PC, port 8888):**
```
o,<right_ticks>,<left_ticks>\n
```
Note the order: **right first, left second**. The Python bridge assumes this.

**Debug (ESP32 → PC, port 8888):**
```
d,<ms>,<tR>,<tL>,<dR>,<dL>,<tgtR>,<tgtL>,<pwmR>,<pwmL>,<x>,<y>,<th>\n
```
Rate-limited to roughly 2 Hz printed on the Python side.

### Encoder channel mapping (subtle!)

The ESP32 firmware labels its two encoder channels **A** and **B**, not L and R.
The mapping is controlled by the `A_IS_RIGHT` macro in the firmware:

- Current setting: `A_IS_RIGHT = 0` → **A = LEFT, B = RIGHT**
- The firmware converts A/B → R/L internally before sending the `o,` packet
- The Python bridge only ever sees right-then-left; it does not need to know
  about A/B

If motor direction or encoder signs ever look reversed, the culprit is almost
always one of these firmware macros: `A_IS_RIGHT`, `INVERT_TICKS_A`,
`INVERT_TICKS_B`.


---

## 6. ROS 2 Package Layout

The main workspace package is `agv_base_controller` under `~/ros2_ws/src/`.
Its expected share layout is:

```
agv_base_controller/
├── agv_base_controller/          # Python nodes
│   └── udp_bridge_node.py
├── config/
│   ├── nav2_params.yaml
│   ├── nav2_view.rviz
│   └── slam_toolbox_params.yaml
├── launch/
│   ├── slam_bringup.launch.py    # mapping
│   ├── nav2_bringup.launch.py    # nav only (assumes map exists)
│   └── agv_navigation.launch.py  # nav + RViz (one-shot launcher)
├── urdf/
│   └── agv.urdf
├── package.xml
└── setup.py
```

Maps are saved to `~/ros2_ws/maps/` by convention (not versioned).

---

## 7. Common Commands

**Build (from workspace root):**
```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select agv_base_controller
source /opt/ros/jazzy/setup.bash
```

**Map a new environment:**
```bash
ros2 launch agv_base_controller slam_bringup.launch.py esp32_ip:=<ip>
# ... drive around with teleop ...
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_map
```

**Navigate on an existing map (terminal):**
```bash
ros2 launch agv_base_controller nav2_bringup.launch.py \
    esp32_ip:=<ip> map:=$HOME/ros2_ws/maps/my_map.yaml
```

**Navigate with RViz (one-click):**
```bash
./start_agv.sh            # from the project root
```

**Teleop for mapping:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

---

## 8. Coding Conventions

The Python code follows standard `rclpy` patterns. Nodes inherit from
`rclpy.node.Node`, use `self.declare_parameter(...)` for all tunables rather
than hard-coding values, and use timers rather than blocking loops. Logging
should go through `self.get_logger()` and not `print()`, because print
statements do not appear in `ros2 run` output reliably.

For YAML parameter files, keep the top-level key equal to the node name
(e.g. `slam_toolbox:` for the `slam_toolbox` node), then `ros__parameters:`
underneath. Nav2 nested nodes like `local_costmap` use a double-nested
structure (`local_costmap: { local_costmap: { ros__parameters: ... } }`) —
this is not a typo, it is required by Nav2.

Lifecycle nodes (`map_server`, `amcl`, `slam_toolbox`) are NOT automatically
active. They must be transitioned `configure` → `activate` either by
`nav2_lifecycle_manager` or by the manual `EmitEvent` chains already written
into the launch files. If a lifecycle node seems "silent" after launch,
check its state with `ros2 lifecycle get /<node_name>` before assuming it
is broken.

---

## 9. Known Issues / Gotchas

**URDF file is malformed.** `agv.urdf` currently has the `base_link`
declaration self-closed (`<link name="base_link"/>`) with the `<visual>`
block hanging outside it. This needs to be rewritten as a proper nested
block. Robot state publisher will fail to load the visual mesh until this
is fixed. The joint tree itself still works because TF only needs the
joint declarations.

**Nav2 params file has no `map_server` yaml_filename.** This is expected —
the filename is injected at launch time via the `map:=` argument. Do not
hard-code a path in `nav2_params.yaml`.

**Handshake packet.** `udp_bridge_node.py` sends `s,0,0\n` once on startup
so the ESP32 learns the PC's IP even before any real command is sent.
Don't remove this — without it the ESP32 will not know where to send
odometry packets if the Python side restarts.

**Encoder sign conventions.** If the robot drives forward but odometry
shows it going backward, do NOT fix it by negating values in
`udp_bridge_node.py`. Fix it in the firmware with `INVERT_TICKS_A` /
`INVERT_TICKS_B`. Keeping the sign logic in one place (firmware) prevents
drift between the two sides.

**Command timeout.** The firmware has a `COMMAND_TIMEOUT_MS = 800` safety
cutoff. If the Python bridge stops sending `s,...` packets for more than
800 ms, the motors stop. This is intentional — do not disable it.

---

## 10. Rules for Editing This Project

When modifying this project, please follow these rules:

1. **Git commit before any multi-file refactor.** Claude Code checkpoints
   are helpful but are no substitute for real version control, especially
   when touching launch files or firmware.

2. **Never modify the ESP32 odom packet format without updating
   `udp_bridge_node.py` in the same change.** Format is `o,<R>,<L>\n` —
   if this ever changes, search for `process_o_packet` on the Python side.

3. **Never modify the cmd_vel → wheel conversion without updating both
   sides.** The Python bridge computes ticks/s from `v`, `ω`, wheel base,
   wheel radius, and ticks-per-rev; the firmware assumes it receives
   ticks/s directly. Any change to units on one side requires the other.

4. **When tuning Nav2, change one parameter at a time.** DWB and costmap
   parameters interact in non-obvious ways; batched edits make regressions
   very hard to localize.

5. **For ROS 2 Jazzy documentation, use `docs.ros.org/en/jazzy` — not
   Kilted, not Humble.** The API is similar but parameter names and
   lifecycle semantics differ in subtle ways.

6. **Do not add a second publisher for `odom → base_link` or `map → odom`.**
   See section 4.

---

## 11. Open Work Items

- [x] Add covariance matrices to the odometry message — done in
      `udp_bridge_node.py:publish_odom`. Diagonal pose/twist covariances
      declared as ROS parameters (`pose_cov_xy`, `pose_cov_yaw`,
      `twist_cov_linear`, `twist_cov_angular`) so they can be tuned without
      touching the source. Non-planar DOFs (z, roll, pitch) set to 1e6.

- [x] Write a Gazebo Harmonic simulation of the robot for offline testing —
      done. New files: `urdf/agv_sim.urdf` (physics + Gazebo plugins) and
      `launch/gazebo_sim.launch.py`. Exposes identical ROS interfaces to the
      real robot (/cmd_vel, /odom, /scan, /imu/data, /tf).