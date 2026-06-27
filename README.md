# Vision-Guided Pick & Stack — Franka Panda (ROS2 Humble + MoveIt2 + Gazebo)

An autonomous pick-and-place system where a simulated Franka Panda arm detects
colored objects using an overhead RGB-D camera, computes their 3D position in
the robot's world frame, and picks one object up to stack it on another —
entirely closed-loop, with no hardcoded coordinates.

## Demo

https://youtu.be/EbJ3fBuhKmc
*(Camera detects the red and blue boxes → MoveIt2 plans and executes a pick
trajectory for the blue box → gripper grasps it → arm stacks it on top of
the red box.)*

## What this demonstrates

- **Perception**: RGB-D camera simulation, HSV color segmentation, contour-based
  centroid detection, point-cloud depth extraction
- **Coordinate transforms**: TF2 static transforms, camera-frame → world-frame
  point conversion, handling sensor-specific axis conventions
- **Motion planning**: MoveIt2 `MoveGroupInterface` (C++), waypoint-based
  trajectory execution, gripper control via joint trajectory controllers
- **System integration**: decoupled ROS2 nodes (vision and motion run
  independently), pose data passed via topics rather than hardcoded values

## Architecture

```
┌─────────────────┐     /camera/points      ┌──────────────────┐
│  Gazebo (RGBD    │────────────────────────▶│   Vision Node    │
│  camera sensor)  │     /camera/image       │   (Python)       │
└─────────────────┘                          └──────────────────┘
                                                       │
                                          /detected_red_object_pose
                                          /detected_blue_object_pose
                                                       │
                                                       ▼
┌─────────────────┐                          ┌──────────────────┐
│  Gazebo (Panda   │◀─────MoveIt2 plans──────│  Motion Planner   │
│  arm + gripper)  │      + executes         │  Node (C++)       │
└─────────────────┘                          └──────────────────┘
```

**Vision pipeline**: subscribes to the overhead camera's RGB image and point
cloud. HSV thresholding isolates red and blue objects; contour centroids give
pixel coordinates; the point cloud provides per-pixel depth, which is
transformed via TF2 into the robot's `world` frame and published as
`PoseStamped` messages on per-color topics.

**Motion pipeline**: waits for both object poses, then plans and executes a
waypoint sequence (pre-grasp → grasp → retreat → pre-place → place → retreat)
using MoveIt2's `MoveGroupInterface`, with explicit gripper open/close commands
sent to the gripper's joint trajectory controller.

## Project structure

```
panda_arm/
├── pick_place/              # C++ motion planning package
│   ├── src/motion_planner.cpp
│   ├── launch/motion_planner.launch.py
│   └── worlds/box.sdf       # Gazebo world: table, boxes, overhead camera
└── pick_place_vision/       # Python vision package
    └── pick_place_vision/box_detector.py
```

## Setup

### Prerequisites
- Ubuntu 22.04
- ROS2 Humble
- Gazebo Ignition Fortress (6.17.0)
- MoveIt2 (`ros-humble-moveit`)

### Dependencies

```bash
sudo apt install ros-humble-ros-ign-gazebo ros-humble-ros-ign-bridge \
  ros-humble-ros-ign-image ros-humble-ros-ign-interfaces \
  ros-humble-ros-ign-gazebo-demos ros-humble-moveit

pip3 install "numpy<2" "opencv-python==4.9.0.80" --user
```

### Build

```bash
mkdir -p ~/panda_ws/src
cd ~/panda_ws/src
git clone https://github.com/Gargie-kapte/panda_arm.git
git clone https://github.com/AndrejOrsula/panda_gz_moveit2.git
vcs import < panda_gz_moveit2/panda_ign_moveit2.repos

cd ~/panda_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install --symlink-install --parallel-workers 4 \
  --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

## Running

Three terminals, in order:

**Terminal 1 — Gazebo + Panda arm + MoveIt2 + RViz2:**
```bash
source ~/panda_ws/install/setup.bash
ros2 launch panda_moveit_config ex_ign_control.launch.py \
  world:=~/panda_ws/src/pick_place/worlds/box.sdf
```

**Terminal 2 — Vision pipeline (camera bridge, TF2, detection node):**
```bash
source ~/panda_ws/install/setup.bash
ros2 launch pick_place_vision vision.launch.py
```
Wait for stable `RED world frame` / `BLUE world frame` log output before
proceeding.

**Terminal 3 — Motion planner:**
```bash
source ~/panda_ws/install/setup.bash
ros2 launch pick_place motion_planner.launch.py
```
The arm will wait for both object poses, then execute the pick-and-stack
sequence automatically.

## Key implementation notes

- **RGB-D point cloud axis convention**: Ignition Fortress's `rgbd_camera`
  sensor publishes point clouds with depth along the sensor's local X axis
  rather than the ROS-standard optical Z axis. This required remapping axes
  before applying the TF2 transform.
- **Physics tunneling**: fast free-fall onto thin collision geometry caused
  spawned objects to tunnel through the table in early iterations; fixed by
  minimizing spawn-height-above-surface.
- **Execution timing**: `MoveGroupInterface::execute()` aborts intermittently
  if planning starts from a stale robot state; resolved with
  `setStartStateToCurrentState()` before every plan call.

## Status

- [x] Arm motion planning and execution (MoveIt2 + Gazebo)
- [x] Gripper control (open/close via joint trajectory controller)
- [x] RGB-D camera simulation and ROS2 bridge
- [x] Multi-color object detection (HSV + contours)
- [x] 2D pixel → 3D world-frame pose estimation (point cloud + TF2)
- [x] Closed-loop pick-and-stack using detected poses
- [ ] Multi-object sorting to distinct drop zones
- [ ] Real-time replanning / dynamic obstacle avoidance

## License

MIT
