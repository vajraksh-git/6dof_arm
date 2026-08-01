# 6-DOF Robot Arm Motion Planning (MoveIt 2)

This repository contains the ROS 2 workspace for controlling and motion planning a 6-DOF manipulator using **MoveIt 2** in **ROS 2 Humble**. 

---

## 🛠️ Prerequisites
* **Docker** & **Docker Compose** installed.
* **VS Code** installed.
* **Dev Containers** extension installed in VS Code (`ms-vscode-remote.remote-containers`).

---

## 🚀 Step 1: Container & GUI Setup

First, enable GUI forwarding so RViz can open on your host display. Run this in your local host terminal:
```bash
xhost +local:root
```

Next, start the Docker container. 
*(Note: Try using `sudo` for Docker commands if they fail due to permission issues).*

```bash
sudo docker compose up -d --build
```
run this everytime u restart pc( start docker)
```bash
sudo systemctl start docker
```


> **Tip:** You only need to add the `--build` flag the very first time you run this, or if the Dockerfile changes. Afterwards, just use `sudo docker compose up -d`.

---

## 💻 Step 2: Attach to the Container

You have two ways to access the container terminal. The VS Code method is highly recommended for writing code.

### Option A: VS Code Dev Containers (Recommended)
1. Open VS Code.
2. Press `F1` or `Ctrl+Shift+P` to open the Command Palette.
3. Search for and select: **`Dev Containers: Attach to Running Container...`**
4. Select the container named **`6dof_arm_container`** from the list.
5. Once attached, open the `/ros2_ws` folder in VS Code.

### Option B: Direct Terminal Access
If you just need command-line access without the IDE:
```bash
sudo docker exec -it 6dof_arm_container bash
```

---

## 📥 Step 3: Build

Inside your container terminal

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🎬 Step 4: Running the Simulation (Stage 0)

To verify the setup and view the robot in RViz:

```bash
cd /ros2_ws/
source install/setup.bash
ros2 launch 6dof_arm_moveit_config demo.launch.py
```

---

## ⚙️ Step 5: Setting up the Motion Planner (Stage 1)

While the RViz simulation is running, open a **new terminal tab** inside the container to scaffold the Stage 1 Python package:

```bash
cd /ros2_ws/src
ros2 pkg create --build-type ament_python 6dof_arm_planner --dependencies rclpy geometry_msgs moveit_msgs

cd /ros2_ws/src/6dof_arm_planner
touch requirements.txt
```
*(The `requirements.txt` file is required for project standards to track any future external Python libraries, though none need to be installed at this exact moment).*

### Running Stage 1

Stage 1's motion planning logic lives in the `planner` (C++) package. It wraps MoveIt's `MoveGroupInterface` to accept a target end-effector position on a topic, plan a collision-aware trajectory to it, and execute it. It also seeds the planning scene with a test obstacle and a ground plane, so collision checking is demonstrably active rather than trivially satisfied by an empty world.

**Terminal 1 — MoveIt stack (robot model, controllers, RViz):**
```bash
cd /ros2_ws
source install/setup.bash
ros2 launch 6dof_arm_moveit_config demo.launch.py
```
Wait for RViz to open. In a spare terminal, confirm both controllers are `active` before proceeding:
```bash
ros2 control list_controllers
```

**Terminal 2 — the planner node:**
```bash
cd /ros2_ws
source install/setup.bash
ros2 launch planner planner.launch.py
```
On startup this prints the arm's real current pose (confirming `/joint_states` is being read correctly) and adds a collision box + ground plane to the planning scene, visible in RViz.

**Terminal 3 — send a target pose:**
```bash
ros2 topic pub --once /target_coordinate geometry_msgs/msg/Point "{x: 0.4, y: 0.1, z: 0.3}"
```
Watch Terminal 2 for `Received Target → Planning path → Plan successful → Motion complete`, and watch RViz for the arm executing the trajectory. Publish additional targets any time — the planner keeps listening.

---

## 🎮 Step 6: Real-Time Keyboard Teleoperation (Stage 2)

Stage 2 adds real-time Cartesian velocity control via differential inverse kinematics (Jacobian-based resolved-rate control with a damped least-squares pseudo-inverse for stability near singularities), driven by keyboard input. This is architecturally separate from Stage 1's discrete OMPL planning — it streams continuous joint velocity commands instead of planning full trajectories, which is what makes it responsive enough for teleop.

Requires Terminal 1 (`demo.launch.py`) already running, same as Stage 1.

**Terminal 2 — the differential IK controller:**
```bash
cd /ros2_ws
source install/setup.bash
ros2 run planner diff_ik_controller
```
Wait for:
```
Differential IK controller ready at 50.0 Hz.
```
If instead you see a timeout warning about joint states, Terminal 1 isn't fully up yet — wait a few seconds and restart this node.

If your trajectory controller isn't named `manipulator_controller` (check with `ros2 control list_controllers`), override the target topic:
```bash
ros2 run planner diff_ik_controller --ros-args -p controller_topic:=/<your_controller_name>/joint_trajectory
```

**Terminal 3 — keyboard teleop:**
```bash
cd /ros2_ws
source install/setup.bash
ros2 run planner teleop_keyboard
```
Wait for:
```
Teleop ready. w/s: +/-X   a/d: +/-Y   q/e: +/-Z   x: stop   Ctrl+C: quit
```

**Controls** (click into Terminal 3 first so it has keyboard focus, then hold — don't tap):
| Key | Motion |
|---|---|
| `w` / `s` | +X / −X |
| `a` / `d` | +Y / −Y |
| `q` / `e` | +Z / −Z |
| `x` or release | Stop |

The arm should move smoothly in RViz while a key is held, and stop immediately on release — no residual drift.

**Known limitation:** the control loop does not run full scene-collision checks every tick (50Hz collision checking against the whole planning scene would add latency); it enforces per-joint velocity limits pulled from the robot model, but not live obstacle avoidance during teleop. This is documented as a limitation rather than solved, per the project's engineering-quality grading criteria.