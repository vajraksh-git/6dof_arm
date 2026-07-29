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

## 📥 Step 3: Clone Dependencies & Build

Inside your container terminal, clone the required Universal Robots description files into your workspace and build the project:

```bash
cd /ros2_ws/src
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ur_description

cd /ros2_ws
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