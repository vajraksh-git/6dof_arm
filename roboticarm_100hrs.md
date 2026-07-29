# Robotic arm mission

# The Question

Modern robots have to interact with the environment, whether it be familiar, unfamilar, structured or unstructured. Safely doing so is of utmost importance. 
A robotic manipulator must reach its desired poses accurately, effectively and safely.

Your task is to design and implement a motion planning software stack that can accept an end effector pose and plan a executable joint trajectory to the point.Also implement a velocity based cartesian controller that can provide velocity commands to the end effector directly (hint: this will involve inverse kinematics).

Do not try to make new planning algorithms, Build a reusable robotics framework. Ideally, your stack should be as urdf/model agnostic as possible, meaning when the urdf is changed, code changed should be as little as possible. 


# Development Environment

To ensure reproducibility across all teams, all development **must** be performed inside the following software environment.

| Component | Version |
|-----------|---------|
| Operating System | Ubuntu 22.04 LTS |
| ROS Distribution | ROS2 Humble Hawksbill |
| Visualiser | Rviz2 |
| Build System | colcon |
| Containerization | Docker |

Each submission **must** include

- Dockerfile
- docker-compose.yml
- README.md

The project should launch on a clean Ubuntu installation using

```bash
docker compose up
```

without requiring any manual dependency installation on the host machine.


# Provided Resources
A urdf of a 6 DOF robotic arm will be provided, along with its mesh files.

It is highly recommended to use Moveit2, for its motion planning capablities and generating a config package with it.
https://moveit.picknik.ai/main/index.html

It is also recommended to not implement your own jacobian solver. Use libraries like pinocchio, KDL etc. or even Moveit for jacobian computation.


#  Platform

The challenge will be conducted using a simulated Universal robots UR7e manipulator. No perception sensors required in challenge. The arm will be of 6 degrees of freedom.
Joint limits config will be given.

# Structure

# Stage 0: 

Recommended reading

- ROS2 Fundamentals
- Nodes, Topics and Services
- Inverse kinematics and motion planning
- Forward kinematics
-	Moveit2
- Launch Files
- RViz

-get familiar with Moveit and the Moveit api. Generate a configuration package with moveit setup assistant.

# Stage 1: 

Start building the motion planning framework. 
Implementation must include
-Robot model loading
-inverse kinematics
-collision checking
-motion planning
-trajectory generation
-trajectory execution

You may use any open-source ROS2 packages where appropriate.

Validation before proceeding

-The robot loads successfully from the provided URDF.
-Joint states are published correctly.
-Forward kinematics computes valid end-effector poses.
-Inverse kinematics computes valid joint configurations.
-Collision-free trajectories are generated.
-Planned trajectories execute successfully.

# Stage 2:
The operator shall be able to command the end-effector in real time using the keyboard.

Your implementation shall include

-Keyboard Teleoperation
-Cartesian Velocity Commands
-Inverse Kinematics
-Differential inverse kinematics
-Smooth Continuous Motion

Validation before proceeding

-Keyboard inputs produce smooth end-effector motion.
-Motion is continuous without discontinuities.
-Joint velocity limits are respected.
-The controller remains stable near singular configurations.
-Motion stops immediately when operator input ceases.


# Deliverables

1.Submit a complete ROS2 workspace containing

-source code
-launch files
-configuration files
-documentation

2.Documentation

-Software Architecture
-Package Layout
-Kinematics Pipeline
-Differential IK Pipeline
-Motion Planning Pipeline
-Collision Checking
-Design Decisions
-Limitations

3.Demonstration

Demonstrate

-Loading the provided UR7e robot
-Planning to arbitrary target poses
-Executing planned trajectories
-Real-time keyboard Cartesian control
-Recording the completed session
(bonus) -Recovering from unreachable targets  (bonus)

# Constraints

-Only the provided robot description package may be used.
-The robot description may not be modified.
-Hard-coded joint configurations receive zero credit.
-Manually specified trajectories are prohibited.
-Any suitable open-source ROS2 package may be used.
-All dependencies must be installed through the submitted Docker configuration.
-The project should build successfully on a clean Ubuntu 22.04 installation using only the submitted repository.
-Every engineering decision should be justified.

# Judging

Evaluation will be based primarily on engineering quality rather than demonstration.


-Software Architecture
-Motion Planning
-Differential Inverse Kinematics
-Collision Checking
-Trajectory Execution
-Robustness
-Documentation
-Code Quality
-Reproducibility

reproducible engineering is more valuable than isolated demonstrations.
