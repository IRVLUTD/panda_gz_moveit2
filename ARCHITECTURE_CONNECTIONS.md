# Integration Architecture: Gazebo, MoveIt2, and ROS2 Control

This document explains how the different software components are connected in the `panda_gz_moveit2` project to enable seamless integration between **Gazebo simulator**, **MoveIt2 framework**, and **ROS2 control** for robotic manipulation with the Franka Emika Panda robot.

## 🏗️ Architecture Overview

The integration follows a layered architecture where each component has specific responsibilities:

- **🌍 Gazebo**: Provides physics simulation and sensor modeling
- **🔗 gz_ros2_control**: Bridge plugin connecting ROS2 control to Gazebo's physics engine
- **🎮 ROS2 control**: Unified interface for robot control with pluggable hardware backends
- **🎯 MoveIt2**: High-level motion planning, collision detection, and trajectory execution

## 🔌 Connection Flow

```
MoveIt2 Move Group
       ↓ (trajectory commands)
ROS2 Control Controllers  
       ↓ (joint commands)
gz_ros2_control Plugin
       ↓ (physics simulation)
Gazebo Simulator
```

## 📋 Detailed Component Connections

### 1. Robot Description Integration

#### URDF Configuration with ROS2 Control
The robot description is defined using xacro macros that configure ROS2 control interfaces:

**File**: [`panda_description/urdf/panda.ros2_control`](panda_description/urdf/panda.ros2_control)
```xml
<ros2_control name="panda_arm_system" type="system">
  <hardware>
    <xacro:if value="${plugin == 'gz'}">
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </xacro:if>
  </hardware>
  <joint name="${prefix}joint1">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <!-- ... additional joints ... -->
</ros2_control>
```

This configuration:
- Defines the hardware plugin (`gz_ros2_control/GazeboSimSystem`) for Gazebo integration
- Specifies command interfaces (position, velocity, effort) for each joint
- Declares state interfaces for feedback (position, velocity, effort)

#### Gazebo Plugin Configuration
**File**: [`panda_description/urdf/panda.gazebo`](panda_description/urdf/panda.gazebo)
```xml
<gazebo>
  <plugin name="gz_ros2_control::GazeboSimROS2ControlPlugin" 
          filename="libgz_ros2_control-system.so">
    <parameters>${controller_parameters}</parameters>
  </plugin>
</gazebo>
```

This plugin:
- Loads the gz_ros2_control system plugin in Gazebo
- Parses ROS2 control tags from the robot description
- Bridges Gazebo physics with ROS2 control interfaces

### 2. MoveIt2 Configuration

#### Move Group Setup
**File**: [`panda_moveit_config/launch/move_group.launch.py`](panda_moveit_config/launch/move_group.launch.py)

The main MoveIt2 node is configured with multiple parameter sets:

```python
Node(
    package="moveit_ros_move_group",
    executable="move_group",
    parameters=[
        robot_description,                    # Robot URDF
        robot_description_semantic,          # SRDF semantic description
        robot_description_kinematics,        # Kinematic solver config
        joint_limits,                        # Safety limits
        planning_pipeline,                   # Motion planning algorithms
        trajectory_execution,                # Execution parameters
        planning_scene_monitor_parameters,   # Environment monitoring
        moveit_controller_manager,           # Controller interface
    ],
)
```

#### Controller Manager Integration
**File**: [`panda_moveit_config/config/moveit_controller_manager.yaml`](panda_moveit_config/config/moveit_controller_manager.yaml)
```yaml
controller_names:
  - joint_trajectory_controller
  - gripper_trajectory_controller

joint_trajectory_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  joints:
    - panda_joint1
    - panda_joint2
    # ... all arm joints
```

This configuration tells MoveIt2:
- Which ROS2 controllers to use for trajectory execution
- The action interface type (`FollowJointTrajectory`)
- Which joints each controller manages

### 3. ROS2 Control Configuration

#### Controller Definitions
**Files**: 
- [`panda_moveit_config/config/controllers_effort.yaml`](panda_moveit_config/config/controllers_effort.yaml)
- [`panda_moveit_config/config/controllers_position.yaml`](panda_moveit_config/config/controllers_position.yaml)
- [`panda_moveit_config/config/controllers_velocity.yaml`](panda_moveit_config/config/controllers_velocity.yaml)

Example effort controller configuration:
```yaml
controller_manager:
  ros__parameters:
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - panda_joint1
      - panda_joint2
      # ... all joints
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity
    gains:
      panda_joint1:
        p: 4000.0
        d: 10.0
        i: 250.0
```

This defines:
- Controller types and their ROS2 control interfaces
- Joint mappings for each controller
- PID gains for effort control (when using effort interface)

#### Controller Spawning
The controllers are automatically spawned by the move_group launch file:

```python
for controller in moveit_controller_manager_yaml["controller_names"] + ["joint_state_broadcaster"]:
    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller],
        )
    )
```

### 4. Launch System Integration

#### Full Gazebo Simulation Launch
**File**: [`panda/launch/gz.launch.py`](panda/launch/gz.launch.py)

The complete integration is orchestrated through a launch sequence:

1. **URDF Generation**: Convert xacro to SDF for Gazebo
```python
xacro2sdf = ExecuteProcess(
    cmd=[
        "ros2", "run", description_package, "xacro2sdf.bash",
        ["ros2_control_plugin:=", "gz"],
        ["ros2_control_command_interface:=", ros2_control_command_interface],
    ]
)
```

2. **Gazebo Launch**: Start simulator with generated model
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
    ])
)
```

3. **MoveIt2 Launch**: Start motion planning with Gazebo backend
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare("panda_moveit_config"), "launch", "move_group.launch.py"
    ]),
    launch_arguments=[
        ("ros2_control_plugin", "gz"),
        ("ros2_control_command_interface", "effort"),
    ]
)
```

4. **ROS-Gazebo Bridge**: Enable communication between ROS2 and Gazebo
```python
Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
)
```

#### Alternative: Fake Controllers Launch
**File**: [`panda_moveit_config/launch/ex_fake_control.launch.py`](panda_moveit_config/launch/ex_fake_control.launch.py)

For testing without physics simulation:
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare("panda_moveit_config"), "launch", "move_group.launch.py"
    ]),
    launch_arguments=[
        ("ros2_control_plugin", "fake"),  # Use fake controllers instead of Gazebo
    ]
)
```

### 5. Runtime Data Flow

#### Motion Planning to Execution Flow

1. **User Input**: Motion planning request via RViz or API
2. **MoveIt2 Planning**: 
   - Uses OMPL planners configured in [`ompl_planning.yaml`](panda_moveit_config/config/ompl_planning.yaml)
   - Applies kinematic constraints from [`joint_limits.yaml`](panda_moveit_config/config/joint_limits.yaml)
   - Uses kinematic solver from [`kinematics.yaml`](panda_moveit_config/config/kinematics.yaml)

3. **Trajectory Generation**: MoveIt2 generates joint trajectory
4. **Controller Interface**: Trajectory sent to `joint_trajectory_controller` via action interface
5. **ROS2 Control**: Controller converts trajectory to joint commands
6. **Gazebo Integration**: gz_ros2_control plugin forwards commands to Gazebo joints
7. **Physics Simulation**: Gazebo simulates robot motion with realistic dynamics
8. **State Feedback**: Joint states flow back through the same chain for monitoring

#### Key ROS2 Topics and Services

- **Trajectory Execution**: `/joint_trajectory_controller/follow_joint_trajectory` (action)
- **Joint States**: `/joint_states` (sensor_msgs/JointState)
- **Planning Scene**: `/monitored_planning_scene` (moveit_msgs/PlanningScene)
- **Motion Planning**: `/plan_kinematic_path` (moveit_msgs/GetMotionPlan)

### 6. Configuration Parameters

#### Trajectory Execution Parameters
**File**: [`panda_moveit_config/launch/move_group.launch.py`](panda_moveit_config/launch/move_group.launch.py)
```python
trajectory_execution = {
    "allow_trajectory_execution": True,
    "moveit_manage_controllers": False,  # ROS2 control manages controllers
    "execution_duration_monitoring": False,
    "trajectory_execution.allowed_execution_duration_scaling": 3.0,
}
```

#### Planning Pipeline Configuration
```python
planning_pipeline = {
    "planning_pipelines": ["ompl"],
    "default_planning_pipeline": "ompl",
    "ompl": {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": [
            "default_planning_request_adapters/ResolveConstraintFrames",
            "default_planning_request_adapters/CheckStartStateBounds",
        ],
        "response_adapters": [
            "default_planning_response_adapters/AddTimeOptimalParameterization",
            "default_planning_response_adapters/ValidateSolution",
        ],
    },
}
```

## 🔧 Command Interface Options

The system supports multiple command interfaces through configuration:

### Effort Control (Default)
- **Command Interface**: `effort`
- **Use Case**: Most realistic physics simulation
- **Config File**: [`controllers_effort.yaml`](panda_moveit_config/config/controllers_effort.yaml)
- **Features**: PID control with tuned gains for each joint

### Position Control
- **Command Interface**: `position`
- **Use Case**: Direct position control
- **Config File**: [`controllers_position.yaml`](panda_moveit_config/config/controllers_position.yaml)
- **Features**: Immediate position setting

### Velocity Control
- **Command Interface**: `velocity`
- **Use Case**: Velocity-based control
- **Config File**: [`controllers_velocity.yaml`](panda_moveit_config/config/controllers_velocity.yaml)
- **Features**: Velocity commands with PID control

### Combined Interfaces
- **Command Interface**: `position,velocity`
- **Config File**: [`controllers_position,velocity.yaml`](panda_moveit_config/config/controllers_position,velocity.yaml)
- **Features**: Dual interface support

## 🚀 Quick Start Commands

### Full Physics Simulation
```bash
ros2 launch panda_moveit_config ex_gz_control.launch.py
```

### Motion Planning Only (No Physics)
```bash
ros2 launch panda_moveit_config ex_fake_control.launch.py
```

### Custom Configuration
```bash
ros2 launch panda_moveit_config move_group.launch.py \
    ros2_control_plugin:=gz \
    ros2_control_command_interface:=effort \
    enable_rviz:=true
```

## 📁 Key File References

### Core Integration Files
- **URDF with ROS2 Control**: [`panda_description/urdf/panda.ros2_control`](panda_description/urdf/panda.ros2_control)
- **Gazebo Plugin**: [`panda_description/urdf/panda.gazebo`](panda_description/urdf/panda.gazebo)
- **Main Robot URDF**: [`panda_description/urdf/panda.urdf.xacro`](panda_description/urdf/panda.urdf.xacro)

### MoveIt2 Configuration
- **Move Group Launch**: [`panda_moveit_config/launch/move_group.launch.py`](panda_moveit_config/launch/move_group.launch.py)
- **Controller Manager**: [`panda_moveit_config/config/moveit_controller_manager.yaml`](panda_moveit_config/config/moveit_controller_manager.yaml)
- **Kinematics**: [`panda_moveit_config/config/kinematics.yaml`](panda_moveit_config/config/kinematics.yaml)
- **Joint Limits**: [`panda_moveit_config/config/joint_limits.yaml`](panda_moveit_config/config/joint_limits.yaml)
- **OMPL Planning**: [`panda_moveit_config/config/ompl_planning.yaml`](panda_moveit_config/config/ompl_planning.yaml)

### ROS2 Control Configuration
- **Effort Controllers**: [`panda_moveit_config/config/controllers_effort.yaml`](panda_moveit_config/config/controllers_effort.yaml)
- **Position Controllers**: [`panda_moveit_config/config/controllers_position.yaml`](panda_moveit_config/config/controllers_position.yaml)
- **Velocity Controllers**: [`panda_moveit_config/config/controllers_velocity.yaml`](panda_moveit_config/config/controllers_velocity.yaml)

### Launch Files
- **Full Gazebo Integration**: [`panda/launch/gz.launch.py`](panda/launch/gz.launch.py)
- **Gazebo Example**: [`panda_moveit_config/launch/ex_gz_control.launch.py`](panda_moveit_config/launch/ex_gz_control.launch.py)
- **Fake Control Example**: [`panda_moveit_config/launch/ex_fake_control.launch.py`](panda_moveit_config/launch/ex_fake_control.launch.py)

This architecture enables seamless integration between motion planning (MoveIt2), robot control (ROS2 control), and physics simulation (Gazebo), providing a complete robotics development and testing environment.
