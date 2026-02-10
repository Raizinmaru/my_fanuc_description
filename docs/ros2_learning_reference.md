<!-- markdownlint-disable MD013 MD024 -->
# ROS2 FANUC Description - Learning Reference

> Comprehensive analysis of the `my_fanuc_description` repository:
> FANUC industrial robot ROS2 Description packages for CRX, LR Mate,
> M-10, M-20, R-1000iA, and R-2000 series.

---

## 1. Repository Structure Overview

### 1-1. Directory Map

```text
my_fanuc_description/
├── .gitattributes                 # Git LFS tracking for mesh files (*.stl, *.obj, *.dae)
├── .gitignore                     # Ignores build/, install/, log/
├── .pre-commit-config.yaml        # Code quality hooks (Prettier, Ruff, codespell, etc.)
├── .prettierrc.cjs                # Prettier XML formatting config
├── CONTRIBUTING.md                # Contribution policy (no external PRs)
├── README.md                      # Project documentation
├── LICENSES/
│   ├── Apache-2.0.txt             # Primary license for source code
│   ├── MIT.txt                    # For specific dependencies
│   ├── BSD-2-Clause.txt           # For specific dependencies
│   └── BSD-3-Clause.txt           # For specific dependencies
│
├── fanuc_crx_description/         # CRX collaborative series (6 models)
├── fanuc_lrmate_description/      # LR Mate small series (2 models)
├── fanuc_m10_description/         # M-10 medium series (1 model)
├── fanuc_m20_description/         # M-20 medium-large series (2 models)
├── fanuc_r1000ia_description/     # R-1000iA heavy series (1 model)
└── fanuc_r2000_description/       # R-2000 heavy series (1 model)
```

### Role of Configuration Files

| File | Purpose |
|------|---------|
| `.pre-commit-config.yaml` | Automated code quality checks before git commits |
| `.prettierrc.cjs` | XML/Xacro formatting rules (preserve whitespace, double quotes) |
| `.gitattributes` | Git LFS tracking for binary mesh files |
| `LICENSES/` | REUSE-compliant multi-license setup (primary: Apache-2.0) |

### 1-2. Cross-Package Comparison

#### Common Structure (all 6 packages)

Every `fanuc_*_description` package follows this identical layout:

```text
fanuc_{series}_description/
├── CMakeLists.txt           # 16 lines, ament_cmake install only
├── package.xml              # format="3", 7 exec_depend
├── launch/
│   └── view_{series}.launch.py   # RViz visualization launcher
├── meshes/
│   └── {model_name}/
│       ├── visual/          # .dae (COLLADA) files
│       └── collision/       # .stl files
├── robot/
│   └── {model_name}.urdf.xacro   # Top-level robot definition
├── urdf/
│   └── {model_name}_urdf_macro.xacro  # Macro definition
└── rviz/
    └── view_{series}.rviz   # RViz display configuration
```

#### Package Inventory

| Package | Models | Mesh Files | Inertial Data | Collision Scale |
|---------|--------|------------|---------------|-----------------|
| fanuc_crx_description | 6 (crx5ia, crx10ia, crx10ia_l, crx10ia_lp, crx20ia_l, crx30ia) | 84 | Precise (CAD-derived) | `0.001` (mm to m) |
| fanuc_lrmate_description | 2 (lrmate200id, lrmate200id7l) | 28 | Approximate (many zeros) | `0.001` (mm to m) |
| fanuc_m10_description | 1 (m10_12-14d) | 14 | Approximate | None (already in m) |
| fanuc_m20_description | 2 (m20_25-18d, m20_35-18d) | 28 | Approximate | None (already in m) |
| fanuc_r1000ia_description | 1 (r1000ia_100f) | 14 | None (missing) | None (already in m) |
| fanuc_r2000_description | 1 (r2000ic_125l) | 14 | None (missing) | None (already in m) |

**Total: 13 robot models, 182 mesh files**

#### Naming Conventions

| Element | Pattern | Example |
|---------|---------|---------|
| Package name | `fanuc_{series}_description` | `fanuc_crx_description` |
| Top-level URDF | `robot/{model}.urdf.xacro` | `robot/crx5ia.urdf.xacro` |
| Macro file | `urdf/{model}_urdf_macro.xacro` | `urdf/crx5ia_urdf_macro.xacro` |
| Mesh directory | `meshes/{model}/visual\|collision/` | `meshes/crx5ia/visual/` |
| Launch file | `launch/view_{series}.launch.py` | `launch/view_crx.launch.py` |
| RViz config | `rviz/view_{series}.rviz` | `rviz/view_crx.rviz` |

#### Files Required for Adding a New Robot Model

To add a new model `{model}` to an existing package `fanuc_{series}_description`:

1. `urdf/{model}_urdf_macro.xacro` - Kinematic chain macro definition
2. `robot/{model}.urdf.xacro` - Top-level robot instantiation
3. `meshes/{model}/visual/base.dae, j1.dae, ..., j6.dae` - Visual meshes
4. `meshes/{model}/collision/base.stl, j1.stl, ..., j6.stl` - Collision meshes
5. Update `launch/view_{series}.launch.py` - Add model to `choices` list

---

## 2. ROS2 Package Configuration

### 2-1. package.xml Design Pattern

Reference: `fanuc_crx_description/package.xml`

```xml
<?xml version="1.0" ?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>fanuc_crx_description</name>
  <version>1.1.1</version>
  <description>CRX series kinematics information.</description>
  <maintainer email="fanuc-ros-maintainer@fanuc.co.jp">FANUC CORPORATION</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>urdf</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### Key Concepts

| Element | Explanation |
|---------|-------------|
| `format="3"` | ROS2 package manifest format 3 (supports conditional dependencies, version ranges). Standard for all ROS2 packages since Humble. |
| `buildtool_depend` | Build system dependency. `ament_cmake` is the CMake-based build tool for ROS2. Only needed during build. |
| `exec_depend` | Runtime dependency. These packages must be installed when running nodes from this package. |
| `build_depend` | Compile-time dependency. Not used here since Description packages have no compiled code. |
| `test_depend` | Test-time dependency. Not used here since no automated tests are defined. |
| `<export><build_type>` | Tells ROS2 tooling (colcon) which build system to use. Required for ament_cmake packages. |

#### Description Package Dependency Pattern

The 7 `exec_depend` entries form a standard "visualization stack":

- **`xacro`**: Processes `.xacro` files into URDF XML at runtime
- **`urdf`**: URDF parsing library
- **`robot_state_publisher`**: Publishes TF transforms from URDF + joint states
- **`joint_state_publisher_gui`**: Provides GUI sliders for joint angle testing
- **`launch` / `launch_ros`**: Python launch system
- **`rviz2`**: 3D visualization tool

#### Dependencies for Extended Package Types

| Package Type | Additional Dependencies |
|-------------|------------------------|
| Control (ros2_control) | `controller_manager`, `ros2_control`, `ros2_controllers`, `hardware_interface` |
| MoveIt2 | `moveit_ros_planning_interface`, `moveit_ros_move_group`, `moveit_kinematics` |
| Custom C++ Node | `rclcpp`, `std_msgs`, `geometry_msgs`, `tf2_ros` |
| Custom Python Node | `rclpy`, `std_msgs`, `geometry_msgs`, `tf2_ros` |
| Custom Messages | `rosidl_default_generators` (build), `rosidl_default_runtime` (exec) |

### 2-2. CMakeLists.txt Minimal Configuration

Reference: `fanuc_crx_description/CMakeLists.txt` (16 lines)

```cmake
cmake_minimum_required(VERSION 3.8)
project(fanuc_crx_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes urdf robot rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

#### Line-by-Line Explanation

| Line | Purpose |
|------|---------|
| `cmake_minimum_required(VERSION 3.8)` | CMake 3.8+ required. ROS2 Humble recommends 3.8+. |
| `project(fanuc_crx_description)` | Must match `<name>` in package.xml exactly. |
| `find_package(ament_cmake REQUIRED)` | Loads ament_cmake macros (`ament_package()`, install helpers). |
| `install(DIRECTORY ... DESTINATION share/${PROJECT_NAME}/)` | Copies resource directories to the install space under `share/`. This is how `FindPackageShare()` locates them at runtime. |
| `ament_package()` | Registers the package with the ament index, generates package marker files. Must be the last call. |

#### Why `share/`?

ROS2 uses a FHS-like install layout:

- `lib/` - Executables and libraries
- `share/{pkg}/` - Data files (URDF, meshes, launch, config)
- `include/` - C++ headers

`FindPackageShare()` resolves to `{install_prefix}/share/{pkg}/`, so
`install(DIRECTORY ... DESTINATION share/${PROJECT_NAME}/)` makes all
resource files discoverable by the launch system.

#### Extensions for C++/Python Nodes

```cmake
# For C++ nodes, add after find_package:
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)
install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

# For Python nodes (ament_cmake with Python):
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/my_script.py DESTINATION lib/${PROJECT_NAME})
```

---

## 3. URDF/Xacro Design Patterns

### 3-1. Two-Layer Architecture

This project separates robot definitions into two layers for maximum reusability:

#### Layer 1: Top-Level Robot File (`robot/*.urdf.xacro`)

```xml
<!-- robot/crx5ia.urdf.xacro (12 lines) -->
<robot name="crx5ia" xmlns:xacro="http://wiki.ros.org/xacro">
  <xacro:include
    filename="$(find fanuc_crx_description)/urdf/crx5ia_urdf_macro.xacro" />

  <link name="world" />
  <link name="ee_link" />
  <xacro:crx5ia parent="world" child="ee_link">
    <origin xyz="0 0 .75" rpy="0 0 0" />
  </xacro:crx5ia>
</robot>
```

**Responsibilities:**

- Defines the `world` frame (TF tree root)
- Defines the `ee_link` frame (end-effector attachment point)
- Includes the macro file and instantiates the robot
- Sets the robot mounting height via `<origin xyz="0 0 .75">`

#### Layer 2: Macro Definition (`urdf/*_urdf_macro.xacro`)

```xml
<!-- urdf/crx5ia_urdf_macro.xacro (396 lines, abbreviated) -->
<robot xmlns:xacro="http://wiki.ros.org/xacro">
  <xacro:macro name="crx5ia"
    params="prefix='' parent *origin child">

    <!-- Link dimensions -->
    <xacro:property name="l_base" value="0.185" />
    <!-- ... -->

    <!-- Joint limits -->
    <xacro:property name="j1_lower_limit" value="${radians(-200)}" />
    <!-- ... -->

    <!-- 7 links: base_link + J1..J6_link -->
    <!-- 6 revolute joints: J1..J6 -->
    <!-- Auxiliary frames: wbase, flange -->
    <!-- Parent/child connections: base_joint, child_joint -->
  </xacro:macro>
</robot>
```

**Responsibilities:**

- Complete kinematic chain definition (links + joints)
- Physical properties (inertia, mass, center of gravity)
- Mesh references (visual + collision)
- Joint limits (position, velocity, effort)
- Auxiliary coordinate frames (wbase, flange)
- Parent/child connection interface

#### Why Two Layers?

| Benefit | Explanation |
|---------|-------------|
| **Multi-robot support** | Multiple instances with different `prefix` values can coexist in one URDF |
| **Flexible mounting** | Different `parent` links and `origin` transforms per instance |
| **Tool attachment** | Different `child` links for different end-effectors |
| **Clean separation** | Robot geometry (Layer 2) is independent of scene setup (Layer 1) |

#### Macro Parameters

| Parameter | Type | Purpose |
|-----------|------|---------|
| `prefix` | String (default: `''`) | Namespace prefix for all link/joint names. Enables multiple instances. |
| `parent` | String | Name of the parent link to attach the robot base to. |
| `*origin` | Block | XML block inserted as the base_joint transform. The `*` means it accepts a child XML element. |
| `child` | String | Name of the child link attached to the flange. |

**Multi-robot example:**

```xml
<xacro:crx5ia prefix="left_" parent="world" child="left_ee_link">
  <origin xyz="-0.5 0 0.75" rpy="0 0 0" />
</xacro:crx5ia>
<xacro:crx5ia prefix="right_" parent="world" child="right_ee_link">
  <origin xyz="0.5 0 0.75" rpy="0 0 0" />
</xacro:crx5ia>
```

### 3-2. Kinematics Chain Analysis

Using CRX-5iA as the reference model:

#### Link Dimensions

```text
l_base = 0.185 m   (base_link to J1 rotation center)
l_1    = 0.0   m   (J1 to J2 offset in X)
l_2    = 0.410 m   (J2 to J3 in Z - upper arm)
l_3    = 0.0   m   (J3 to J4 offset in Z)
l_4    = 0.430 m   (J4 to J5 in X - forearm)
l_5    = 0.130 m   (J5 to J6 offset in -Y)
l_6    = 0.145 m   (J6 to flange in X)
```

#### Joint Axis Configuration

All FANUC robots in this repository share the same axis pattern:

| Joint | Axis | Direction | Motion |
|-------|------|-----------|--------|
| J1 | Z | `(0, 0, 1)` | Base rotation (yaw) |
| J2 | Y | `(0, 1, 0)` | Shoulder pitch |
| J3 | -Y | `(0, -1, 0)` | Elbow pitch (opposite to J2) |
| J4 | -X | `(-1, 0, 0)` | Wrist roll |
| J5 | -Y | `(0, -1, 0)` | Wrist pitch |
| J6 | -X | `(-1, 0, 0)` | Wrist roll (tool rotation) |

This is the standard FANUC 6-axis configuration and follows a ZYY-XYX Euler angle pattern
for the wrist, consistent with industrial robot conventions.

#### Joint Limits Comparison (All Models)

| Model | J1 range | J2 range | J3 range | Max effort J1 (Nm) | Payload |
|-------|----------|----------|----------|---------------------|---------|
| CRX-5iA | +/-200 deg | +/-179.9 deg | -68..248 deg | 160 | 5 kg |
| LR Mate 200iD | +/-170 deg | -100..145 deg | -70..205 deg | 450 | 7 kg |
| M-10iD/12 | +/-170 deg | -100..145 deg | -70..205 deg | 2000 | 12 kg |
| M-20iD/25 | +/-185 deg | -100..160 deg | -90..220 deg | 7000 | 25 kg |
| R-1000iA/100F | +/-185 deg | -60..76 deg | -79..180 deg | 10000 | 100 kg |
| R-2000iC/125L | +/-185 deg | -60..76 deg | -79..180 deg | 10000 | 125 kg |

#### Kinematic Parameters by Model (DH-like)

| Model | Base Height | Upper Arm | Forearm | Flange Offset |
|-------|-------------|-----------|---------|---------------|
| CRX-5iA | 0.185 m | 0.410 m | 0.430 m | 0.145 m |
| LR Mate 200iD | 0.330 m | 0.330 m | 0.335 m | 0.080 m |
| M-10iD/12 | 0.450 m | 0.640 m | 0.700 m | 0.075 m |
| M-20iD/25 | 0.425 m | 0.840 m | 0.890 m | 0.090 m |
| R-1000iA/100F | 0.520 m | 1.100 m | 1.160 m | 0.220 m |
| R-2000iC/125L | 0.670 m | 1.075 m | 1.730 m | 0.215 m |

#### Inertial Properties

Three levels of detail exist across packages:

1. **Precise (CRX series)**: CAD-derived values with full 6-element inertia tensors
2. **Approximate (LR Mate, M-10, M-20)**: Estimated values, some with zero inertia tensors
3. **Missing (R-1000iA, R-2000)**: No `<inertial>` elements defined

This affects simulation fidelity in Gazebo/Isaac Sim. For dynamic simulation,
all models would need proper inertial data.

#### Mesh Reference Pattern

**Visual meshes** (COLLADA .dae):

```xml
<mesh filename="package://fanuc_crx_description/meshes/crx5ia/visual/base.dae" />
```

- No scale attribute needed (meshes are in meters)
- Rich material/color information

**Collision meshes** (STL):

```xml
<!-- CRX and LR Mate: STL in millimeters, needs scale -->
<mesh filename="package://fanuc_crx_description/meshes/crx5ia/collision/base.stl"
      scale="0.001 0.001 0.001" />

<!-- M-10, M-20, R-1000iA, R-2000: STL in meters, no scale -->
<mesh filename="package://fanuc_m10_description/meshes/m10_12-14d/collision/base.stl" />
```

### 3-3. Auxiliary Frames

#### wbase (FANUC World Coordinates)

```xml
<link name="${prefix}wbase" />
<joint name="${prefix}base_link-wbase" type="fixed">
  <origin xyz="0 0 ${l_base}" rpy="0 0 0" />
  <parent link="${prefix}base_link" />
  <child link="${prefix}wbase" />
</joint>
```

- Represents FANUC's "World" coordinate system origin
- Located at the intersection of J1 rotation axis and the robot's mechanical zero plane
- Corresponds to the coordinate system used by FANUC teach pendants
- Offset from `base_link` by the base height (same as J1 joint origin)

#### flange (Tool Mounting Surface)

```xml
<link name="${prefix}flange" />
<joint name="${prefix}J6-flange" type="fixed">
  <origin xyz="${l_6} 0 0" rpy="0 0 0" />
  <parent link="${prefix}J6_link" />
  <child link="${prefix}flange" />
</joint>
```

- Represents the mechanical tool mounting surface (ISO 9409-1 flange)
- Origin at the center of the flange face
- Z-axis pointing outward along the tool approach direction
- All end-effector tools are attached relative to this frame

#### TF Tree Structure

```text
world
  └── base_joint (fixed)
        └── base_link
              ├── base_link-wbase (fixed) → wbase
              └── J1 (revolute, Z-axis)
                    └── J1_link
                          └── J2 (revolute, Y-axis)
                                └── J2_link
                                      └── J3 (revolute, -Y-axis)
                                            └── J3_link
                                                  └── J4 (revolute, -X-axis)
                                                        └── J4_link
                                                              └── J5 (revolute, -Y-axis)
                                                                    └── J5_link
                                                                          └── J6 (revolute, -X-axis)
                                                                                └── J6_link
                                                                                      └── J6-flange (fixed) → flange
                                                                                            └── child_joint (fixed) → ee_link
```

#### Frame Naming Standards

| Frame | ROS2 Convention | FANUC Convention |
|-------|-----------------|------------------|
| `world` | TF tree root | N/A |
| `base_link` | Robot mounting frame (REP-120) | User Frame |
| `wbase` | N/A (project-specific) | World Frame |
| `flange` | Aligned with REP-103 | Tool Frame (UTOOL=0) |
| `ee_link` | End-effector tip | Tool Center Point (TCP) |

---

## 4. Launch File Design

### 4-1. Python Launch File Structure

Reference: `fanuc_crx_description/launch/view_crx.launch.py`

#### Import Structure

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration,
    PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
```

| Module | Purpose |
|--------|---------|
| `ament_index_python` | Resolve package install paths via ament index |
| `launch` | Core launch system (descriptions, arguments, substitutions) |
| `launch_ros` | ROS2-specific launch extensions (Node, FindPackageShare) |

#### Launch Argument Pattern

```python
DeclareLaunchArgument(
    "robot_model",
    description="The robot model to visualize (required)",
    choices=["crx5ia", "crx10ia", "crx10ia_l", "crx10ia_lp", "crx20ia_l", "crx30ia"],
)
robot_model = LaunchConfiguration("robot_model")
```

- `DeclareLaunchArgument` declares a parameter that can be set from the command line
- `choices` restricts valid values (validation at launch time)
- `LaunchConfiguration` creates a lazy reference to the argument value
- Usage: `ros2 launch fanuc_crx_description view_crx.launch.py robot_model:=crx5ia`

#### Xacro Processing Pipeline

```python
# Step 1: Build the path to the xacro file
description_file = PathJoinSubstitution([
    FindPackageShare("fanuc_crx_description"),
    "robot",
    PythonExpression(["'", robot_model, ".urdf.xacro'"]),
])

# Step 2: Execute xacro to generate URDF XML
robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    description_file,
])
```

This is a lazy evaluation pipeline:

1. `FindPackageShare` resolves to the package's `share/` directory at launch time
2. `PythonExpression` dynamically constructs the filename from the launch argument
3. `Command` executes `xacro {path}` and captures the URDF XML output
4. The result is passed as the `robot_description` parameter to `robot_state_publisher`

#### Node Startup Pattern

```python
# GUI for manual joint angle control
joint_state_publisher_node = Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
)

# Publishes TF from URDF + /joint_states
robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="both",
    parameters=[{"robot_description": robot_description_content}],
)

# 3D visualization
rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="both",
    arguments=["--display-config", rviz_file],
)
```

Data flow: `joint_state_publisher_gui` -> `/joint_states` -> `robot_state_publisher` -> `/tf` -> `rviz2`

### 4-2. Substitution Pattern Reference

| Pattern | Usage in Project | Description |
|---------|-----------------|-------------|
| `FindPackageShare(pkg)` | Path to installed `share/` dir | Resolves via ament index |
| `PathJoinSubstitution([...])` | Build file paths | OS-independent path joining |
| `PythonExpression([...])` | Dynamic filename construction | Evaluates Python at launch time |
| `FindExecutable(name=...)` | Locate `xacro` binary | Searches `PATH` |
| `Command([...])` | Execute xacro | Runs command, captures stdout |
| `LaunchConfiguration(name)` | Access launch arguments | Lazy reference to arg value |

#### Additional Substitutions for Extended Development

| Pattern | Use Case |
|---------|----------|
| `IfCondition(cond)` / `UnlessCondition(cond)` | Conditional node launching (e.g., `use_sim:=true`) |
| `GroupAction([...])` | Scope actions with shared namespace/parameters |
| `IncludeLaunchDescription(...)` | Compose launch files from other packages |
| `SetEnvironmentVariable(name, value)` | Set env vars (e.g., Gazebo model path) |
| `TimerAction(period=..., actions=[...])` | Delayed node startup |
| `RegisterEventHandler(...)` | React to lifecycle events (node started/exited) |

---

## 5. Build System (ament_cmake)

### 5-1. Build-to-Execution Flow

#### Step 1: Build

```bash
colcon build --packages-select fanuc_crx_description
```

Processing sequence:

1. `cmake_minimum_required(VERSION 3.8)` - Version check
2. `find_package(ament_cmake REQUIRED)` - Load ament macros
3. `install(DIRECTORY launch meshes urdf robot rviz DESTINATION share/${PROJECT_NAME}/)` -
   Copy all resource directories to `install/fanuc_crx_description/share/fanuc_crx_description/`
4. `ament_package()` - Generate ament index marker files

#### Step 2: Source

```bash
source install/setup.bash
```

This adds the package to the ament index, enabling:

- `FindPackageShare("fanuc_crx_description")` resolution
- `ros2 pkg prefix fanuc_crx_description` lookup
- `ros2 launch` tab completion

#### Step 3: Launch

```bash
ros2 launch fanuc_crx_description view_crx.launch.py robot_model:=crx5ia
```

Runtime sequence:

1. Ament index resolves package path
2. Launch system loads `view_crx.launch.py`
3. Xacro processes `robot/crx5ia.urdf.xacro` into URDF XML
4. Nodes start: joint_state_publisher_gui, robot_state_publisher, rviz2

#### Development Tip: Symlink Install

```bash
colcon build --symlink-install --packages-select fanuc_crx_description
```

Creates symlinks instead of copies. Changes to source files (xacro, launch, config)
take effect immediately without rebuilding. Does not work for compiled code.

---

## 6. Control Package Extension Guide

### 6-1. ros2_control Integration

To extend from Description to Control, create `fanuc_crx_control/`:

```text
fanuc_crx_control/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── controllers.yaml
├── urdf/
│   └── crx5ia_ros2_control.xacro
└── launch/
    └── control.launch.py
```

#### ros2_control URDF Extension

```xml
<!-- urdf/crx5ia_ros2_control.xacro -->
<robot xmlns:xacro="http://wiki.ros.org/xacro">
  <xacro:include filename="$(find fanuc_crx_description)/urdf/crx5ia_urdf_macro.xacro"/>

  <ros2_control name="FanucCRXSystem" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
      <param name="calculate_dynamics">true</param>
    </hardware>
    <xacro:macro name="joint_interface" params="name">
      <joint name="${name}">
        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
    </xacro:macro>

    <xacro:joint_interface name="J1"/>
    <xacro:joint_interface name="J2"/>
    <xacro:joint_interface name="J3"/>
    <xacro:joint_interface name="J4"/>
    <xacro:joint_interface name="J5"/>
    <xacro:joint_interface name="J6"/>
  </ros2_control>
</robot>
```

#### Controller Configuration

```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - J1
      - J2
      - J3
      - J4
      - J5
      - J6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
```

### 6-2. Custom Hardware Interface

For real FANUC robot communication, implement `hardware_interface::SystemInterface`:

```cpp
// Key methods to implement:
hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info);
std::vector<hardware_interface::StateInterface> export_state_interfaces();
std::vector<hardware_interface::CommandInterface> export_command_interfaces();
hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period);
hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period);
```

Joint parameters to read from URDF (via `HardwareInfo`):

- 6 joints, all revolute
- Position limits: from `j*_lower_limit` / `j*_upper_limit`
- Velocity limits: from `j*_velocity_limit`
- Effort limits: from `j*_effort_limit`

---

## 7. MoveIt2 Integration Guide

### 7-1. MoveIt2 Config Package

Key configuration files needed:

#### SRDF (Semantic Robot Description)

```xml
<!-- config/crx5ia.srdf -->
<robot name="crx5ia">
  <group name="manipulator">
    <chain base_link="base_link" tip_link="flange"/>
  </group>
  <group_state name="home" group="manipulator">
    <joint name="J1" value="0"/>
    <joint name="J2" value="0"/>
    <joint name="J3" value="0"/>
    <joint name="J4" value="0"/>
    <joint name="J5" value="0"/>
    <joint name="J6" value="0"/>
  </group_state>
  <end_effector name="tool" parent_link="flange" group="manipulator"/>
  <!-- Disable collision checking between adjacent links -->
  <disable_collisions link1="base_link" link2="J1_link" reason="Adjacent"/>
  <!-- ... additional pairs from collision matrix generation -->
</robot>
```

#### Kinematics Configuration

```yaml
# config/kinematics.yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```

#### Joint Limits (from URDF)

```yaml
# config/joint_limits.yaml
joint_limits:
  J1:
    has_velocity_limits: true
    max_velocity: 2.618  # radians(150)
    has_acceleration_limits: true
    max_acceleration: 5.0
  J2:
    has_velocity_limits: true
    max_velocity: 2.618  # radians(150)
    has_acceleration_limits: true
    max_acceleration: 5.0
  # ... J3-J6
```

### 7-2. Planning API Examples

```python
# MoveGroupInterface usage
from moveit_msgs.msg import MoveGroupAction
from geometry_msgs.msg import Pose, PoseStamped

# Joint space planning
move_group.set_joint_value_target([0.0, -0.5, 1.0, 0.0, 0.5, 0.0])
plan = move_group.plan()

# Cartesian space planning
waypoints = [pose1, pose2, pose3]
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

# Frame references from this project:
# - base_frame: "base_link"
# - planning_frame: "base_link" (or "world")
# - end_effector_link: "flange" (or "ee_link")
```

---

## 8. Custom Node Development Guide

### 8-1. Robot State Monitor Node

Input topics from the Description stack:

- `/joint_states` (`sensor_msgs/JointState`): J1-J6 position, velocity, effort
- `/tf` (`tf2_msgs/TFMessage`): Full kinematic chain transforms

Output topics:

- `/fanuc/tcp_pose` (`geometry_msgs/PoseStamped`): Flange position in world frame
- `/fanuc/joint_limits_status`: Per-joint proximity to limits

Key parameters derived from URDF:

```python
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
JOINT_LIMITS = {
    "J1": {"lower": radians(-200), "upper": radians(200)},
    "J2": {"lower": radians(-179.9), "upper": radians(179.9)},
    "J3": {"lower": radians(-68), "upper": radians(248)},
    "J4": {"lower": radians(-190), "upper": radians(190)},
    "J5": {"lower": radians(-179.9), "upper": radians(179.9)},
    "J6": {"lower": radians(-225), "upper": radians(225)},
}
```

### 8-2. Custom Message Definitions

For a `fanuc_msgs` package:

```text
# msg/JointLimitsStatus.msg
std_msgs/Header header
string[] joint_names
float64[] current_positions
float64[] lower_limits
float64[] upper_limits
float64[] limit_proximity    # 0.0 (at center) to 1.0 (at limit)
bool[] near_limit

# srv/GetKinematicsInfo.srv
string robot_model
---
string[] joint_names
float64[] link_lengths
float64[] lower_limits
float64[] upper_limits
float64[] velocity_limits
float64[] effort_limits

# action/MoveToJointTarget.action
float64[] target_positions
float64 max_velocity_scaling
---
bool success
string message
---
float64[] current_positions
float64 progress
```

Build requirements in `CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointLimitsStatus.msg"
  "srv/GetKinematicsInfo.srv"
  "action/MoveToJointTarget.action"
  DEPENDENCIES std_msgs
)
```

---

## 9. Gazebo Simulation Integration

### Key Resources from Description Packages

| Resource | Format | Usage |
|----------|--------|-------|
| Visual meshes | `.dae` (COLLADA) | Gazebo rendering |
| Collision meshes | `.stl` | Physics engine collision detection |
| Inertial data | `<inertial>` XML | Dynamic simulation (mass, inertia) |
| Joint limits | `<limit>` XML | Joint constraint enforcement |

### Gazebo Extension Design

Following the 2-layer pattern, create a separate file for Gazebo properties:

```xml
<!-- urdf/crx5ia_gazebo.xacro -->
<robot xmlns:xacro="http://wiki.ros.org/xacro">
  <gazebo>
    <plugin filename="gz_ros2_control-system"
            name="gz_ros2_control::GazeboSimSystem">
      <parameters>$(find fanuc_crx_control)/config/controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Per-link physics properties -->
  <xacro:macro name="gazebo_link" params="name mu:=0.8">
    <gazebo reference="${name}">
      <mu1>${mu}</mu1>
      <mu2>${mu}</mu2>
      <material>Gazebo/Grey</material>
    </gazebo>
  </xacro:macro>
</robot>
```

---

## 10. Testing and CI/CD

### 10-1. URDF Validation Tests

A test suite should validate all 13 robot models across these dimensions:

1. **Xacro compilation**: `.xacro` files produce valid XML
2. **URDF structure**: Valid kinematic tree with expected links/joints
3. **Mesh references**: All referenced mesh files exist on disk
4. **Joint limits**: `lower < upper` for all joints
5. **Inertial parameters**: Positive mass, positive semi-definite inertia tensors (where present)

See `tests/test_urdf_validation.py` for the complete test implementation.

### 10-2. Code Quality Configuration

#### Pre-commit Hooks (from `.pre-commit-config.yaml`)

| Hook | Target | Purpose |
|------|--------|---------|
| `trailing-whitespace` | All files | Remove trailing spaces |
| `end-of-file-fixer` | All files | Ensure newline at EOF |
| `check-yaml` | `*.yaml` | YAML syntax validation |
| `check-ast` | `*.py` | Python syntax validation |
| `prettier` | `*.xml, *.xacro` | XML formatting (preserves whitespace, double quotes) |
| `codespell` | All files | Spelling corrections |
| `markdownlint-cli2` | `*.md` | Markdown formatting |
| `ruff-check` | `*.py` | Python linting with auto-fix |
| `ruff-format` | `*.py` | Python formatting (Black-compatible) |

#### Additional Hooks for Extended Packages

| Tool | Purpose | Target |
|------|---------|--------|
| `clang-format` | C++ code formatting | `*.cpp, *.hpp` |
| `clang-tidy` | C++ static analysis | `*.cpp, *.hpp` |
| `mypy` | Python type checking | `*.py` |
| `ament_lint_auto` | ROS2 standard linting | All package files |
| `ament_cppcheck` | C++ error detection | `*.cpp, *.hpp` |

---

## Appendix: Quick Reference

### Launch Commands

```bash
# CRX series
ros2 launch fanuc_crx_description view_crx.launch.py robot_model:=crx5ia
ros2 launch fanuc_crx_description view_crx.launch.py robot_model:=crx10ia

# LR Mate series
ros2 launch fanuc_lrmate_description view_lrmate.launch.py robot_model:=lrmate200id

# M-10 series
ros2 launch fanuc_m10_description view_m10.launch.py robot_model:=m10_12-14d

# M-20 series
ros2 launch fanuc_m20_description view_m20.launch.py robot_model:=m20_25-18d

# R-1000iA series
ros2 launch fanuc_r1000ia_description view_r1000ia.launch.py robot_model:=r1000ia_100f

# R-2000 series
ros2 launch fanuc_r2000_description view_r2000.launch.py robot_model:=r2000ic_125l
```

### Build Commands

```bash
# Build all packages
colcon build

# Build single package with symlinks (for development)
colcon build --symlink-install --packages-select fanuc_crx_description

# Source workspace
source install/setup.bash

# Run URDF validation tests
python -m pytest tests/test_urdf_validation.py -v
```

### Useful ROS2 CLI Commands

```bash
# List installed packages
ros2 pkg list | grep fanuc

# Get package path
ros2 pkg prefix fanuc_crx_description

# Process xacro manually
xacro $(ros2 pkg prefix fanuc_crx_description)/share/fanuc_crx_description/robot/crx5ia.urdf.xacro

# Validate URDF
xacro robot/crx5ia.urdf.xacro | check_urdf /dev/stdin

# View TF tree
ros2 run tf2_tools view_frames
```
