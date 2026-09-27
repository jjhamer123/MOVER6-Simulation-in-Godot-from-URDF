# MOVER6 Simulation in Godot (from URDF)

A Godot 4 simulation of a robotic arm URDF, built from its URDF and
driven live over ROS 2. The arm can act either as a **standalone physics
simulation** (commands drive the sim directly) or as a **digital shadow** of a
real Mover6 (the sim only mirrors state and never actuates), controlled by a
Simulink model talking to Godot over ROS 2 `JointJog`/`JointState` messages.

This project started life addressing hardware-accessibility limits in
robotics education (ELE303 → funded through the University of Sheffield's
SURE scheme): not every student can get time on a physical arm, but everyone
can open a simulation. The project isn't tied to the Mover6 specifically -
it's built as a general **URDF-in, ROS 2-controllable arm-out** pipeline, so
the same setup works for any serial robot arm you have a URDF for.

The arm's geometry, joints, and mass properties are all imported from the
Mover6 URDF (`urdf/cpr_robot/`) via the `godot_urdf` addon

## Important: this repo alone won't run

`JointJogController` and `JointStatePublisher` (the ROS 2 subscriber and
publisher used by `ros_2_arm.gd` and the demo scripts) are **not** GDScript -
they're native classes exposed by the
[`godot_ros`](https://github.com/nordstream3/Godot-4-ROS2-integration) C++
module, compiled directly into a custom Godot editor/export binary linked
against ROS 2 Jazzy. You need that custom binary (built and pointed at your
ROS 2 install) to open or run this project - the stock Godot editor will
fail to resolve those classes.

Everything else - `RobotJointController`, `ArmRig`, `BaseLinkLogger`,
`RosArmBridge` - is ordinary GDScript checked into this repo.

## Requirements

- A custom Godot 4.4 binary with the `godot_ros` module compiled in, linked
  against **ROS 2 Jazzy**
- ROS 2 Jazzy (native Linux is the smoothest path; see *Platform notes*
  below for Windows/WSL2)
- Jolt Physics (already selected as the 3D physics engine in
  `project.godot`)

## Repository structure

```
Robot.tscn                     Main scene: the assembled arm + controllers
control.gd                     Minimal demo: nudges joint1 at a fixed velocity
project.godot                  Godot project config (Jolt physics, mobile renderer)

Helpers/
  arm_rig.gd (ArmRig)          Loads a URDF, wires up the downstream nodes
  logger.gd (BaseLinkLogger)   Periodically logs the end-effector's pose

ros_2_arm.gd (RosArmBridge)    ROS 2 <-> Godot bridge: JointJog in, JointState
                                out; SIMULATION vs SHADOW mode; per-joint sign
                                overrides and soft limits

addons/godot_urdf/             URDF import + robot assembly
  controller/RobotJointController.gd
                                Drives Generic6DOFJoint3D motors per joint;
                                estimates link mass from collision geometry
                                when the URDF has no <inertial> tag
  urdf_6dof_joint3d.gd, urdf_parser.gd, urdf_loader.gd, ...
                                URDF parsing/import pipeline

addons/stl-io/                 STL mesh import (Mover6 link geometry)
addons/yaml_dot_gd/            YAML parsing (used by the URDF import config)

urdf/cpr_robot/                Mover6 URDF (matches the real cpr_ros2 driver)
urdf/CPMOVER6.urdf             Older/alternate copy - see Known issues
urdf/ur5e.urdf                 UR5e URDF, kept for reference/testing

pub_joint_states.gd / .cpp     Legacy/demo: standalone JointState publisher
ros_2test.gd                   Legacy/demo: fixed joint velocities + angle print
control(temp_demo).gd          Legacy/demo: sine-wave joint state simulator
robotfixes.gd                  One-off fix for STL meshes imported with the
                                wrong up-axis (rotates tagged meshes 90°)
```
## Setting up

1. Build the custom Godot binary with `godot_ros` linked against ROS 2
   Jazzy (see the `godot_ros` project for build instructions), or obtain a
   prebuilt copy.
2. Clone this repo and open it with that custom binary - **not** stock
   Godot.
3. Source your ROS 2 Jazzy environment before launching Godot from a
   terminal, so the linked `rclcpp` node can find your ROS 2 graph:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ./godot_ros_binary --path /path/to/this/repo
   ```
4. Open `Robot.tscn`. Confirm the `ArmRig` node's `URDF` export points at
   `urdf/cpr_robot/robots/CPRMover6.urdf.xacro`.
5. On the `RosArmBridge` node, set:
   - `mode`: `SIMULATION` to let incoming `/JointJog` drive the physics, or
     `SHADOW` to only mirror `/joint_states` without actuating
   - `max_joint_velocity`: the real arm's max joint speed in rad/s (Mover6's
   - `joint_sign_overrides`: only needed if a joint visibly moves opposite
     to the command; leave empty for a fresh URDF (see *Known issues*)

## Running it

- **With ROS 2**: `ros_2test.gd` commands a fixed velocity to
  joints and prints end joint angle every frame - useful for checking
  the `godot_ros` link is alive without needing a controller on the other
  end.

## Using this with a different arm

Nothing about the pipeline is Mover6-specific - the `godot_urdf` addon reads
any standard URDF (or xacro that resolves to one) and builds the links,
joints, and collision shapes from it, and `RobotJointController` drives
whatever `Generic6DOFJoint3D`s come out of that import by joint name. To
swap in a different robot arm:

1. Drop the new URDF (plus its mesh files - STL via `stl-io`, or standard
   Godot-importable formats) under `urdf/`.
2. Point `ArmRig`'s `URDF` export at the new file and hit **Load robot**.
3. Check the joint names that come out (`RobotJointController.get_joint_names()`,
   or just watch the console on `reinitialize()`) match what you intend to
   command over ROS 2 - these come straight from the URDF's `<joint name="...">`
   attributes, so rename joints in the URDF if you want tidier names.
4. Set `RosArmBridge.max_joint_velocity` to the new arm's real max joint
   speed, and re-check the mass/axis-sign gotchas below - every one of them
   is a property of the *URDF*, not of this codebase, so they resurface on
   any arm whose URDF has the same gaps (no `<inertial>`, axis direction
   that doesn't match your driver's convention).

## Controlling the arm over ROS 2

Once Godot is running the scene, the sim exposes the same two-topic
interface a real ROS 2-driven arm typically would:

| Topic | Direction | Message type | Purpose |
|---|---|---|---|
| `/JointJog` | subscribed by Godot | `control_msgs/msg/JointJog` | Command joint velocities |
| `/joint_states` | published by Godot | `sensor_msgs/msg/JointState` | Reports position + velocity per joint |

**Commanding the arm** - publish a `JointJog` with `joint_names` matching
the names reported by `RobotJointController`, and `velocities` as a
normalized fraction of `max_joint_velocity` (`-1.0` to `1.0`, not raw
rad/s). Positions/displacements aren't used by this bridge; leave them
empty. From the command line:

```bash
ros2 topic pub /JointJog control_msgs/msg/JointJog \
  "{joint_names: ['joint1', 'joint2'], velocities: [0.3, -0.2]}" --once
```

Or from Python with `rclpy`:

```python
from control_msgs.msg import JointJog
msg = JointJog()
msg.joint_names = ["joint1", "joint2"]
msg.velocities = [0.3, -0.2]
publisher.publish(msg)  # publisher created on /JointJog
```

**Reading the arm's state** - subscribe to `/joint_states` for live
position/velocity feedback, or watch it directly:

```bash
ros2 topic echo /joint_states
```

A few things worth knowing before wiring up a controller:

- Velocities are clamped to `[-1, 1]` and scaled by `max_joint_velocity`
  inside `RosArmBridge` - sending anything outside that range just gets
  clamped, it won't overdrive the joint.
- Soft limits from the URDF are respected: a commanded velocity that would
  push a joint past its limit (within `limit_margin`) is zeroed out rather
  than actuating through the stop.
- In `SHADOW` mode, `/JointJog` commands are still received and logged but
  never applied to the physics - the arm only reports whatever state it's
  already in. Use this when you want to observe a real arm's telemetry
  side-by-side with the sim without the sim fighting the real motion.
- `is_online()` on `RosArmBridge` reports whether a `/JointJog` message has
  arrived within `connection_timeout_sec` - useful for a HUD/status
  indicator if you're building a teaching UI around this.
- For a closed feedback loop (a controller that reacts to `/joint_states`
  rather than sending open-loop velocities), watch out for integral windup
  if the controller's position error sits near zero for a while before a
  large correction fires - see *Known issues* below.

## Known issues / things to check if you fork this for a new arm

- **Missing `<inertial>` data.** If a URDF link has no mass specified,
  Godot silently defaults it to 1 kg, which can produce a PI controller
  overshoot that looks stable at first and then destabilizes into chaotic
  motion after the arm settles near its target (integral windup interacting
  with unrealistically light links). `RobotJointController` now estimates a
  mass from each link's collision geometry when `<inertial>` is absent -
  check `estimate_missing_mass` / `link_mass_overrides` if a new arm behaves
  the same way.
- **Axis sign conventions.** The Mover6 URDF's `<axis>` directions didn't
  match the real `cpr_ros2` driver's convention for any of its six joints -
  fixed by negating all six axes directly in
  `urdf/cpr_robot/robots/CPRMover6.urdf.xacro` (safe here since every joint
  is `continuous` with no bounded limits to swap). If you bring in a
  different arm and it moves the wrong way, prefer fixing the URDF's axes
  over reaching for `joint_sign_overrides` - the override dictionary is a
  per-Godot-instance patch, not a fix to the source of truth.
- **STL up-axis.** Some imported STL meshes come in with the wrong up-axis;
  `robotfixes.gd` rotates any mesh tagged `is_stl` (metadata or Godot group)
  by 90° about X to correct it. Only needed for meshes actually affected -
  check visually after import before assuming you need this.

## Platform notes

Native Linux is the most reliable target for the ROS 2 side.

## Credits

- [`godot_urdf`](https://codeberg.org/brean/godot_urdf) by Askar Sulaimanov
  and Andreas Bresser - URDF import
- [`stl-io`](https://github.com/) by Valentin Bisson - STL mesh import
- [`godot_ros`](https://github.com/nordstream3/Godot-4-ROS2-integration) by
  nordstream3 - the Godot↔ROS 2 native module this project is built on
- CPR's `cpr_ros2` driver - reference for the real Mover6's joint/velocity
  conventions

## License

MIT - see `LICENSE`.
