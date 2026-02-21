# OptiTrack4PX4
### An OptiTrack → ROS 2 → PX4 External Vision Bridge
![Status](https://img.shields.io/badge/Status-Hardware_Validated-blue)
![OptiTrack](https://img.shields.io/badge/Motive-Client-blue)
[![ROS 2 Humble_Compatible](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![ROS 2 Jazzy_Compatible](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)
[![PX4 Compatible](https://img.shields.io/badge/PX4-Autopilot-pink)](https://github.com/PX4/PX4-Autopilot)

`optitrack4px4` is a ROS 2 (C++) package that streams OptiTrack motion capture data into the PX4 EKF using External Vision fusion, enabling position and heading fusion for hardware flight experiments. The package connects to OptiTrack's Motive software via the NatNet protocol, converts Y-up ENU measurements to NED, and publishes pose data in quaternions as well as euler angles under the rigid body name as defined in Motive.

Another launch file then relays this information for use in PX4 External Vision EKF Fusion by publishing the data as `px4_msgs/msg/VehicleOdometry` messages on `/fmu/in/vehicle_visual_odometry`, and handles the quaternion reordering and timestamping that PX4 expects. A secondary full-state relay node is available to merge the fused EKF output back from `/fmu/out/vehicle_odometry`, and `/fmu/out/vehicle_local_position` into one topic that relays all pose data including available higher order derivatives from the EKF for convenient logging and control.

Tested with ROS 2 Jazzy Jalisco (Ubuntu 24.04) and Humble Hawksbill (Ubuntu 22.04).

---

## Quick start

Assumes ROS 2 Jazzy/Humble is already installed and `rosdep` initialized.

1. Source ROS 2:

    ```bash
    source /opt/ros/$ROS_DISTRO/setup.bash
    ```

2. Create (or choose) a workspace directory:

    ```bash
    mkdir -p ~/ws_mocap_px4_msgs_drivers/src
    cd ~/ws_mocap_px4_msgs_drivers/src
    ```

3. Clone the packages into `src/`:

    ```bash
    git clone git@github.com:evannsm/optitrack4px4.git
    git clone -b v1.16_minimal_msgs git@github.com:evannsm/px4_msgs.git
    git clone git@github.com:evannsm/mocap_msgs.git
    git clone git@github.com:evannsm/mocap_px4_relays.git
    cd ..   # back to workspace root
    ```

4. Install ROS 2 dependencies:

    ```bash
    rosdep install --from-paths src --rosdistro $ROS_DISTRO -y --ignore-src
    ```

5. Build with colcon (Python invocation helps with virtual environments):

    ```bash
    python3 -m colcon build                 \
      --symlink-install                     \
      --cmake-args                          \
        -DCMAKE_BUILD_TYPE=Release          \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    ```

6. Source the overlay:

    ```bash
    source install/setup.bash
    ```

### Setting up networking parameters

The OptiTrack client connects to Motive over the network using the NatNet protocol. You need to configure the server and local IP addresses.

1. Edit the default `server_address` in `optitrack4px4/launch/client.launch.py` (or pass it as a launch argument) to match the IP of the machine running Motive. For example:
```python
    server_address_arg = DeclareLaunchArgument(
        'server_address',
        default_value='192.168.1.113',
        description='OptiTrack/Motive server IP address'
    )
```

2. Similarly, set `local_address` to the IP of the Ubuntu machine running this ROS 2 stack on the same LAN.

3. The default connection mode is **Unicast** on NatNet command port `1510` and data port `1511`. These can be overridden via launch arguments or the config file at `config/optitrack4px4_params.yaml`.

4. Make sure the Ubuntu computer and the Motive computer are on the same LAN and can reach each other.

### Launching for vision fusion

To run the OptiTrack client and visual odometry relay (the typical flight-test configuration):

In one terminal:
```bash
ros2 launch optitrack4px4 client.launch.py
```

In another terminal:
```bash
ros2 launch mocap_px4_relays visual_odometry_relay.launch.py
```

And to also include the full state relay:

```bash
ros2 launch mocap_px4_relays full_state_relay.launch.py
```

Or use the combined launch files to start everything from a single command:

```bash
# Client + visual odometry relay
ros2 launch optitrack4px4 client_and_visual_odometry.launch.py

# Client + visual odometry relay + full state relay
ros2 launch optitrack4px4 client_vision_full_all.launch.py
```

The combined launch files accept a `rigid_body_name` argument (default `drone`) and automatically remap the relay's input topic to match:

```bash
ros2 launch optitrack4px4 client_and_visual_odometry.launch.py rigid_body_name:=quadrotor
```

> **Note:** The relay nodes have been moved to the [`mocap_px4_relays`](../mocap_px4_relays/) package so they can be reused with any motion capture source.

### Example topic tree (all three nodes running) assuming your rigid body is named `drone` in Motive

```text
/optitrack/
├── drone/
│   ├── drone       [geometry_msgs/PoseStamped]
│   └── drone_euler [mocap_msgs/PoseEuler]

/fmu/in/vehicle_visual_odometry          [px4_msgs/VehicleOdometry]   <- to EKF   (mocap_px4_relays)
/merge_odom_localpos/full_state_relay    [mocap_msgs/FullState]       <- from EKF  (mocap_px4_relays)
```

```bash
# Verify vision data is reaching PX4
ros2 topic echo /fmu/in/vehicle_visual_odometry

# Check the merged full-state output
ros2 topic echo /merge_odom_localpos/full_state_relay

# Raw OptiTrack pose
ros2 topic echo /optitrack/drone/drone
```

#### Published topics of the client.launch.py alone

All topics are published under the configured `namespace` (default `optitrack`). Topics are created dynamically for each rigid body discovered in Motive:

```text
/<namespace>/<rigid_body_name>/<rigid_body_name>         [geometry_msgs/PoseStamped]
/<namespace>/<rigid_body_name>/<rigid_body_name>_euler    [mocap_msgs/PoseEuler]
```

- **PoseStamped**: position (x, y, z) + quaternion (qw, qx, qy, qz) in NED
- **PoseEuler**: position (x, y, z) + roll, pitch, yaw in radians

`<rigid_body_name>` is taken from the rigid body definitions in Motive.

#### TF tree

```text
map (world_frame)
└── optitrack (optitrack_frame)          [static]
    ├── <rigid_body_1>_<rigid_body_1>    [dynamic]
    └── <rigid_body_2>_<rigid_body_2>    [dynamic]
```

The static `map -> optitrack` transform is defined by `map_xyz` and `map_rpy`. Dynamic child frames update with each OptiTrack measurement.

---

## Relay nodes

The **visual_odometry_relay** and **full_state_relay** nodes have been moved to the [`mocap_px4_relays`](../mocap_px4_relays/) package so they can be reused with any motion capture source (Vicon, OptiTrack, etc.). See that package's README for full documentation.

### Data pipeline

```text
OptiTrack Motive (Y-up ENU, millimeters)
        |
   optitrack_client node  (optitrack4px4)
        |  connects via NatNet (Unicast/Multicast)
        |  converts Y-up ENU -> NED, mm -> m
        v
  /optitrack/*rigid_body_name*/*rigid_body_name* (geometry_msgs/PoseStamped, NED)
        |
  visual_odometry_relay node  (mocap_px4_relays)
        |  reorders quaternion, stamps, publishes at 35 Hz
        v
  /fmu/in/vehicle_visual_odometry  (px4_msgs/VehicleOdometry)
        |
   PX4 EKF2 (fuses vision + IMU)
        |
        v
  /fmu/out/vehicle_odometry & /fmu/out/vehicle_local_position
        |
  full_state_relay node  (mocap_px4_relays)  [optional]
        |  merges both into one topic at 40 Hz
        v
  /merge_odom_localpos/full_state_relay  (mocap_msgs/FullState)
```

For the EKF to accept vision input you must enable it on the PX4 side (the `EKF2_EV_CTRL` and `EKF2_HGT_REF` must be set up according to what your motion capture system can provide). It is also recommended to turn off magnetometer fusion to avoid issues in indoor environments.

---

## Configuration

All parameters can be set via the config file (`config/optitrack4px4_params.yaml`) or overridden as launch arguments.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `connection_type` | `Unicast` | `Unicast` or `Multicast` |
| `server_address` | `192.168.1.113` | IP of the machine running Motive |
| `local_address` | `192.168.1.200` | IP of this machine |
| `multicast_address` | `239.255.42.99` | Multicast group (Multicast mode only) |
| `server_command_port` | `1510` | NatNet command port |
| `server_data_port` | `1511` | NatNet data port |
| `namespace` | `optitrack` | Topic namespace prefix |
| `world_frame` | `map` | Global TF reference frame |
| `optitrack_frame` | `optitrack` | OptiTrack TF reference frame |
| `map_xyz` | `[0.0, 0.0, 0.0]` | Static translation: world_frame -> optitrack_frame (meters) |
| `map_rpy` | `[0.0, 0.0, 0.0]` | Static rotation: world_frame -> optitrack_frame |
| `map_rpy_in_degrees` | `false` | If `true`, `map_rpy` values are in degrees |

---

## Requirements

- [OptiTrack Motive](https://optitrack.com/software/motive/) running on another machine, with NatNet streaming enabled and reachable over the network
- ROS 2 Jazzy Jalisco or Humble Hawksbill installed and sourced (at least *ros-jazzy-ros-base* and *ros-dev-tools* packages, [installation guide](https://docs.ros.org/en/jazzy/Installation.html))
- *rosdep* initialized and updated for managing ROS 2 package dependencies ([installation guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html))
- px4_msgs package (forked minimal version available [here](https://github.com/evannsm/px4_msgs))
- mocap_msgs package (available [here](https://github.com/evannsm/mocap_msgs))

> Note: the NatNet SDK is vendored inside this repository; no system-wide NatNet install is needed.

---

## Compatibility

- **ROS 2**: Jazzy Jalisco and Humble Hawksbill
- **OS / arch**: Ubuntu 24.04 and 22.04; `x86_64` tested
- **OptiTrack stack**: Motive with NatNet streaming; NatNet SDK **1.12** (vendored)

---

## Package layout

```text
optitrack4px4/
├── src/
│   ├── communicator.cpp            # optitrack_client – connects via NatNet, converts Y-up ENU -> NED
│   ├── publisher.cpp               # per-rigid-body publisher creation
│   └── utils.cpp                   # frame conversion utilities (Eigen quaternion math)
├── include/optitrack4px4/
│   ├── communicator.hpp
│   ├── publisher.hpp
│   └── utils.hpp
├── launch/
│   ├── client.launch.py                      # optitrack_client only
│   ├── client_and_visual_odometry.launch.py  # client + relay (uses mocap_px4_relays)
│   └── client_vision_full_all.launch.py      # all three nodes (uses mocap_px4_relays)
├── config/
│   └── optitrack4px4_params.yaml             # default parameters
└── NatNetSDK/                                # vendored NatNet SDK 1.12 (headers + libNatNet.so)
```

---

## Building & linking details

- The package is C++17 and uses `ament_cmake`.
- The **NatNet SDK (1.12)** is vendored in `NatNetSDK/` and linked directly, so you don't need a system-wide installation.
- **Eigen3** is required for quaternion math and is installed via rosdep.
- The install step ships `libNatNet.so` and sets RPATH so that runtime lookups succeed without extra `LD_LIBRARY_PATH` setup.

## Troubleshooting / FAQ

**EKF not fusing vision data**: Ensure `EKF2_EV_CTRL` is set to enable position and/or yaw fusion from external vision. Check that `/fmu/in/vehicle_visual_odometry` is being published at the expected rate with `ros2 topic hz`.

**The node can't connect to OptiTrack/Motive**: verify `server_address`, `local_address`, and network reachability (ping); check that Motive is running and NatNet streaming is enabled in Motive's Data Streaming settings. Confirm the command port (`1510`) and data port (`1511`) match Motive's configuration.

**Frames look misaligned**: adjust `map_xyz` / `map_rpy` and confirm radians vs degrees via `map_rpy_in_degrees`.

**Full state relay not publishing**: the gating requires both `/fmu/out/vehicle_odometry` and `/fmu/out/vehicle_local_position` to arrive at >= 50 Hz. Confirm PX4 is running and the DDS/uXRCE bridge is healthy.

**I don't see TF in RViz**: confirm TF display is enabled and the fixed frame matches your global frame (`world_frame`/`optitrack_frame`).

---

### Frames & mapping

- The OptiTrack frame -> world frame mapping is configurable via `world_frame`, `optitrack_frame`, `map_xyz` and `map_rpy`.
- `map_rpy_in_degrees` lets you specify rotations in degrees when convenient.
- Frame IDs for rigid bodies are derived from the names defined in Motive.

> Units follow ROS conventions (positions in meters, rotations in radians) in downstream consumers; ensure your system uses consistent units end-to-end.

---

## License & attribution

- **License:** GNU General Public License v3.0 (GPL-3.0)
