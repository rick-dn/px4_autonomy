# px4_autonomy

A lightweight ROS2 flight controller interface for PX4 drones. Provides centralized offboard control with clean service-based API.

![Visitor Count](https://hits.seeyou.design/pages/view/rick-dn.px4_autonomy?style=for-the-badge&color=blue)

## Version History

### v0.1.0
- Initial release
- Single monolithic `vehicle_interface` node
- Position control: takeoff, goto, land, hover
- Services: arm, disarm, offboard mode, land

### v0.2.0 (Current)
- **Breaking Changes:**
  - Separated `vehicle_interface` into modular components
  - `PositionController` handles all position-based control
  - `VelocityController` for velocity-based movement
  
- **New Features:**
  - Mode switching between position and velocity control
  - Velocity control with parameters: vx, vy, vz, vyaw
  - Dynamic mode switching via `_switch_mode()`
  - RTL (Return to Launch) service
  
- **Known Issues:**
  - ⚠️ **Bug:** Previous position parameters (goto_x, goto_y, goto_z, goto_yaw) not cleared after velocity control
    - **Workaround:** Check current parameters before goto: `ros2 param get /vehicle_interface goto_x`
    - Vehicle will navigate to previously set position if not updated
    - Will be fixed in v0.3.0

### Future (v0.3.0+)
- Parameter auto-reset on mode switch
- Trajectory following
- Attitude control mode
- Enhanced state monitoring

## Prerequisites

- ROS2 Humble
- PX4 Autopilot (SITL or hardware)
- Micro XRCE-DDS Agent running
- px4_msgs package

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/px4_autonomy.git
cd ~/ros2_ws
colcon build --packages-select px4_autonomy
source install/setup.bash
```

## Running px4_autonomy

### Terminal 1 - PX4 Simulation

```bash
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic
```

### Terminal 2 - XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 3 - Launch px4_autonomy

```bash
ros2 launch px4_autonomy offboard.launch
```

## Basic Flight Sequence

```bash
# 1. Arm the drone
ros2 service call /vehicle/arm std_srvs/srv/Trigger

# 2. Switch to offboard mode
ros2 service call /vehicle/set_offboard_mode std_srvs/srv/Trigger

# 3. Takeoff
ros2 service call /vehicle/takeoff std_srvs/srv/Trigger

# 4. Set parameters and go to position
ros2 param set /vehicle_interface goto_x 5.0
ros2 param set /vehicle_interface goto_y 5.0
ros2 param set /vehicle_interface goto_z 3.0
ros2 service call /vehicle/goto std_srvs/srv/Trigger

# 5. Hover at current position
ros2 service call /vehicle/hover std_srvs/srv/Trigger

# 6. Land
ros2 service call /vehicle/land std_srvs/srv/Trigger
```

## Available Services

| Service | Parameters | Description |
|---------|-----------|-------------|
| `/vehicle/arm` | - | Arm the drone |
| `/vehicle/disarm` | - | Disarm the drone |
| `/vehicle/set_offboard_mode` | - | Switch to offboard mode |
| `/vehicle/takeoff` | `takeoff_altitude` (default: 2.5m) | Takeoff to altitude |
| `/vehicle/goto` | `goto_x`, `goto_y`, `goto_z`, `goto_yaw` | Go to position |
| `/vehicle/hover` | - | Hold current position |
| `/vehicle/land` | - | Land and auto-disarm |
| `/vehicle/rtl` | - | Return to launch |
| `/vehicle/velocity_start` | `vx`, `vy`, `vz`, `vyaw` | Start velocity control |
| `/vehicle/velocity_stop` | - | Stop velocity control |

## Setting Parameters

```bash
# Position control parameters
ros2 param set /vehicle_interface takeoff_altitude 3.5
ros2 param set /vehicle_interface goto_x 10.0
ros2 param set /vehicle_interface goto_y 10.0
ros2 param set /vehicle_interface goto_z 5.0
ros2 param set /vehicle_interface goto_yaw 0.0

# Velocity control parameters
ros2 param set /vehicle_interface vx 2.0
ros2 param set /vehicle_interface vy 1.0
ros2 param set /vehicle_interface vz 0.0
ros2 param set /vehicle_interface vyaw 0.5
```

## Monitoring

```bash
# Vehicle state (position + orientation)
ros2 topic echo /vehicle/state

# Available services
ros2 service list | grep vehicle

# Topic frequency
ros2 topic hz /fmu/in/offboard_control_mode
```

## Integrating with Your Code

```python
from rclpy.node import Node
from std_srvs.srv import Trigger

class MyAutonomousNode(Node):
    def __init__(self):
        super().__init__('my_autonomous_node')
        self.arm_client = self.create_client(Trigger, '/vehicle/arm')
        self.takeoff_client = self.create_client(Trigger, '/vehicle/takeoff')
    
    async def fly(self):
        await self.arm_client.call_async(Trigger.Request())
        await self.takeoff_client.call_async(Trigger.Request())
```

## Troubleshooting

**Services not found?**
```bash
ros2 service list | grep vehicle
```

**Drone won't takeoff?**
- Verify heartbeat: `ros2 topic hz /fmu/in/offboard_control_mode`
- Check if armed: `ros2 topic echo /fmu/out/vehicle_status_v1 --field arming_state --once`
- Check if in offboard mode: `ros2 topic echo /fmu/out/vehicle_status_v1 --field nav_state --once`

**QoS mismatch warning?**
- Ensure all subscribers use `get_px4_qos_profile()` for PX4 topics

---

**Clean abstraction between your autonomous systems and PX4.**

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International License (CC-BY-NC-4.0)**.

You are free to:
- Share and redistribute the material
- Adapt, remix, and build upon the material

**Under the following terms:**
- **Attribution** — You must give appropriate credit to the original authors
- **NonCommercial** — You may not use this material for commercial purposes

For full license details, see the [LICENSE](LICENSE) file or visit [CC-BY-NC-4.0](https://creativecommons.org/licenses/by-nc/4.0/).
