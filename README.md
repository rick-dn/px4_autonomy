# px4_autonomy

A lightweight ROS2 flight controller interface for PX4 drones. Provides a clean abstraction layer between your autonomous systems and PX4 autopilot, enabling reliable offboard control with safety mechanisms.

## Overview

`px4_autonomy` manages the communication between ROS2 applications and PX4 through a centralized vehicle interface. Instead of dealing with PX4's low-level message formats directly, you call simple services to control your drone.

**Status:** Foundation layer - stable and ready for building autonomous applications on top.

## What It Does

- ✅ **Centralized Control** - Single interface to PX4 (no direct topic access needed)
- ✅ **Continuous Heartbeat** - Maintains offboard mode connection reliably
- ✅ **State Management** - Tracks drone state bidirectionally
- ✅ **Service-Based API** - Clean interface for external nodes (AI, navigation, vision, etc.)
- ✅ **Position Control** - Takeoff, goto, land, hover commands
- ✅ **Minimal Logging** - Logs only state changes, monitor via topics

## Quick Start

### Prerequisites
- ROS2 Humble
- PX4 Autopilot (SITL or hardware)
- Micro XRCE-DDS Agent
- px4_msgs package

### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/px4_autonomy.git
cd ~/ros2_ws
colcon build --packages-select px4_autonomy
source install/setup.bash
```

### Package Structure

The `px4_autonomy` package includes:
- **nodes/** - Main executable nodes (vehicle_interface, etc.)
- **control/** - Control modules (position control stable; velocity control in development)
- **msg/** - Custom message definitions
- **config/** - Configuration files

### Run It

**Terminal 1 - PX4 Simulation:**
```bash
cd PX4-Autopilot
make px4_sitl gazebo-classic
```

**Terminal 2 - XRCE-DDS Agent:**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3 - Vehicle Interface Node:**
```bash
ros2 run px4_autonomy vehicle_interface
```

The vehicle_interface node will start, establish heartbeat with PX4, and expose all available services.

## Basic Flight Sequence

```bash
# 1. Arm the drone
ros2 service call /vehicle/arm std_srvs/srv/Trigger

# 2. Switch to offboard mode
ros2 service call /vehicle/set_offboard_mode std_srvs/srv/Trigger

# 3. Takeoff
ros2 service call /vehicle/takeoff std_srvs/srv/Trigger

# 4. Go somewhere
ros2 param set /vehicle_interface goto_x 5.0
ros2 param set /vehicle_interface goto_y 5.0
ros2 param set /vehicle_interface goto_z 3.0
ros2 service call /vehicle/goto std_srvs/srv/Trigger

# 5. Land
ros2 service call /vehicle/land std_srvs/srv/Trigger
```

## Available Services

| Service | Purpose |
|---------|---------|
| `/vehicle/arm` | Arm the drone |
| `/vehicle/disarm` | Disarm the drone |
| `/vehicle/set_offboard_mode` | Switch to offboard mode |
| `/vehicle/takeoff` | Takeoff to altitude |
| `/vehicle/land` | Land and auto-disarm |
| `/vehicle/goto` | Go to position (x, y, z) |
| `/vehicle/hover` | Hold current position |

## Monitoring Topics

```bash
# Vehicle state (position + orientation)
ros2 topic echo /vehicle/state

# Raw PX4 status
ros2 topic echo /fmu/out/vehicle_status_v1

# Raw PX4 position
ros2 topic echo /fmu/out/vehicle_local_position_v1
```

## Integrating with Your Code

Any ROS2 node can control the drone by calling services:

```python
from rclpy.node import Node
from std_srvs.srv import Trigger

class MyAutonomousNode(Node):
    def __init__(self):
        super().__init__('my_autonomous_node')
        self.arm_client = self.create_client(Trigger, '/vehicle/arm')
        self.takeoff_client = self.create_client(Trigger, '/vehicle/takeoff')
    
    def fly(self):
        # Arm
        self.arm_client.call_async(Trigger.Request())
        # Takeoff
        self.takeoff_client.call_async(Trigger.Request())
```

No need to understand PX4 internals - just call services.

## Architecture

```
vehicle_interface.py
├── Heartbeat Publisher (50Hz)
├── Position Setpoint Publisher (10Hz)
├── State Monitor (subscribes to PX4)
└── Service Interface (external control)
```

The node runs continuously, publishing heartbeat signals to keep PX4 in offboard mode. When you call a service, it updates the target position/orientation and publishes setpoints.

## Key Implementation Details

### Coordinate Frame
Uses NED (North-East-Down) frame internally. Altitude parameters are positive (e.g., `z=3.0` means 3m up).

### QoS Settings
```python
reliability=ReliabilityPolicy.BEST_EFFORT,
durability=DurabilityPolicy.TRANSIENT_LOCAL,
history=HistoryPolicy.KEEP_LAST,
depth=1
```

### Heartbeat
- Published at 50Hz continuously
- Required for offboard mode stability
- Automatically managed

## Troubleshooting

**Service not found?**
```bash
ros2 service list | grep vehicle
```

**Drone won't takeoff?**
- Check heartbeat: `ros2 topic hz /fmu/in/offboard_control_mode`
- Verify armed: `ros2 topic echo /fmu/out/vehicle_status_v1 --field arming_state --once`

**Can't disarm while hovering?**
- Normal - PX4 safety prevents this in offboard mode
- Use `/vehicle/land` service instead

## What's Next

This is a foundation layer. Currently stable features:
- ✅ Position control (takeoff, goto, land, hover)

In development:
- 🔄 Velocity control - Speed-based movement commands

Future features:
- **AI Agents** - Call services to execute autonomous behaviors
- **SLAM/Navigation** - Add perception and path planning
- **Computer Vision** - Process camera streams for object detection
- **Attitude Control** - Direct orientation control mode

## Documentation

For detailed information, see the [Wiki](../../wiki) (coming soon).

## License

MIT

## Contributing

This is a foundation project. Pull requests welcome!

---

**Getting off the ground** - reliably and safely.
