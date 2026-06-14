# Drive Safety Gateway

A ROS 2 safety arbitration node that sits between an autonomous navigation stack and the drive hardware.

The node receives planner velocity commands, applies safety rules, and publishes the final velocity command that reaches the motors.

This implementation was developed as part of the Peer Robotics Robotics Software Engineer assignment.

---

# Features

Implementation requirements:

1. Emergency stop (highest priority)

2. Obstacle-based velocity slowdown

3. Hard stop for close obstacles

4. Planner command timeout protection

5. Velocity clamping

6. Fixed-rate publishing (20 Hz)

7. ROS parameters

8. Automated unit tests

9. QoS configuration

---

# Architecture

The package is intentionally split into two layers.

## 1. Core Decision Logic

File:

```text
drive_safety_gateway/gateway_logic.py
```

Contains:

- GatewayLogic
- GatewayParams
- VelocityCommand
- GatewayState

This layer contains all arbitration decisions and can be tested without ROS.

Responsibilities:

- E-stop handling
- Obstacle slowdown
- Timeout handling
- Velocity limiting
- State transitions

---

## 2. ROS Interface Layer

File:

```text
drive_safety_gateway/gateway_node.py
```

Responsibilities:

- ROS publishers
- ROS subscribers
- Parameter handling
- QoS configuration
- Fixed-rate timer publishing

The node simply gathers ROS inputs and forwards them to GatewayLogic.

This separation keeps the logic testable and independent of the ROS graph.

---

# Arbitration Priority

The following priority order is implemented:

```text
ESTOP
  ↓
TIMEOUT
  ↓
BLOCKED
  ↓
SLOW
  ↓
NORMAL
```

Examples:

### ESTOP + Obstacle

Result:

```text
ESTOP
```

### ESTOP + TIMEOUT

Result:

```text
ESTOP
```

### TIMEOUT + BLOCKED

Result:

```text
TIMEOUT
```

This ensures deterministic behavior when multiple safety conditions occur simultaneously.

---

# Topics

## Subscribed Topics

### Planner Command

```text
/nav/cmd_vel
```

Type:

```text
geometry_msgs/msg/Twist
```

---

### Emergency Stop

```text
/safety/estop
```

Type:

```text
std_msgs/msg/Bool
```

---

### Minimum Obstacle Distance

```text
/safety/min_obstacle_dist
```

Type:

```text
std_msgs/msg/Float32
```

---

## Published Topics

### Final Drive Command

```text
/drive/cmd_vel
```

Type:

```text
geometry_msgs/msg/Twist
```

---

### Gateway State

```text
/drive/gateway_state
```

Type:

```text
std_msgs/msg/String
```

Possible values:

```text
NORMAL
SLOW
BLOCKED
TIMEOUT
ESTOP
```

---

# Parameters

| Parameter | Default |
|------------|----------|
| slow_distance | 1.0 |
| stop_distance | 0.3 |
| timeout_sec | 0.5 |
| publish_rate | 20.0 |
| max_linear_vel | 1.0 |
| max_angular_vel | 1.5 |

Example:

```bash
ros2 launch drive_safety_gateway gateway.launch.py
```

---

# QoS Choices

The node uses explicit QoS profiles instead of default queue depths.

## Sensor Inputs

Used for:

```text
/safety/estop
/safety/min_obstacle_dist
```

Configuration:

```python
ReliabilityPolicy.RELIABLE
HistoryPolicy.KEEP_LAST
depth=10
```

Reason:

Safety-related messages should not be dropped.

---

## Planner Velocity Commands

Used for:

```text
/nav/cmd_vel
```

Configuration:

```python
ReliabilityPolicy.RELIABLE
HistoryPolicy.KEEP_LAST
depth=10
```

Reason:

Velocity commands should arrive reliably while avoiding excessive queue growth.

---

# Build

```bash
Pre-requisite:
  - Ubuntu 22.04
  - ROS 2 Humble
  - Python 3.10+
  - colcon build tools

Clone the repository:
    mkdir -p peer_robot_ws/src
    cd ~/peer_robot_ws/src

    git clone https://github.com/prashant-022/peer_robotics_safety_drive_gateway.git

Build the workspace:
    cd ~/peer_robot_ws

    source /opt/ros/humble/setup.bash

    colcon build --symlink-install

Source the package:
    source /opt/ros/humble/setup.bash
    source ~/peer_robot_ws/install/setup.bash
```

---

# Run

```bash
ros2 launch drive_safety_gateway gateway.launch.py
```

---

# Manual Verification

## Check Available Topics

```bash
ros2 topic list
```

Screenshot:

```text
media/available_topics.png
```

---

## Test 1 - TIMEOUT

Launch node and do not publish any planner command.

Expected:

```text
State = TIMEOUT
Velocity = 0
```

Screenshot:

```text
media/timeout.png
```

---

## Test 2 - NORMAL

Publish:

```bash
ros2 topic pub /nav/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.8}, angular: {z: 0.2}}"
```

Expected:

```text
State = NORMAL
```

Video:

```text
media/normal.webm
```

---

## Test 3 - SLOW

Publish:

```bash
ros2 topic pub /safety/min_obstacle_dist std_msgs/msg/Float32 \
"{data: 0.65}"
```

Expected:

```text
State = SLOW
Linear velocity scaled down
```

Video:

```text
media/slow.webm
```

---

## Test 4 - BLOCKED

Publish:

```bash
ros2 topic pub /safety/min_obstacle_dist std_msgs/msg/Float32 \
"{data: 0.2}"
```

Expected:

```text
State = BLOCKED
Linear velocity = 0
Angular velocity allowed
```

Video:

```text
media/blocked.webm
```

---

## Test 5 - ESTOP

Publish:

```bash
ros2 topic pub /safety/estop std_msgs/msg/Bool \
"{data: true}"
```

Expected:

```text
State = ESTOP
Linear velocity = 0
Angular velocity = 0
```

Video:

```text
media/estop_true.webm
```

Reset:

```bash
ros2 topic pub /safety/estop std_msgs/msg/Bool \
"{data: false}"
```

---

# Unit Tests

Run:

```bash
python3 -m pytest test/test_gateway_logic.py -v
```

Current result:

```text
10 passed
```

Covered cases:

- ESTOP
- TIMEOUT
- BLOCKED
- SLOW
- NORMAL
- Linear velocity clamping
- Angular velocity clamping
- ESTOP > TIMEOUT priority
- ESTOP > BLOCKED priority
- Slowdown scaling

Screenshot:

```text
media/pytest_of_launch_file.png
```

---

# Assumptions

## Rotation During BLOCKED

The assignment states:

```text
Below 0.3 m, linear velocity is zero
(rotation may still be allowed)
```

Implementation:

```text
linear velocity = 0
angular velocity = preserved
```

---

## ESTOP Behaviour

When ESTOP is active:

```text
linear velocity = 0
angular velocity = 0
```

The node resumes operation once:

```text
/safety/estop = false
```

---

# Future Improvements

With additional development time:

- Dynamic parameter updates
- Lifecycle node support
- Diagnostics publisher
- Latched gateway state topic
- Integration tests using launch_testing
- Hardware-in-the-loop validation
- Separate watchdog for sensor timeout monitoring

---

# Package Structure

```text
drive_safety_gateway/

├── drive_safety_gateway
│   ├── gateway_logic.py
│   ├── gateway_node.py
│   └── __init__.py
│
├── launch
│   └── gateway.launch.py
│
├── test
│   ├── test_gateway_logic.py
│   ├── test_flake8.py
│   ├── test_pep257.py
│   └── test_copyright.py
│
├── package.xml
├── setup.py
├── setup.cfg
├── LICENSE
└── README.md
```

---

# Author

Prashant Kumar (Email: kprashant0422@gmail.com)

Robotics Software Engineer Candidate
