from enum import Enum
from dataclasses import dataclass


class GatewayState(Enum):
    NORMAL = "NORMAL"
    SLOW = "SLOW"
    BLOCKED = "BLOCKED"
    TIMEOUT = "TIMEOUT"
    ESTOP = "ESTOP"


@dataclass
class GatewayParams:
    slow_distance: float = 1.0
    stop_distance: float = 0.3
    timeout_sec: float = 0.5

    max_linear_vel: float = 1.0
    max_angular_vel: float = 1.5


@dataclass
class VelocityCommand:
    linear_x: float
    angular_z: float


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


class GatewayLogic:
    def __init__(self, params: GatewayParams):
        self.params = params

    def _apply_limits(self, cmd: VelocityCommand):

        return VelocityCommand(
            linear_x=clamp(
                cmd.linear_x, -self.params.max_linear_vel, self.params.max_linear_vel
            ),
            angular_z=clamp(
                cmd.angular_z, -self.params.max_angular_vel, self.params.max_angular_vel
            ),
        )

    def evaluate(
        self,
        cmd: VelocityCommand,
        estop: bool,
        obstacle_distance: float,
        time_since_last_cmd: float,
    ):

        # Priority - 1: ESTOP
        if estop:
            return (VelocityCommand(0.0, 0.0), GatewayState.ESTOP)

        # Priority - 2: TIMEOUT
        if time_since_last_cmd > self.params.timeout_sec:
            return (VelocityCommand(0.0, 0.0), GatewayState.TIMEOUT)

        # Priority - 3: BLOCKED
        if obstacle_distance < self.params.stop_distance:
            blocked_cmd = VelocityCommand(linear_x=0.0, angular_z=cmd.angular_z)

            return (self._apply_limits(blocked_cmd), GatewayState.BLOCKED)

        # Priority - 4: SLOW
        if obstacle_distance < self.params.slow_distance:

            scale = (obstacle_distance - self.params.stop_distance) / (
                self.params.slow_distance - self.params.stop_distance
            )

            slow_cmd = VelocityCommand(
                linear_x=cmd.linear_x * scale, angular_z=cmd.angular_z
            )

            return (self._apply_limits(slow_cmd), GatewayState.SLOW)

        # Priority - 5: Normal:
        return (self._apply_limits(cmd), GatewayState.NORMAL)
