import pytest

from drive_safety_gateway.gateway_logic import (
    GatewayLogic,
    GatewayParams,
    GatewayState,
    VelocityCommand,
)


@pytest.fixture
def logic():
    return GatewayLogic(GatewayParams())


def test_estop_has_highest_priority(logic):

    cmd, state = logic.evaluate(
        VelocityCommand(1.0, 0.5),
        estop=True,
        obstacle_distance=0.1,
        time_since_last_cmd=10.0,
    )

    assert state == GatewayState.ESTOP
    assert cmd.linear_x == 0.0
    assert cmd.angular_z == 0.0


def test_timeout(logic):

    cmd, state = logic.evaluate(
        VelocityCommand(1.0, 0.5),
        estop=False,
        obstacle_distance=5.0,
        time_since_last_cmd=1.0,
    )

    assert state == GatewayState.TIMEOUT
    assert cmd.linear_x == 0.0
    assert cmd.angular_z == 0.0


def test_blocked(logic):

    cmd, state = logic.evaluate(
        VelocityCommand(1.0, 0.5),
        estop=False,
        obstacle_distance=0.2,
        time_since_last_cmd=0.1,
    )

    assert state == GatewayState.BLOCKED
    assert cmd.linear_x == 0.0

    # rotation allowed
    assert cmd.angular_z == 0.5


def test_slow(logic):

    cmd, state = logic.evaluate(
        VelocityCommand(1.0, 0.5),
        estop=False,
        obstacle_distance=0.65,
        time_since_last_cmd=0.1,
    )

    assert state == GatewayState.SLOW

    # should be scaled
    assert 0.0 < cmd.linear_x < 1.0

    # angular unchanged
    assert cmd.angular_z == 0.5


def test_normal(logic):

    cmd, state = logic.evaluate(
        VelocityCommand(0.8, 0.2),
        estop=False,
        obstacle_distance=2.0,
        time_since_last_cmd=0.1,
    )

    assert state == GatewayState.NORMAL
    assert cmd.linear_x == 0.8
    assert cmd.angular_z == 0.2


def test_linear_velocity_clamping(logic):

    cmd, state = logic.evaluate(
        VelocityCommand(10.0, 0.0),
        estop=False,
        obstacle_distance=5.0,
        time_since_last_cmd=0.1,
    )

    assert state == GatewayState.NORMAL
    assert cmd.linear_x == 1.0


def test_angular_velocity_clamping(logic):

    cmd, state = logic.evaluate(
        VelocityCommand(0.0, 10.0),
        estop=False,
        obstacle_distance=5.0,
        time_since_last_cmd=0.1,
    )

    assert state == GatewayState.NORMAL
    assert cmd.angular_z == 1.5


def test_estop_priority_over_timeout(logic):

    cmd, state = logic.evaluate(
        VelocityCommand(1.0, 0.5),
        estop=True,
        obstacle_distance=5.0,
        time_since_last_cmd=100.0,
    )

    assert state == GatewayState.ESTOP


def test_estop_priority_over_blocked(logic):

    cmd, state = logic.evaluate(
        VelocityCommand(1.0, 0.5),
        estop=True,
        obstacle_distance=0.1,
        time_since_last_cmd=0.1,
    )

    assert state == GatewayState.ESTOP


def test_slowdown_scale(logic):
    cmd, state = logic.evaluate(
        VelocityCommand(1.0, 0.5),
        estop=False,
        obstacle_distance=0.65,
        time_since_last_cmd=0.1,
    )

    assert state == GatewayState.SLOW
    assert cmd.linear_x == pytest.approx(0.5)
