#!/usr/bin/env python3
"""MCP server for Go2 robot — exposes robot actions as LLM-callable tools."""

import os
import re
import subprocess

from mcp.server.fastmcp import FastMCP

WALK_SPEED = 0.3   # m/s
TURN_SPEED = 0.5   # rad/s
SERVICE    = '/robot/command'
SRV_TYPE   = 'go2_interfaces/srv/RobotCommand'

mcp = FastMCP(
    'go2-robot',
    instructions=(
        'Control a Unitree Go2 quadruped robot. '
        'The robot must be standing before walking or turning — '
        'stand_up() handles any starting position automatically. '
        'Use get_state() if unsure of current position.'
    ),
)


def _call(action: str, x: float = 0.0, y: float = 0.0,
          z: float = 0.0, value: float = 0.0) -> str:
    req = f"{{action: '{action}', x: {x}, y: {y}, z: {z}, value: {value}}}"
    try:
        proc = subprocess.run(
            ['ros2', 'service', 'call', SERVICE, SRV_TYPE, req],
            capture_output=True, text=True, timeout=45,
            env=os.environ.copy(),
        )
        out = proc.stdout + proc.stderr
        m = re.search(
            r"success=(\w+),\s*message='([^']*)',\s*robot_state='([^']*)'", out)
        if m:
            ok    = m.group(1) == 'True'
            msg   = m.group(2)
            state = m.group(3)
            return f"{'OK' if ok else 'FAILED'}: {msg} — robot is {state}"
        return f'service output: {out[:300]}'
    except subprocess.TimeoutExpired:
        return 'Timeout: robot_controller service did not respond within 45s'
    except Exception as exc:
        return f'Error calling service: {exc}'


@mcp.tool()
def get_state() -> str:
    """Get the robot's current state (standing, sitting, lying down, or moving)."""
    return _call('get_state')


@mcp.tool()
def stand_up() -> str:
    """Make the robot stand up. Works from any position: lying, sitting, or already standing."""
    return _call('stand_up')


@mcp.tool()
def sit() -> str:
    """Make the robot sit. Automatically stands up first if currently lying down."""
    return _call('sit')


@mcp.tool()
def lay_down() -> str:
    """Make the robot lie down flat."""
    return _call('lay_down')


@mcp.tool()
def walk_forward(distance_meters: float = 0.3) -> str:
    """Walk forward. distance_meters=0.3 is roughly 1 foot. Stands up first if needed."""
    duration = distance_meters / WALK_SPEED
    return _call('walk', x=WALK_SPEED, value=duration)


@mcp.tool()
def walk_backward(distance_meters: float = 0.3) -> str:
    """Walk backward. distance_meters=0.3 is roughly 1 foot. Stands up first if needed."""
    duration = distance_meters / WALK_SPEED
    return _call('walk', x=-WALK_SPEED, value=duration)


@mcp.tool()
def strafe_left(distance_meters: float = 0.3) -> str:
    """Sidestep left without turning. Stands up first if needed."""
    duration = distance_meters / WALK_SPEED
    return _call('walk', y=WALK_SPEED, value=duration)


@mcp.tool()
def strafe_right(distance_meters: float = 0.3) -> str:
    """Sidestep right without turning. Stands up first if needed."""
    duration = distance_meters / WALK_SPEED
    return _call('walk', y=-WALK_SPEED, value=duration)


@mcp.tool()
def turn_left(degrees: float = 90.0) -> str:
    """Turn left (counterclockwise) by the given degrees. Stands up first if needed."""
    return _call('turn', value=degrees)


@mcp.tool()
def turn_right(degrees: float = 90.0) -> str:
    """Turn right (clockwise) by the given degrees. Stands up first if needed."""
    return _call('turn', value=-degrees)


@mcp.tool()
def hello() -> str:
    """Make the robot wave hello. Stands up first if needed."""
    return _call('hello')


def main():
    mcp.run()


if __name__ == '__main__':
    main()
