#!/usr/bin/env python3
"""MCP server for Go2 robot — exposes RobotAPI methods as LLM-callable tools."""

from mcp.server.fastmcp import FastMCP

from .robot_api import RobotAPI

mcp = FastMCP(
    "go2-robot",
    instructions=(
        "Control a Unitree Go2 quadruped robot. "
        "The robot must be standing before walking or turning — "
        "stand_up() handles any starting position automatically. "
        "Use get_state() if unsure of current position."
    ),
)

_api: RobotAPI | None = None


def _get_api() -> RobotAPI:
    global _api
    if _api is None:
        _api = RobotAPI(node_name="mcp_robot_api")
    return _api


@mcp.tool()
def get_state() -> str:
    """Get the robot's current state (standing, sitting, lying down, or moving)."""
    return _get_api().get_state()


@mcp.tool()
def stand_up() -> str:
    """Make the robot stand up. Works from any position: lying, sitting, or already standing."""
    return _get_api().stand_up()


@mcp.tool()
def sit() -> str:
    """Make the robot sit. Automatically stands up first if currently lying down."""
    return _get_api().sit()


@mcp.tool()
def lay_down() -> str:
    """Make the robot lie down flat."""
    return _get_api().lay_down()


@mcp.tool()
def walk_forward(distance_meters: float = 0.3) -> str:
    """Walk forward. distance_meters=0.3 is roughly 1 foot. Stands up first if needed."""
    return _get_api().walk_forward(distance_meters)


@mcp.tool()
def walk_backward(distance_meters: float = 0.3) -> str:
    """Walk backward. distance_meters=0.3 is roughly 1 foot. Stands up first if needed."""
    return _get_api().walk_backward(distance_meters)


@mcp.tool()
def strafe_left(distance_meters: float = 0.3) -> str:
    """Sidestep left without turning. Stands up first if needed."""
    return _get_api().strafe_left(distance_meters)


@mcp.tool()
def strafe_right(distance_meters: float = 0.3) -> str:
    """Sidestep right without turning. Stands up first if needed."""
    return _get_api().strafe_right(distance_meters)


@mcp.tool()
def turn_left(degrees: float = 90.0) -> str:
    """Turn left (counterclockwise) by the given degrees. Stands up first if needed."""
    return _get_api().turn_left(degrees)


@mcp.tool()
def turn_right(degrees: float = 90.0) -> str:
    """Turn right (clockwise) by the given degrees. Stands up first if needed."""
    return _get_api().turn_right(degrees)


@mcp.tool()
def hello() -> str:
    """Make the robot wave hello. Stands up first if needed."""
    return _get_api().hello()


@mcp.tool()
def speak(text: str) -> str:
    """
    Make the robot say something aloud via ElevenLabs TTS.
    Requires speech_processor tts_node running with ELEVENLABS_API_KEY set.
    """
    return _get_api().speak(text)


@mcp.tool()
def get_detections() -> list:
    """
    Return the most recent COCO object detections from the front camera.
    Each item has: class_id, score, center_x, center_y, width, height.
    Returns [] if coco_detector_node is not running or camera is inactive.
    """
    return _get_api().get_detections()


@mcp.tool()
def get_odometry() -> dict:
    """
    Return the robot's current position and orientation from odometry.
    Fields: position {x,y,z} metres, orientation {x,y,z,w} quaternion.
    """
    return _get_api().get_odometry()


@mcp.tool()
def get_imu() -> dict:
    """
    Return latest IMU data: quaternion, accelerometer, gyroscope, rpy, temperature.
    """
    return _get_api().get_imu()


@mcp.tool()
def send_command(api_id: int, parameter: str = "") -> str:
    """
    Send a raw sport-mode command by api_id (escape hatch).
    Use for unwrapped commands: Dance1=1022, Stretch=1017, WiggleHips=1033, etc.
    See robot_commands.py for the full ROBOT_CMD dict.
    """
    return _get_api().send_command(api_id, parameter or None)


def main():
    mcp.run()


if __name__ == "__main__":
    main()
