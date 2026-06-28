import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from go2_interfaces.msg import Go2State, WebRtcReq
from go2_interfaces.srv import RobotCommand

from .state_machine import RobotState

WALK_SPEED = 0.3   # m/s
TURN_SPEED = 0.5   # rad/s
CMD_HZ     = 10    # cmd_vel publish rate

CMD_RECOVERY_STAND = 1006
CMD_BALANCE_STAND  = 1002
CMD_STAND_DOWN     = 1005
CMD_SIT            = 1009
CMD_RISE_SIT       = 1010
CMD_HELLO          = 1016


class RobotControllerNode(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self._state = RobotState.UNKNOWN
        self._lock  = threading.Lock()

        cb = ReentrantCallbackGroup()

        self._webrtc_pub = self.create_publisher(WebRtcReq, 'webrtc_req', 10)
        self._vel_pub    = self.create_publisher(Twist, 'cmd_vel_out', 10)

        self.create_subscription(
            Go2State, 'go2_states', self._on_go2_state, 10, callback_group=cb)
        self.create_service(
            RobotCommand, 'robot/command', self._on_command, callback_group=cb)

        self.get_logger().info('robot_controller ready')

    # ── telemetry ──────────────────────────────────────────────────────────

    def _on_go2_state(self, msg: Go2State):
        with self._lock:
            if self._state == RobotState.UNKNOWN:
                if any(f != 0 for f in msg.foot_force):
                    self._state = RobotState.STANDING
                else:
                    self._state = RobotState.DAMPED

    # ── service ────────────────────────────────────────────────────────────

    def _on_command(self, req: RobotCommand.Request,
                    res: RobotCommand.Response) -> RobotCommand.Response:
        action = req.action.strip()
        try:
            if   action == 'stand_up':  ok, msg = self._stand_up()
            elif action == 'lay_down':  ok, msg = self._lay_down()
            elif action == 'sit':       ok, msg = self._sit()
            elif action == 'walk':      ok, msg = self._walk(req.x, req.y, req.z, req.value)
            elif action == 'turn':      ok, msg = self._turn(req.value)
            elif action == 'hello':     ok, msg = self._hello()
            elif action == 'get_state': ok, msg = True, 'ok'
            else:                       ok, msg = False, f"unknown action '{action}'"
        except Exception as exc:
            self.get_logger().error(f'{action} raised: {exc}')
            ok, msg = False, str(exc)

        res.success = ok
        res.message = msg
        with self._lock:
            res.robot_state = self._state.value
        return res

    # ── helpers ────────────────────────────────────────────────────────────

    def _webrtc(self, api_id: int):
        msg = WebRtcReq()
        msg.api_id    = api_id
        msg.topic     = 'rt/api/sport/request'
        msg.parameter = str(api_id)  # firmware requires non-empty parameter
        self._webrtc_pub.publish(msg)
        self.get_logger().info(f'→ webrtc api_id={api_id}')

    def _vel_loop(self, x: float, y: float, z: float, duration: float):
        twist = Twist()
        twist.linear.x  = x
        twist.linear.y  = y
        twist.angular.z = z
        n = max(int(duration * CMD_HZ), 1)
        for _ in range(n):
            self._vel_pub.publish(twist)
            time.sleep(1.0 / CMD_HZ)
        self._vel_pub.publish(Twist())  # stop

    # ── actions ────────────────────────────────────────────────────────────

    def _stand_up(self):
        with self._lock:
            state = self._state

        if state == RobotState.STANDING:
            return True, 'already standing'

        if state == RobotState.SITTING:
            self._webrtc(CMD_RISE_SIT)
            time.sleep(3.0)

        self._webrtc(CMD_RECOVERY_STAND)
        time.sleep(4.0)
        self._webrtc(CMD_BALANCE_STAND)
        time.sleep(1.0)

        with self._lock:
            self._state = RobotState.STANDING
        return True, 'standing'

    def _lay_down(self):
        with self._lock:
            state = self._state

        if state == RobotState.DAMPED:
            return True, 'already lying down'

        if state != RobotState.STANDING:
            ok, msg = self._stand_up()
            if not ok:
                return False, f'could not stand: {msg}'

        self._webrtc(CMD_STAND_DOWN)
        time.sleep(3.0)

        with self._lock:
            self._state = RobotState.DAMPED
        return True, 'lying down'

    def _sit(self):
        with self._lock:
            state = self._state

        if state == RobotState.SITTING:
            return True, 'already sitting'

        if state != RobotState.STANDING:
            ok, msg = self._stand_up()
            if not ok:
                return False, f'could not stand: {msg}'

        self._webrtc(CMD_SIT)
        time.sleep(2.0)

        with self._lock:
            self._state = RobotState.SITTING
        return True, 'sitting'

    def _walk(self, x: float, y: float, z: float, duration: float):
        with self._lock:
            state = self._state

        if state != RobotState.STANDING:
            ok, msg = self._stand_up()
            if not ok:
                return False, f'could not stand: {msg}'

        if duration <= 0:
            duration = 1.0

        with self._lock:
            self._state = RobotState.MOVING

        self._vel_loop(x, y, z, duration)

        with self._lock:
            self._state = RobotState.STANDING
        return True, f'walked {duration:.2f}s (x={x:.2f} y={y:.2f})'

    def _turn(self, degrees: float):
        with self._lock:
            state = self._state

        if state != RobotState.STANDING:
            ok, msg = self._stand_up()
            if not ok:
                return False, f'could not stand: {msg}'

        radians   = abs(degrees) * math.pi / 180.0
        duration  = radians / TURN_SPEED
        angular_z = TURN_SPEED if degrees >= 0 else -TURN_SPEED

        with self._lock:
            self._state = RobotState.MOVING

        self._vel_loop(0.0, 0.0, angular_z, duration)

        with self._lock:
            self._state = RobotState.STANDING
        direction = 'left' if degrees >= 0 else 'right'
        return True, f'turned {abs(degrees):.1f}° {direction}'

    def _hello(self):
        with self._lock:
            state = self._state

        if state != RobotState.STANDING:
            ok, msg = self._stand_up()
            if not ok:
                return False, f'could not stand: {msg}'

        self._webrtc(CMD_HELLO)
        time.sleep(3.0)
        return True, 'said hello'


def main():
    rclpy.init()
    node = RobotControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
