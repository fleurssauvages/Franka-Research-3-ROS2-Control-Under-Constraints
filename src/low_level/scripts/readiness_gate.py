#!/usr/bin/env python3

import time
from typing import Dict, List

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


class ReadinessGate(Node):
    def __init__(self) -> None:
        super().__init__('readiness_gate')

        self.declare_parameter('controller_manager_service', 'controller_manager/list_controllers')
        self.declare_parameter('required_controllers', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('required_publisher_topics', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('required_subscriber_topics', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('required_true_topics', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('stable_time', 0.25)
        self.declare_parameter('warn_after', 5.0)
        self.declare_parameter('check_period', 0.10)

        self.controller_manager_service = str(
            self.get_parameter('controller_manager_service').value
        )
        self.required_controllers = self._optional_string_array('required_controllers')
        self.required_publisher_topics = self._optional_string_array(
            'required_publisher_topics'
        )
        self.required_subscriber_topics = self._optional_string_array(
            'required_subscriber_topics'
        )
        self.required_true_topics = self._optional_string_array('required_true_topics')
        self.stable_time = float(self.get_parameter('stable_time').value)
        self.warn_after = float(self.get_parameter('warn_after').value)
        self.check_period = float(self.get_parameter('check_period').value)

        if self.stable_time < 0.0:
            raise ValueError('stable_time must be >= 0')
        if self.check_period <= 0.0:
            raise ValueError('check_period must be > 0')

        self.started_at = time.monotonic()
        self.ready_since = None
        self.last_warn = 0.0
        self.controller_states: Dict[str, str] = {}
        self.true_topic_values: Dict[str, bool] = {
            topic: False for topic in self.required_true_topics
        }

        self.list_client = None
        self.list_future = None
        if self.required_controllers:
            self.list_client = self.create_client(ListControllers, self.controller_manager_service)

        self.bool_subscriptions = []
        for topic in self.required_true_topics:
            qos = rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            )
            sub = self.create_subscription(
                Bool,
                topic,
                lambda msg, topic_name=topic: self._on_bool(topic_name, msg),
                qos,
            )
            self.bool_subscriptions.append(sub)

        self.timer = self.create_timer(self.check_period, self._check)


    def _optional_string_array(self, name: str) -> List[str]:
        """Return an optional STRING_ARRAY parameter, defaulting to an empty list.

        rclpy parameters declared with only Parameter.Type.STRING_ARRAY are
        intentionally uninitialized until a launch/YAML override supplies a
        value.  Most readiness conditions are optional, so an omitted one must
        mean "no requirement" rather than raising ParameterUninitializedException.
        """
        try:
            value = self.get_parameter(name).value
        except ParameterUninitializedException:
            return []
        if value is None:
            return []
        return [str(item) for item in value]

    def _on_bool(self, topic: str, msg: Bool) -> None:
        self.true_topic_values[topic] = bool(msg.data)

    def _request_controller_states(self) -> None:
        if self.list_client is None:
            return
        if self.list_future is not None and not self.list_future.done():
            return
        if not self.list_client.service_is_ready():
            self.list_client.wait_for_service(timeout_sec=0.0)
            return
        self.list_future = self.list_client.call_async(ListControllers.Request())

    def _consume_controller_states(self) -> None:
        if self.list_future is None or not self.list_future.done():
            return
        try:
            response = self.list_future.result()
            self.controller_states = {
                controller.name: controller.state for controller in response.controller
            }
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'list_controllers failed: {exc}')
        self.list_future = None

    def _missing(self) -> List[str]:
        missing: List[str] = []

        for name in self.required_controllers:
            state = self.controller_states.get(name)
            if state != 'active':
                missing.append(f'controller {name}=active (currently {state or "unknown"})')

        for topic in self.required_publisher_topics:
            if self.count_publishers(topic) < 1:
                missing.append(f'publisher on {topic}')

        for topic in self.required_subscriber_topics:
            if self.count_subscribers(topic) < 1:
                missing.append(f'subscriber on {topic}')

        for topic in self.required_true_topics:
            if not self.true_topic_values.get(topic, False):
                missing.append(f'{topic}=true')

        return missing

    def _check(self) -> None:
        self._request_controller_states()
        self._consume_controller_states()

        missing = self._missing()
        now = time.monotonic()

        if missing:
            self.ready_since = None
            if self.warn_after > 0.0 and now - self.started_at >= self.warn_after:
                if now - self.last_warn >= 2.0:
                    self.get_logger().warn('Waiting for readiness: ' + '; '.join(missing))
                    self.last_warn = now
            return

        if self.ready_since is None:
            self.ready_since = now
            return

        if now - self.ready_since < self.stable_time:
            return

        self.get_logger().info('Readiness conditions satisfied; releasing dependent startup.')
        self.timer.cancel()
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = ReadinessGate()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
