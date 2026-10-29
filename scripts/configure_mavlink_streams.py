#!/usr/bin/env python3
"""Configure PX4 MAVLink stream intervals after MAVROS is connected."""

import threading
import time

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong, CommandLongRequest


MAV_CMD_SET_MESSAGE_INTERVAL = 511


class StreamConfigurator:
    def __init__(self):
        self._connected = threading.Event()
        self._state_sub = rospy.Subscriber(
            "/mavros/state", State, self._state_callback, queue_size=1
        )

    def _state_callback(self, message):
        if message.connected:
            self._connected.set()

    def wait_for_connection(self, timeout):
        return self._connected.wait(timeout)


def set_interval(proxy, message_id, interval_us, attempts):
    request = CommandLongRequest()
    request.command = MAV_CMD_SET_MESSAGE_INTERVAL
    request.confirmation = 0
    request.param1 = float(message_id)
    request.param2 = float(interval_us)

    for attempt in range(1, attempts + 1):
        try:
            response = proxy(request)
            if response.success:
                rospy.loginfo(
                    "Configured MAVLink message %d interval to %d us",
                    message_id,
                    interval_us,
                )
                return True
            rospy.logwarn(
                "PX4 rejected interval for message %d (result=%d, attempt %d/%d)",
                message_id,
                response.result,
                attempt,
                attempts,
            )
        except rospy.ServiceException as error:
            rospy.logwarn(
                "MAVLink interval service failed for message %d "
                "(attempt %d/%d): %s",
                message_id,
                attempt,
                attempts,
                error,
            )
        time.sleep(1.0)
    return False


def main():
    rospy.init_node("configure_sim_mavlink_streams")
    interval_us = int(rospy.get_param("~interval_us", 10000))
    connection_timeout = float(rospy.get_param("~connection_timeout", 60.0))
    service_timeout = float(rospy.get_param("~service_timeout", 20.0))
    attempts = int(rospy.get_param("~attempts", 5))
    message_ids = rospy.get_param("~message_ids", [105, 31])

    configurator = StreamConfigurator()
    if not configurator.wait_for_connection(connection_timeout):
        rospy.logfatal(
            "MAVROS did not connect within %.1f seconds", connection_timeout
        )
        raise SystemExit(1)

    service_name = "/mavros/cmd/command"
    try:
        rospy.wait_for_service(service_name, timeout=service_timeout)
    except rospy.ROSException as error:
        rospy.logfatal("Service %s is unavailable: %s", service_name, error)
        raise SystemExit(1)

    proxy = rospy.ServiceProxy(service_name, CommandLong)
    failed = [
        int(message_id)
        for message_id in message_ids
        if not set_interval(proxy, int(message_id), interval_us, attempts)
    ]
    if failed:
        rospy.logfatal("Failed to configure MAVLink messages: %s", failed)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
