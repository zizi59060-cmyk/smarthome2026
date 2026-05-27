from __future__ import annotations

import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from example_interfaces.srv import SetBool

from smarthome_common_interfaces.msg import CommFrame, LowerState, ObjectTarget
from smarthome_common_interfaces.srv import ArmCommand

from .protocol import (
    CmdId,
    Frame,
    FrameParser,
    pack_frame,
    pack_chassis_vel,
    pack_object_target,
    pack_gripper,
    pack_estop,
    pack_arm_command,
    pack_nav_event,
    unpack_lower_state,
    unpack_ack,
)
from .serial_transport import SerialTransport


class UnifiedCommNode(Node):
    def __init__(self) -> None:
        super().__init__("smarthome_comm_node")

        self.declare_parameter("serial_device", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("fake_mode", True)
        self.declare_parameter("read_hz", 200.0)
        self.declare_parameter("heartbeat_hz", 2.0)
        self.declare_parameter("serial_reconnect_interval", 1.0)
        self.declare_parameter("warn_disconnected_hz", 0.5)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("object_target_topic", "/smarthome/object_target")
        self.declare_parameter("task_state_topic", "/smarthome/task/state")
        self.declare_parameter("nav_reached_zone_topic", "/smarthome/navigation/reached_zone")
        self.declare_parameter("lower_state_topic", "/smarthome/lower_state")
        self.declare_parameter("raw_rx_topic", "/smarthome/comm/raw_rx")
        self.declare_parameter("raw_tx_topic", "/smarthome/comm/raw_tx")
        self.declare_parameter("enable_cmd_vel", True)
        self.declare_parameter("enable_object_target", True)
        self.declare_parameter("enable_nav_event", True)
        self.declare_parameter("zone_sequence", ["A", "B", "C", "D", "E", "F"])

        self.seq = 0
        self.rx_count = 0
        self.tx_count = 0
        self.drop_count = 0
        self.parser = FrameParser()
        self._last_drop_warn_time = 0.0
        self._was_connected = False
        self._last_nav_event_key = None
        self.zone_ids = self.build_zone_id_map()

        fake_mode = bool(self.get_parameter("fake_mode").value)
        device = str(self.get_parameter("serial_device").value)
        baudrate = int(self.get_parameter("baudrate").value)
        reconnect_interval = float(self.get_parameter("serial_reconnect_interval").value)
        self.transport = SerialTransport(
            device=device,
            baudrate=baudrate,
            fake=fake_mode,
            reconnect_interval=reconnect_interval,
        )

        opened = self.transport.open()
        self._was_connected = self.transport.is_open
        if fake_mode:
            self.get_logger().info("unified comm started in FAKE mode")
        elif opened:
            self.get_logger().info(f"unified comm connected: {device}@{baudrate}")
        else:
            self.get_logger().warn(
                f"serial not connected yet: {device}@{baudrate}; "
                f"will retry every {reconnect_interval:.1f}s "
                f"({self.transport.last_error})"
            )

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.raw_rx_pub = self.create_publisher(CommFrame, str(self.get_parameter("raw_rx_topic").value), qos)
        self.raw_tx_pub = self.create_publisher(CommFrame, str(self.get_parameter("raw_tx_topic").value), qos)
        self.lower_state_pub = self.create_publisher(LowerState, str(self.get_parameter("lower_state_topic").value), qos)
        self.event_pub = self.create_publisher(String, "/smarthome/comm/event", qos)

        if bool(self.get_parameter("enable_cmd_vel").value):
            self.cmd_vel_sub = self.create_subscription(
                Twist,
                str(self.get_parameter("cmd_vel_topic").value),
                self.on_cmd_vel,
                qos,
            )
        if bool(self.get_parameter("enable_object_target").value):
            self.target_sub = self.create_subscription(
                ObjectTarget,
                str(self.get_parameter("object_target_topic").value),
                self.on_object_target,
                qos,
            )
        if bool(self.get_parameter("enable_nav_event").value):
            self.task_state_sub = self.create_subscription(
                String,
                str(self.get_parameter("task_state_topic").value),
                self.on_task_state,
                qos,
            )
            self.nav_reached_sub = self.create_subscription(
                String,
                str(self.get_parameter("nav_reached_zone_topic").value),
                self.on_nav_reached_zone,
                qos,
            )

        self.gripper_srv = self.create_service(SetBool, "/smarthome/comm/gripper", self.on_gripper)
        self.estop_srv = self.create_service(SetBool, "/smarthome/comm/set_estop", self.on_estop)
        self.arm_srv = self.create_service(ArmCommand, "/smarthome/comm/arm_command", self.on_arm_command)

        read_period = 1.0 / max(1.0, float(self.get_parameter("read_hz").value))
        heartbeat_period = 1.0 / max(0.1, float(self.get_parameter("heartbeat_hz").value))
        reconnect_period = max(0.1, reconnect_interval)
        self.read_timer = self.create_timer(read_period, self.read_serial_once)
        self.heartbeat_timer = self.create_timer(heartbeat_period, self.send_heartbeat)
        self.reconnect_timer = self.create_timer(reconnect_period, self.ensure_serial_connected)
        self.fake_state_timer = self.create_timer(0.5, self.publish_fake_state)

    def destroy_node(self) -> bool:
        self.transport.close()
        return super().destroy_node()

    def _next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFF
        return self.seq

    def build_zone_id_map(self) -> dict[str, int]:
        sequence = [str(x).upper() for x in self.get_parameter("zone_sequence").value]
        return {name: index + 1 for index, name in enumerate(sequence) if name}

    def publish_event(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)

    def ensure_serial_connected(self) -> None:
        if self.transport.fake:
            return
        was_connected = self._was_connected
        connected = self.transport.try_reconnect()
        if connected and not was_connected:
            self.get_logger().info(
                f"serial reconnected: {self.transport.device}@{self.transport.baudrate}"
            )
            self.publish_event("SERIAL_RECONNECTED")
        elif was_connected and not connected:
            self.get_logger().warn(f"serial disconnected: {self.transport.last_error}")
            self.publish_event("SERIAL_DISCONNECTED")
        self._was_connected = connected

    def warn_frame_dropped(self, frame: Frame) -> None:
        self.drop_count += 1
        warn_hz = max(0.1, float(self.get_parameter("warn_disconnected_hz").value))
        now = time.monotonic()
        if now - self._last_drop_warn_time >= 1.0 / warn_hz:
            self._last_drop_warn_time = now
            reason = self.transport.last_error or "serial not connected"
            self.get_logger().warn(
                f"drop tx frame {frame.name}; serial is disconnected ({reason})"
            )
            self.publish_event(f"DROP_TX_{frame.name}")

    def publish_raw(self, frame: Frame, direction: int) -> None:
        msg = CommFrame()
        msg.stamp = self.get_clock().now().to_msg()
        msg.cmd_id = int(frame.cmd_id)
        msg.seq = int(frame.seq)
        msg.direction = int(direction)
        msg.payload = list(frame.payload)
        msg.crc_ok = bool(frame.crc_ok)
        msg.name = frame.name
        if direction == CommFrame.DIRECTION_RX:
            self.raw_rx_pub.publish(msg)
        else:
            self.raw_tx_pub.publish(msg)

    def send_frame(self, cmd_id: int, payload: bytes = b"") -> bool:
        seq = self._next_seq()
        frame = Frame(cmd_id=cmd_id, seq=seq, payload=payload, crc_ok=True)
        raw = pack_frame(cmd_id, seq, payload)
        written = self.transport.write(raw)
        self.ensure_serial_connected()

        if written != len(raw):
            self.warn_frame_dropped(frame)
            return False

        self.tx_count += 1
        self.publish_raw(frame, CommFrame.DIRECTION_TX)
        return True

    def on_cmd_vel(self, msg: Twist) -> None:
        payload = pack_chassis_vel(msg.linear.x, msg.linear.y, msg.angular.z)
        self.send_frame(CmdId.CHASSIS_VEL, payload)

    def on_object_target(self, msg: ObjectTarget) -> None:
        p = msg.pose.pose.position
        payload = pack_object_target(msg.class_id, msg.source, p.x, p.y, p.z, msg.score)
        self.send_frame(CmdId.VISION_TARGET, payload)

    def on_task_state(self, msg: String) -> None:
        zone_name = self.parse_reached_zone(msg.data)
        if zone_name:
            self.send_nav_reached_event(zone_name)

    def on_nav_reached_zone(self, msg: String) -> None:
        zone_name = self.parse_reached_zone(msg.data)
        if zone_name:
            self.send_nav_reached_event(zone_name)
        else:
            self.get_logger().warn(f"ignore unknown reached zone message: {msg.data!r}")

    def parse_reached_zone(self, text: str) -> str | None:
        data = text.strip().upper()
        if data in self.zone_ids:
            return data

        patterns = (
            r"\bNAV_DONE_([A-Z0-9]+)_STATUS_",
            r"\bNAV_REACHED_([A-Z0-9]+)\b",
            r"\bREACHED_([A-Z0-9]+)\b",
            r"\bARRIVED_([A-Z0-9]+)\b",
        )
        for pattern in patterns:
            match = re.search(pattern, data)
            if match and match.group(1) in self.zone_ids:
                return match.group(1)
        return None

    def send_nav_reached_event(self, zone_name: str) -> bool:
        zone = zone_name.upper()
        zone_id = self.zone_ids.get(zone)
        if zone_id is None:
            self.get_logger().warn(f"unknown zone {zone_name!r}; known zones: {sorted(self.zone_ids)}")
            return False

        event_key = zone_id
        if event_key == self._last_nav_event_key:
            return True
        self._last_nav_event_key = event_key

        payload = pack_nav_event(zone_id)
        ok = self.send_frame(CmdId.NAV_EVENT, payload)
        if ok:
            self.publish_event(f"NAV_REACHED_{zone}")
            self.get_logger().info(f"sent NAV_EVENT reached zone={zone} id={zone_id}")
        return ok

    def on_gripper(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        ok = self.send_frame(CmdId.GRIPPER, pack_gripper(request.data))
        response.success = ok
        response.message = "gripper open" if request.data else "gripper close"
        if not ok:
            response.message += "; serial disconnected"
        return response

    def on_estop(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        ok = self.send_frame(CmdId.ESTOP, pack_estop(request.data))
        response.success = ok
        response.message = "estop enabled" if request.data else "estop released"
        if not ok:
            response.message += "; serial disconnected"
        return response

    def on_arm_command(self, request: ArmCommand.Request, response: ArmCommand.Response) -> ArmCommand.Response:
        p = request.target_pose.pose.position
        q = request.target_pose.pose.orientation
        payload = pack_arm_command(
            request.command,
            request.class_id,
            (p.x, p.y, p.z),
            (q.x, q.y, q.z, q.w),
        )
        ok = self.send_frame(CmdId.ARM_COMMAND, payload)
        response.accepted = ok
        response.message = f"arm command sent: command={request.command}, class_id={request.class_id}"
        if not ok:
            response.message = "arm command dropped; serial disconnected"
        return response

    def send_heartbeat(self) -> None:
        now_ms = int(time.time() * 1000) & 0xFFFFFFFF
        self.send_frame(CmdId.HEARTBEAT_TX, now_ms.to_bytes(4, "little"))

    def read_serial_once(self) -> None:
        data = self.transport.read_available()
        self.ensure_serial_connected()
        if not data:
            return
        for frame in self.parser.feed(data):
            self.rx_count += 1
            self.publish_raw(frame, CommFrame.DIRECTION_RX)
            if not frame.crc_ok:
                self.get_logger().warn(f"CRC error on {frame.name}")
                continue
            self.handle_frame(frame)

    def handle_frame(self, frame: Frame) -> None:
        if frame.cmd_id == int(CmdId.LOWER_STATE):
            parsed = unpack_lower_state(frame.payload)
            if parsed is None:
                self.get_logger().warn("LOWER_STATE payload too short")
                return
            msg = LowerState()
            msg.stamp = self.get_clock().now().to_msg()
            msg.mode = int(parsed["mode"])
            msg.estop = bool(parsed["estop"])
            msg.battery_voltage = float(parsed["battery_voltage"])
            msg.battery_current = float(parsed["battery_current"])
            msg.chassis_temp = float(parsed["chassis_temp"])
            msg.error_code = int(parsed["error_code"])
            msg.uptime_ms = int(parsed["uptime_ms"])
            msg.rx_count = int(self.rx_count)
            msg.tx_count = int(self.tx_count)
            msg.text = "real lower state"
            self.lower_state_pub.publish(msg)
        elif frame.cmd_id == int(CmdId.ACK):
            ack = unpack_ack(frame.payload)
            if ack:
                status = "OK" if ack["ok"] else f"ERR reason={ack['reason']}"
                self.get_logger().info(f"ACK for 0x{ack['ack_cmd']:04X}: {status}")
        else:
            self.publish_event(f"rx {frame.name} len={len(frame.payload)}")

    def publish_fake_state(self) -> None:
        if not self.transport.fake:
            return
        msg = LowerState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = 1
        msg.estop = False
        msg.battery_voltage = 24.0
        msg.battery_current = 0.5
        msg.chassis_temp = 35.0
        msg.error_code = 0
        msg.uptime_ms = int(time.time() * 1000) & 0xFFFFFFFF
        msg.rx_count = int(self.rx_count)
        msg.tx_count = int(self.tx_count)
        msg.text = "fake lower state; serial disabled"
        self.lower_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnifiedCommNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
