import json
import socket
import subprocess
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class WifiConfigApplier(Node):
    def __init__(self):
        super().__init__("wifi_config_applier")

        self.declare_parameter("topic", "/wifi_config")
        self.declare_parameter("nmcli_path", "nmcli")
        self.declare_parameter("interface", "wlan0")
        self.declare_parameter("host_info_topic", "/host_info")
        self.declare_parameter("host_info_repeats", 8)
        self.declare_parameter("host_info_interval_sec", 1.0)
        self.declare_parameter("host_info_route_target", "192.168.31.1")
        self.declare_parameter("publish_host_info_on_startup", True)

        self.topic = self.get_parameter("topic").value
        self.nmcli_path = self.get_parameter("nmcli_path").value
        self.interface = self.get_parameter("interface").value
        self.host_info_topic = self.get_parameter("host_info_topic").value
        self.host_info_repeats = max(1, int(self.get_parameter("host_info_repeats").value))
        self.host_info_interval_sec = max(0.1, float(self.get_parameter("host_info_interval_sec").value))
        self.host_info_route_target = self.get_parameter("host_info_route_target").value
        self.publish_host_info_on_startup = bool(
            self.get_parameter("publish_host_info_on_startup").value
        )

        self._lock = threading.Lock()
        self._last_applied = None
        wifi_config_qos = QoSProfile(depth=10)
        wifi_config_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(
            String, self.topic, self._on_wifi_config, wifi_config_qos
        )
        self.host_info_publisher = self.create_publisher(
            String, self.host_info_topic, 10
        )
        self.host_info_timer = self.create_timer(
            self.host_info_interval_sec, self._publish_host_info_once
        )
        self.get_logger().info(
            f"Listening on {self.topic}; applying Wi-Fi config with {self.nmcli_path} "
            f"for interface={self.interface}; publishing host info on {self.host_info_topic}"
        )
        if self.publish_host_info_on_startup:
            self._schedule_host_info_burst(initial_delay=2.0)

    def _extract_credentials(self, data: str):
        payload = json.loads(data)
        ssid = payload.get("ssid") or payload.get("wifi")
        password = payload.get("password") or payload.get("psk")
        if not ssid:
            raise ValueError("missing ssid/wifi field")
        if password is None:
            raise ValueError("missing password field")
        return str(ssid), str(password)

    def _run_nmcli(self, ssid: str, password: str):
        list_cmd = [self.nmcli_path, "-t", "-f", "NAME,TYPE", "connection", "show"]
        result = subprocess.run(list_cmd, capture_output=True, text=True, check=False)
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                name, _, kind = line.partition(":")
                if kind == "wifi" and name == ssid:
                    delete_cmd = [self.nmcli_path, "connection", "delete", "id", ssid]
                    delete_result = subprocess.run(
                        delete_cmd, capture_output=True, text=True, check=False
                    )
                    if delete_result.returncode != 0:
                        raise RuntimeError(
                            "failed to delete old wifi profile: "
                            f"{delete_result.stderr.strip() or delete_result.stdout.strip()}"
                        )
                    break

        cmd = [
            self.nmcli_path,
            "device",
            "wifi",
            "connect",
            ssid,
            "password",
            password,
        ]
        if self.interface:
            cmd.extend(["ifname", self.interface])

        redacted = cmd.copy()
        if "password" in redacted:
            idx = redacted.index("password")
            if idx + 1 < len(redacted):
                redacted[idx + 1] = "****"
        self.get_logger().info(f"Running: {' '.join(redacted)}")
        result = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if result.returncode != 0:
            raise RuntimeError(
                f"nmcli failed ({result.returncode}): {result.stderr.strip() or result.stdout.strip()}"
            )
        return result.stdout.strip()

    def _get_interface_ip(self):
        if not self.interface:
            return ""
        result = subprocess.run(
            ["ip", "-4", "addr", "show", "dev", self.interface],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            return ""
        for line in result.stdout.splitlines():
            line = line.strip()
            if line.startswith("inet "):
                return line.split()[1].split("/", 1)[0]
        return ""

    def _get_route_ip(self):
        if not self.host_info_route_target:
            return ""
        result = subprocess.run(
            ["ip", "route", "get", str(self.host_info_route_target)],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            return ""
        parts = result.stdout.split()
        if "src" in parts:
            index = parts.index("src") + 1
            if index < len(parts):
                return parts[index]
        return ""

    def _get_socket_ip(self):
        target = str(self.host_info_route_target or "8.8.8.8")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect((target, 80))
            return sock.getsockname()[0]
        except OSError:
            return ""
        finally:
            sock.close()

    def _get_host_info_text(self):
        hostname = socket.gethostname().strip() or "unknown"
        ip_addr = self._get_interface_ip() or self._get_route_ip() or self._get_socket_ip()
        if not ip_addr or ip_addr == "0.0.0.0":
            return ""
        return f"hostname={hostname} ip={ip_addr}"

    def _publish_host_info_once(self):
        text = self._get_host_info_text()
        if not text:
            self.get_logger().debug("Skipping host_info publish until a valid IP is available")
            return
        msg = String()
        msg.data = text
        self.host_info_publisher.publish(msg)
        self.get_logger().info(f"Published host_info: {msg.data}")

    def _publish_host_info_burst(self, initial_delay: float = 0.0):
        if initial_delay > 0.0:
            time.sleep(initial_delay)
        for attempt in range(self.host_info_repeats):
            if not rclpy.ok():
                return
            try:
                self._publish_host_info_once()
            except Exception as exc:
                self.get_logger().warning(f"Failed to publish host_info: {exc}")
            if attempt + 1 < self.host_info_repeats:
                time.sleep(self.host_info_interval_sec)

    def _schedule_host_info_burst(self, initial_delay: float = 0.0):
        thread = threading.Thread(
            target=self._publish_host_info_burst,
            kwargs={"initial_delay": initial_delay},
            daemon=True,
        )
        thread.start()

    def _on_wifi_config(self, msg: String):
        try:
            payload = msg.data.strip()
            try:
                ssid, password = self._extract_credentials(payload)
            except json.JSONDecodeError:
                parts = payload.split(",", 1)
                if len(parts) != 2:
                    parts = payload.split(";", 1)
                if len(parts) != 2:
                    raise
                ssid, password = parts[0].strip(), parts[1].strip()
        except Exception as exc:
            self.get_logger().error(f"Invalid wifi_config payload: {exc}")
            return

        key = (ssid, password)
        with self._lock:
            if key == self._last_applied:
                self.get_logger().debug(f"Ignoring duplicate wifi_config for ssid={ssid}")
                return

            try:
                output = self._run_nmcli(ssid, password)
            except Exception as exc:
                self.get_logger().error(f"Failed to apply wifi_config for ssid={ssid}: {exc}")
                return

            self._last_applied = key
            if output:
                self.get_logger().info(output)
            self.get_logger().info(f"Applied wifi_config for ssid={ssid}")
            self._schedule_host_info_burst()


def main(args=None):
    rclpy.init(args=args)
    node = WifiConfigApplier()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
