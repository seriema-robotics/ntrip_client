#!/usr/bin/env python3

import os
import sys
import json
import base64
import socket

import rclpy
from std_srvs.srv import Trigger

from ntrip_ros2_base import NTRIPRosBase
from ntrip_client.ntrip_client import NTRIPClient

class NTRIPRos(NTRIPRosBase):
  def __init__(self):
    # Init the node and declare params
    super().__init__('ntrip_client')
    self.declare_parameters(
      namespace='',
      parameters=[
        ('host', '127.0.0.1'),
        ('port', 2101),
        ('mountpoint', 'mount'),
        ('ntrip_version', 'None'),
        ('authenticate', False),
        ('username', ''),
        ('password', ''),
        ('ssl', False),
        ('cert', 'None'),
        ('key', 'None'),
        ('ca_cert', 'None'),
        ('rtcm_timeout_seconds', NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS),
        ('enabled', True),
      ]
    )

    # Read some mandatory config
    host = self.get_parameter('host').value
    port_param = self.get_parameter('port').value
    try:
      port = int(port_param)
    except ValueError:
      port = port_param
    mountpoint = self.get_parameter('mountpoint').value

    # Optionally get the ntrip version from the launch file
    ntrip_version = self.get_parameter('ntrip_version').value
    if ntrip_version == 'None':
      ntrip_version = None

    # If we were asked to authenticate, read the username and password
    username = None
    password = None
    if self.get_parameter('authenticate').value:
      username = self.get_parameter('username').value
      password = self.get_parameter('password').value
      if not username:
        self.get_logger().error('Requested to authenticate, but param "username" was not set')
        sys.exit(1)
      if not password:
        self.get_logger().error('Requested to authenticate, but param "password" was not set')
        sys.exit(1)

    # Initialize the client
    self._client = NTRIPClient(
      host=host,
      port=port,
      mountpoint=mountpoint,
      ntrip_version=ntrip_version,
      username=username,
      password=password,
      logerr=self.get_logger().error,
      logwarn=self.get_logger().warning,
      loginfo=self.get_logger().info,
      logdebug=self.get_logger().debug
    )

    # Get some SSL parameters for the NTRIP client
    self._client.ssl = self.get_parameter('ssl').value
    self._client.cert = self.get_parameter('cert').value
    self._client.key = self.get_parameter('key').value
    self._client.ca_cert = self.get_parameter('ca_cert').value
    if self._client.cert == 'None':
      self._client.cert = None
    if self._client.key == 'None':
      self._client.key = None
    if self._client.ca_cert == 'None':
      self._client.ca_cert = None

    # Get some timeout parameters for the NTRIP client
    self._client.nmea_parser.nmea_max_length = self._nmea_max_length
    self._client.nmea_parser.nmea_min_length = self._nmea_min_length
    self._client.reconnect_attempt_max = self._reconnect_attempt_max
    self._client.reconnect_attempt_wait_seconds = self._reconnect_attempt_wait_seconds
    self._client.rtcm_timeout_seconds = self.get_parameter('rtcm_timeout_seconds').value

    # ROS 2 Services callback takes (request, response) and returns response
    self._restart_service = self.create_service(Trigger, '~/restart', self.handle_restart)
    self._get_status_service = self.create_service(Trigger, '~/get_status', self.handle_get_status)
    self._get_mountpoints_service = self.create_service(Trigger, '~/get_mountpoints', self.handle_get_mountpoints)

  def run(self):
    from sensor_msgs.msg import NavSatFix
    from nmea_msgs.msg import Sentence

    # Setup our subscribers
    self._nmea_sub = self.create_subscription(Sentence, 'nmea', self.subscribe_nmea, 10)
    self._fix_sub = self.create_subscription(NavSatFix, 'fix', self.subscribe_fix, 10)

    # Check if we should connect on start
    enabled = self.get_parameter('enabled').value
    if enabled:
      # Connect the client
      if self._client.connect():
        # Start the timer that will check for RTCM data
        self._rtcm_timer = self.create_timer(0.1, self.publish_rtcm)
        self.get_logger().info('Successfully connected to NTRIP server on startup.')
      else:
        self.get_logger().warning('Unable to connect to NTRIP server on startup, but keeping node alive.')
    else:
      self.get_logger().info('NTRIP client initialized in disabled state.')
    return True

  def handle_restart(self, request, response):
    self.get_logger().info("NTRIP Restart request received. Reloading parameters...")
    
    # 1. Stop the RTCM timer to prevent concurrent socket access
    if self._rtcm_timer:
      self._rtcm_timer.cancel()
      self.destroy_timer(self._rtcm_timer)
      self._rtcm_timer = None

    # 2. Disconnect current connection
    self._client.disconnect()

    # Check if enabled
    enabled = self.get_parameter('enabled').value
    if not enabled:
      self.get_logger().info("NTRIP client is disabled. Not reconnecting.")
      response.success = True
      response.message = "NTRIP client disconnected and disabled."
      return response

    # 3. Reload params
    host = self.get_parameter('host').value
    port_param = self.get_parameter('port').value
    try:
      port = int(port_param)
    except ValueError:
      port = port_param
    mountpoint = self.get_parameter('mountpoint').value
    
    # Update client parameters
    self._client._host = host
    self._client._port = port
    self._client._mountpoint = mountpoint

    if self.get_parameter('authenticate').value:
      username = self.get_parameter('username').value
      password = self.get_parameter('password').value
      if username and password:
        self._client._basic_credentials = base64.b64encode('{}:{}'.format(
          username, password).encode('utf-8')).decode('utf-8')
    else:
      self._client._basic_credentials = None

    # Get optional SSL and cert parameters
    self._client.ssl = self.get_parameter('ssl').value
    self._client.cert = self.get_parameter('cert').value
    self._client.key = self.get_parameter('key').value
    self._client.ca_cert = self.get_parameter('ca_cert').value
    if self._client.cert == 'None':
      self._client.cert = None
    if self._client.key == 'None':
      self._client.key = None
    if self._client.ca_cert == 'None':
      self._client.ca_cert = None
    self._client.rtcm_timeout_seconds = self.get_parameter('rtcm_timeout_seconds').value

    # 4. Attempt to reconnect
    self.get_logger().info("Reconnecting to NTRIP server at {}:{}/{}".format(host, port, mountpoint))
    success = self._client.connect()

    # 5. Restart the RTCM timer if connection was successful
    if success:
      self._rtcm_timer = self.create_timer(0.1, self.publish_rtcm)
      msg = "Successfully reconnected to NTRIP server at {}:{}/{}".format(host, port, mountpoint)
      self.get_logger().info(msg)
      response.success = True
      response.message = msg
    else:
      msg = "Failed to reconnect to NTRIP server at {}:{}/{}".format(host, port, mountpoint)
      self.get_logger().error(msg)
      response.success = False
      response.message = msg
    return response

  def handle_get_status(self, request, response):
    status_data = {
      "enabled": self.get_parameter('enabled').value,
      "connected": getattr(self._client, "_connected", False),
      "host": getattr(self._client, "_host", ""),
      "port": getattr(self._client, "_port", ""),
      "mountpoint": getattr(self._client, "_mountpoint", ""),
      "rtcm_timeout_seconds": getattr(self._client, "rtcm_timeout_seconds", 4),
      "last_rtcm_received": getattr(self._client, "_recv_rtcm_last_packet_timestamp", 0)
    }
    response.success = True
    response.message = json.dumps(status_data)
    return response

  def handle_get_mountpoints(self, request, response, default_host='127.0.0.1', default_port=2101, socket_timeout=5.0, buffer_size=4096, filter_keyword="RTCM"):
    host = self.get_parameter('host').value
    try:
      port = int(self.get_parameter('port').value)
    except ValueError:
      port = self.get_parameter('port').value

    self.get_logger().info("Fetching NTRIP sourcetable from {}:{}".format(host, port))
    try:
      sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      sock.settimeout(socket_timeout)
      sock.connect((host, port))
      
      request_str = "GET / HTTP/1.0\r\nUser-Agent: NTRIP Client/1.0\r\n\r\n"
      sock.sendall(request_str.encode('utf-8'))
      
      response_data = b""
      while True:
        chunk = sock.recv(buffer_size)
        if not chunk:
          break
        response_data += chunk
      sock.close()
      
      content = response_data.decode('utf-8', errors='ignore')
      
      mountpoints = []
      for line in content.splitlines():
        if line.startswith('STR;'):
          parts = line.split(';')
          if len(parts) > 3:
            if filter_keyword in parts[3].upper():
              mountpoints.append(parts[1])
          elif len(parts) > 1:
            mountpoints.append(parts[1])
            
      response.success = True
      response.message = json.dumps(mountpoints)
    except Exception as e:
      err_msg = "Failed to fetch sourcetable from {}:{}: {}".format(host, port, str(e))
      self.get_logger().error(err_msg)
      response.success = False
      response.message = err_msg
    return response

if __name__ == '__main__':
  # Start the node
  rclpy.init()
  node = NTRIPRos()
  if not node.run():
    sys.exit(1)
  try:
    # Spin until we are shut down
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  except BaseException as e:
    raise e
  finally:
    node.stop()
    # Shutdown the node and stop rclpy
    rclpy.shutdown()
