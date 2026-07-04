#!/usr/bin/env python3

import sys
import importlib

import rospy

from ntrip_client.ntrip_ros_base import NTRIPRosBase
from ntrip_client.ntrip_client import NTRIPClient

# Try to import a couple different types of RTCM messages
_MAVROS_MSGS_NAME = "mavros_msgs"
_RTCM_MSGS_NAME = "rtcm_msgs"
have_mavros_msgs = False
have_rtcm_msgs = False
if importlib.util.find_spec(_MAVROS_MSGS_NAME) is not None:
  have_mavros_msgs = True
  from mavros_msgs.msg import RTCM as mavros_msgs_RTCM
if importlib.util.find_spec(_RTCM_MSGS_NAME) is not None:
  have_rtcm_msgs = True
  from rtcm_msgs.msg import Message as rtcm_msgs_RTCM

class NTRIPRos(NTRIPRosBase):
  def __init__(self):
    # Init the node
    super().__init__('ntrip_client')

    # Read some mandatory config
    host = rospy.get_param('~host', '127.0.0.1')
    port = rospy.get_param('~port', '2101')
    mountpoint = rospy.get_param('~mountpoint', 'mount')

    # Optionally get the ntrip version from the launch file
    ntrip_version = rospy.get_param('~ntrip_version', None)
    if ntrip_version == '':
      ntrip_version = None

    # If we were asked to authenticate, read the username and password
    username = None
    password = None
    if rospy.get_param('~authenticate', False):
      username = rospy.get_param('~username', None)
      password = rospy.get_param('~password', None)
      if username is None:
        rospy.logerr(
          'Requested to authenticate, but param "username" was not set')
        sys.exit(1)
      if password is None:
        rospy.logerr(
          'Requested to authenticate, but param "password" was not set')
        sys.exit(1)

    # Initialize the client
    self._client = NTRIPClient(
      host=host,
      port=port,
      mountpoint=mountpoint,
      ntrip_version=ntrip_version,
      username=username,
      password=password,
      logerr=rospy.logerr,
      logwarn=rospy.logwarn,
      loginfo=rospy.loginfo,
      logdebug=rospy.logdebug
    )

    # Get some SSL parameters for the NTRIP client
    self._client.ssl = rospy.get_param('~ssl', False)
    self._client.cert = rospy.get_param('~cert', None)
    self._client.key = rospy.get_param('~key', None)
    self._client.ca_cert = rospy.get_param('~ca_cert', None)

    # Set parameters on the client
    self._client.nmea_parser.nmea_max_length = self._nmea_max_length
    self._client.nmea_parser.nmea_min_length = self._nmea_min_length
    self._client.reconnect_attempt_max = self._reconnect_attempt_max
    self._client.reconnect_attempt_wait_seconds = self._reconnect_attempt_wait_seconds
    self._client.rtcm_timeout_seconds = rospy.get_param('~rtcm_timeout_seconds', NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS)

    from std_srvs.srv import Trigger
    self._restart_service = rospy.Service('~restart', Trigger, self.handle_restart)
    self._get_status_service = rospy.Service('~get_status', Trigger, self.handle_get_status)
    self._get_mountpoints_service = rospy.Service('~get_mountpoints', Trigger, self.handle_get_mountpoints)

  def handle_restart(self, req, default_host='127.0.0.1', default_port=2101, default_mountpoint='mount', default_rtcm_timeout=NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS):
    from std_srvs.srv import TriggerResponse
    import base64

    rospy.loginfo("NTRIP Restart request received. Reloading parameters...")
    
    # 1. Stop the RTCM timer to prevent concurrent socket access
    if self._rtcm_timer:
      self._rtcm_timer.shutdown()
      self._rtcm_timer = None

    # 2. Disconnect current connection
    self._client.disconnect()

    # 3. Reload params from parameter server
    host = rospy.get_param('~host', default_host)
    try:
      port = int(rospy.get_param('~port', default_port))
    except ValueError:
      port = rospy.get_param('~port', default_port)
    mountpoint = rospy.get_param('~mountpoint', default_mountpoint)
    
    # Update client parameters
    self._client._host = host
    self._client._port = port
    self._client._mountpoint = mountpoint

    if rospy.get_param('~authenticate', False):
      username = rospy.get_param('~username', None)
      password = rospy.get_param('~password', None)
      if username is not None and password is not None:
        self._client._basic_credentials = base64.b64encode('{}:{}'.format(
          username, password).encode('utf-8')).decode('utf-8')
    else:
      self._client._basic_credentials = None

    # Get optional SSL and cert parameters
    self._client.ssl = rospy.get_param('~ssl', False)
    self._client.cert = rospy.get_param('~cert', None)
    self._client.key = rospy.get_param('~key', None)
    self._client.ca_cert = rospy.get_param('~ca_cert', None)
    self._client.rtcm_timeout_seconds = rospy.get_param('~rtcm_timeout_seconds', default_rtcm_timeout)

    # 4. Attempt to reconnect
    rospy.loginfo("Reconnecting to NTRIP server at %s:%s/%s", host, port, mountpoint)
    success = self._client.connect()

    # 5. Restart the RTCM timer if connection was successful
    if success:
      self._rtcm_timer = rospy.Timer(rospy.Duration(0.1), self.publish_rtcm)
      msg = "Successfully reconnected to NTRIP server at {}:{}/{}".format(host, port, mountpoint)
      rospy.loginfo(msg)
      return TriggerResponse(success=True, message=msg)
    else:
      msg = "Failed to reconnect to NTRIP server at {}:{}/{}".format(host, port, mountpoint)
      rospy.logerr(msg)
      return TriggerResponse(success=False, message=msg)

  def handle_get_status(self, req):
    from std_srvs.srv import TriggerResponse
    import json
    
    status_data = {
      "connected": getattr(self._client, "_connected", False),
      "host": getattr(self._client, "_host", ""),
      "port": getattr(self._client, "_port", ""),
      "mountpoint": getattr(self._client, "_mountpoint", ""),
      "rtcm_timeout_seconds": getattr(self._client, "rtcm_timeout_seconds", 4),
      "last_rtcm_received": getattr(self._client, "_recv_rtcm_last_packet_timestamp", 0)
    }
    return TriggerResponse(success=True, message=json.dumps(status_data))

  def handle_get_mountpoints(self, req, default_host='127.0.0.1', default_port=2101):
    from std_srvs.srv import TriggerResponse
    import urllib.request
    import json
    
    host = rospy.get_param('~host', default_host)
    try:
      port = int(rospy.get_param('~port', default_port))
    except ValueError:
      port = rospy.get_param('~port', default_port)

    url = "http://{}:{}/".format(host, port)
    rospy.loginfo("Fetching NTRIP sourcetable from %s", url)
    try:
      req_http = urllib.request.Request(url, headers={'User-Agent': 'NTRIP Client/1.0'})
      with urllib.request.urlopen(req_http, timeout=5.0) as response:
        content = response.read().decode('utf-8', errors='ignore')
      
      mountpoints = []
      for line in content.splitlines():
        if line.startswith('STR;'):
          parts = line.split(';')
          if len(parts) > 1:
            mountpoints.append(parts[1])
            
      return TriggerResponse(success=True, message=json.dumps(mountpoints))
    except Exception as e:
      err_msg = "Failed to fetch sourcetable from {}: {}".format(url, str(e))
      rospy.logerr(err_msg)
      return TriggerResponse(success=False, message=err_msg)



if __name__ == '__main__':
  ntrip_ros = NTRIPRos()
  sys.exit(ntrip_ros.run())
