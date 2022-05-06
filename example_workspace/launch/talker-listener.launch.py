# (c) 2022 Copyright, Real-Time Innovations, Inc. (RTI) All rights reserved.
# 
# RTI grants Licensee a license to use, modify, compile, and create derivative
# works of the Software solely in combination with RTI Connext DDS. Licensee
# may redistribute copies of the Software provided that all such copies are
# subject to this License. The Software is provided "as is", with no warranty
# of any type, including any warranty for fitness for any purpose. RTI is
# under no obligation to maintain or support the Software. RTI shall not be
# liable for any incidental or consequential damages arising out of the use or
# inability to use the Software. For purposes of clarity, nothing in this
# License prevents Licensee from using alternate versions of DDS, provided
# that Licensee may not combine or link such alternate versions of DDS with
# the Software.
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  cfg_dir = Path(get_package_share_directory("example_workspace")) / "connext"

  # Pass some custom environment variables to the launched processes to
  # configure Connext DDS:
  # - NDDS_QOS_PROFILES: load a custom QoS configuration file.
  # - RMW_IMPLEMENTATION: select Connext as the ROS 2 RMW. Not needed if
  #   running inside one of the developement containers, since they have
  #   this variable set to "rmw_connextdds".
  connext_env = {
    "NDDS_QOS_PROFILES": str(cfg_dir / "example_custom_qos.xml"),
    "RMW_IMPLEMENTATION": "rmw_connextdds",
  }

  return LaunchDescription([
    Node(
      package="demo_nodes_cpp",
      executable="talker",
      additional_env=connext_env,
    ),
    Node(
      package="demo_nodes_cpp",
      executable="listener",
      additional_env=connext_env,
    )
  ])
