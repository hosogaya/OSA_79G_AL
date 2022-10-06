import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    device_name = LaunchConfiguration("device_name", default='/dev/ttyUSB0')
    param_path = os.path.join(get_package_share_directory('osa_79g_al_connector'), "config", "osa_79g_al_connector.param.yaml")

    def load_node_param(path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)
            return data['/**']['ros__parameters']
        
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_name', default_value='/dev/ttyUSB0',
            description='a port which osa-79g-al is connected'
        ),
        Node(
            package= 'osa_79g_al_connector',
            executable='osa_79g_al_connector',
            name='osa_79g_al',
            output = 'screen',
            parameters = [load_node_param(param_path)],
            arguments=[device_name]
        ),
    ])