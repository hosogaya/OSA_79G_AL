import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_path = os.path.join(get_package_share_directory('front_milli_wave_sensor_connector'), "config", "front_milli_wave_sensor_connector.param.yaml")

    def load_node_param(path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)
            return data['/**']['ros__parameters']
        
    return LaunchDescription([
        Node(
            package= 'front_milli_wave_sensor_connector',
            executable='front_milli_wave_sensor_connector',
            name='front_milli_wave_sensor',
            output = 'screen',
            parameters = [load_node_param(param_path)],
        ),
    ])