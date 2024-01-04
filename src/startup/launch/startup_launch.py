from os.path import join as Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Similasyon kapatılınca programda sonlanması sağlanır.
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

startup_path = get_package_share_directory("startup")
calibrate_world = Path(startup_path, "worlds", "calibration.world")
config = Path(startup_path, 'config', 'params.yaml')

calibrate_sim = ExecuteProcess(cmd = ["gz", "sim", "-r", "4", calibrate_world])
rviz2 = ExecuteProcess(cmd = ["rviz2"])

def generate_launch_description():
    return LaunchDescription([
        calibrate_sim   
    ])