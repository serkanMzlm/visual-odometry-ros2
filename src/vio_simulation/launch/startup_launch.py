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

vio_simulation = get_package_share_directory("vio_simulation")
calibrate_world = Path(vio_simulation, "worlds", "calibration.world")
config = Path(vio_simulation, 'config', 'params.yml')

calibrate_sim = ExecuteProcess(cmd = ["gz", "sim", "-r", calibrate_world])
rviz2 = ExecuteProcess(cmd = ["rviz2"])

save_frame_node = Node(
    package="save_frame",     
    executable="save_frame_node",
    parameters=[config],
    # ros_arguments=[ "--log-level", "camera_node:=debug",
    #             "--remap", "camera_node:=my_command_node"],
    output="screen"
)

calibrate_node = Node(
    package="camera_calibration",     
    executable="camera_calibration_node",
    parameters=[config],
    # ros_arguments=[ "--log-level", "camera_node:=debug",
    #             "--remap", "camera_node:=my_command_node"],
    output="screen"
)

def generate_launch_description():
    print(config)
    return LaunchDescription([
        # calibrate_sim,
        # save_frame_node,
        calibrate_node
    ])
