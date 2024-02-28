from os.path import join as Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

vio_simulation = get_package_share_directory("vio_simulation")
sim_world = Path(vio_simulation, "worlds", "visual_odamtry.world")
config = Path(vio_simulation, 'config', 'params.yml')

sim = ExecuteProcess(cmd = ["gz", "sim", "-r", sim_world])
rviz2 = ExecuteProcess(cmd = ["rviz2"])

camera_bridge = Node(
    package="ros_gz_bridge",     
    executable="parameter_bridge",
    arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image']
)

def generate_launch_description():
    print(config)
    return LaunchDescription([
        sim,
        camera_bridge,
    ])