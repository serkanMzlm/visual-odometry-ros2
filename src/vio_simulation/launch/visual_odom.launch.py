from os.path import join as Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

vio_simulation = get_package_share_directory("vio_simulation")
sim_world = Path(vio_simulation, "worlds", "visual_odamtry.world")
config = Path(vio_simulation, 'config', 'params.yml')

sim = ExecuteProcess(cmd = ["gz", "sim", "-r", sim_world])
sim_server = ExecuteProcess(cmd = ["gz", "sim", "-r", "-s", sim_world])

rviz2 = ExecuteProcess(cmd = ["rviz2"])

joy_node = Node(
            package="joy",                                       
            executable="joy_node",
            output="screen"
)

bridge_keyboard = Node(
            package="ros_gz_bridge",                                               
            executable="parameter_bridge",
            arguments=[
                "/keyboard/keypress@std_msgs/msg/Int32[gz.msgs.Int32"
            ],
            remappings=[("/keyboard/keypress","/keypress")],
            output="screen"
          )

control_node = Node(
            package="vehicle_control",                                       
            executable="control_node",
            output="screen"
)

live_camera = Node(
    package="live_camera",     
    executable="live_camera_node",
    parameters=[config],
    output="screen"
)

camera_bridge = Node(
    package="ros_gz_bridge",     
    executable="parameter_bridge",
    arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image']
)

control_bridge = Node(
            package="ros_gz_bridge",                                               
            executable="parameter_bridge",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
            ],
            output="screen"
)

def generate_launch_description():
    print(config)
    return LaunchDescription([
        sim,
        # sim_server,
        # live_camera,
        # joy_node,
        bridge_keyboard,
        control_node,
        # rviz2,
        camera_bridge,
        control_bridge,
    ])