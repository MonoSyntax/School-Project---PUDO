
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command,LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
 
def generate_launch_description():

    pkg_share= get_package_share_directory("simulation_2025")

    gz_sim_resource_path = os.path.join(pkg_share, 'models')
    
    # Add the parent directory of the package to the resource path
    # This ensures that meshes referenced as model://simulation_2025/... can be found
    pkg_share_parent = os.path.dirname(pkg_share)
    os.environ["GZ_SIM_RESOURCE_PATH"] = f"{gz_sim_resource_path}:{pkg_share_parent}"

    world = LaunchConfiguration('world')

    declare_world_cmd=DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_share, "models", "TeknofestWORLDV2.sdf"),
        description="Robot controller to start.",
    )
    
    gazebo = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"),
        launch_arguments=[("gz_args", ["-r -v 4 ", LaunchConfiguration("world")])],
    )

    ros_gz_bridge_node= Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        ros_arguments=["-p", f"config_file:={os.path.join(pkg_share, 'config', 'ign_bridge.yaml')}"],
        remappings=[
            ('/lidar/points', '/lslidar_point_cloud'),
        ]
    )

    robot_spawner = TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                        os.path.join(pkg_share, 'launch', 'robot_spawner.launch.py')
                        )
                    )
                ]
    )

    traffic_light= Node(
        package='simulation_2025',
        executable='traffic_light.py',
        name='traffic_light'
    )

    # Create the launch description and populate
    ld = LaunchDescription()
   
    # Add any actions

    ld.add_action(declare_world_cmd)
    ld.add_action(gazebo)
    ld.add_action(ros_gz_bridge_node)
    ld.add_action(traffic_light)
    ld.add_action(robot_spawner)

    return ld
