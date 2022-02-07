import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('crane_plus_description'),
        'urdf',
        'crane_plus.urdf.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    crane_plus_controllers = os.path.join(
        get_package_share_directory('crane_plus_control'),
        'config',
        'crane_plus_controllers.yaml'
        )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, crane_plus_controllers],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    spawn_joint_state_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner.py joint_state_controller'],
                shell=True,
                output='screen',
            )

    spawn_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner.py crane_plus_arm_controller'],
                shell=True,
                output='screen',
            )

    spawn_gripper_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner.py crane_plus_gripper_controller'],
                shell=True,
                output='screen',
            )

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])

    return LaunchDescription([
      controller_manager,
      spawn_joint_state_controller,
      spawn_arm_controller,
      spawn_gripper_controller,
      robot_state_publisher
    ])
