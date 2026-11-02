"""Isolated simulation only. Optional ROS outputs always use /zekeep_sim."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('python_executable', default_value='/usr/bin/python3'),
        DeclareLaunchArgument('publish_ros', default_value='false'),
        DeclareLaunchArgument('friction', default_value='1.2'),
        DeclareLaunchArgument('object_mass', default_value='0.04'),
        DeclareLaunchArgument('table_z', default_value='0.0'),
        ExecuteProcess(cmd=[LaunchConfiguration('python_executable'), '-m', 'zekeep_sim.server',
            '--bringup', FindPackageShare('zekeep_bringup'), '--friction', LaunchConfiguration('friction'),
            '--object-mass', LaunchConfiguration('object_mass'), '--table-z', LaunchConfiguration('table_z'),
            PythonExpression(["'--ros' if '", LaunchConfiguration('publish_ros'), "'.lower() == 'true' else '--port=8083'"])],
            additional_env={'MUJOCO_GL':'egl'}, output='screen'),
    ])
