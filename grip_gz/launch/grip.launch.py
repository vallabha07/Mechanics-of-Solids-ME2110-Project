from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _setup(context):
    return [Node(package='grip_gz', executable='grip_dashboard', output='screen', parameters=[{
        'object': LaunchConfiguration('object').perform(context),
        'pad': LaunchConfiguration('pad').perform(context),
        'seed': int(LaunchConfiguration('seed').perform(context)),
        'cad': LaunchConfiguration('cad').perform(context),
        'cad_material': LaunchConfiguration('cad_material').perform(context),
        'cad_unit': LaunchConfiguration('cad_unit').perform(context),
        'cad_args': LaunchConfiguration('cad_args').perform(context),
        'fingers': int(LaunchConfiguration('fingers').perform(context)),
    }])]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('object', default_value='apple'),
        DeclareLaunchArgument('pad', default_value='small'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('cad', default_value=''),
        DeclareLaunchArgument('cad_material', default_value='aluminium'),
        DeclareLaunchArgument('cad_unit', default_value='mm'),
        DeclareLaunchArgument('cad_args', default_value=''),
        DeclareLaunchArgument('fingers', default_value='5'),
        OpaqueFunction(function=_setup),
    ])
