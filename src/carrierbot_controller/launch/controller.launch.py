from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )
    
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="false",
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
    )

    wheel_seperation_arg = DeclareLaunchArgument(
        "wheel_seperation",
        default_value="0.17",
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="false",
    )
        
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_seperation = LaunchConfiguration("wheel_seperation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["carrierbot_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller),
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="carrierbot_controller",
                executable="simple_controller_py",
                parameters=[{"wheel_radius": wheel_radius,
                            "wheel_seperation": wheel_seperation}],
                condition=IfCondition(use_python)
            ),

            Node(
                package="carrierbot_controller",
                executable="simple_controller",
                parameters=[{"wheel_radius": wheel_radius,
                            "wheel_seperation": wheel_seperation}],
                output="screen",
                condition=IfCondition(use_simple_controller)
            ),

            Node(
                package="carrierbot_controller",
                executable="cmd_vel_relay",
                output="screen",
                condition=IfCondition(use_simple_controller)
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_python_arg,
        wheel_radius_arg,
        wheel_seperation_arg,
        use_simple_controller_arg,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller,
    ])


