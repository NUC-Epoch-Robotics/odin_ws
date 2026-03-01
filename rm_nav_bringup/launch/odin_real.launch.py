import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals

def generate_launch_description():
    # ===================== 1. 路径与环境变量获取 =====================
    rm_nav_bringup_dir = get_package_share_directory('rm_nav_bringup')
    navigation2_launch_dir = os.path.join(get_package_share_directory('rm_navigation'), 'launch')
    odin_driver_dir = get_package_share_directory('odin_ros_driver') 

    # Launch 配置变量
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_nav_rviz = LaunchConfiguration('nav_rviz')

    # 解析机器人 URDF 模型及雷达外参
    launch_params = yaml.safe_load(open(os.path.join(
        rm_nav_bringup_dir, 'config', 'reality', 'measurement_params_real.yaml')))
    robot_description = Command(['xacro ', os.path.join(
        rm_nav_bringup_dir, 'urdf', 'sentry_robot_real.xacro'),
        ' xyz:=', launch_params['base_link2odin_frame']['xyz'], 
        ' rpy:=', launch_params['base_link2odin_frame']['rpy']])

    # 定义各类参数文件路径
    slam_toolbox_map_dir = PathJoinSubstitution([rm_nav_bringup_dir, 'map', world])
    slam_toolbox_localization_file_dir = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'mapper_params_localization_real.yaml')
    slam_toolbox_mapping_file_dir = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'mapper_params_online_async_real.yaml')
    segmentation_params = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'segmentation_real.yaml')
    nav2_map_dir = [PathJoinSubstitution([rm_nav_bringup_dir, 'map', world]), ".yaml"]
    nav2_params_file_dir = os.path.join(rm_nav_bringup_dir, 'config', 'reality', 'nav2_params_real.yaml')

    # ===================== 2. 声明命令行参数 =====================
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='False')
    declare_nav_rviz_cmd = DeclareLaunchArgument('nav_rviz', default_value='True')
    declare_world_cmd = DeclareLaunchArgument('world', default_value='328', description='Map file name')
    declare_mode_cmd = DeclareLaunchArgument('mode', default_value='nav', description='nav or mapping')
    declare_localization_cmd = DeclareLaunchArgument('localization', default_value='odin', description='odin, slam_toolbox, amcl, icp')

    # ===================== 3. 核心节点定义 =====================

    # [基础] 机器人状态与 TF 树根节点发布
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        output='screen'
    )

    # [感知与定位] 启动 Odin SDK (输出 /odin1/cloud_slam 并发布 map->odom)
    start_odin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(odin_driver_dir, 'launch', 'odin1_ros2.launch.py'))
    )

    # [坐标对齐] 补全 base_link 到 odin_link 的静态转换
    odin_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'odin1_base_link', 'base_link']
    )
    
    # [环境优化 1] 线拟合地面分割 (多层场地避障核心)
    bringup_linefit_ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        name='ground_segmentation',
        parameters=[segmentation_params],
        output='screen'
    )

    # [环境优化 2] 3D 降维到 2D 投影 (规避桥梁/限高干扰)
    bringup_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in',  ['/segmentation/obstacle']), # 仅接收非地面(纯障碍物)点云
            ('scan',  ['/scan'])
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 0.1,
            'angle_min': -3.14159,  # -M_PI/2
            'angle_max': 3.14159,   # M_PI/2
            'angle_increment': 0.0043,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    # [系统支撑] 地图加载与兼容性定位组
    start_localization_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'nav'),
        actions=[
            Node(
                condition=LaunchConfigurationEquals('localization', 'slam_toolbox'),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_toolbox_localization_file_dir,
                    {'use_sim_time': use_sim_time,
                     'map_file_name': slam_toolbox_map_dir,
                     'map_start_pose': [0.0, 0.0, 0.0]}
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir,'localization_amcl_launch.py')),
                condition=LaunchConfigurationEquals('localization', 'amcl'),
                launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_params_file_dir, 'map': nav2_map_dir}.items()
            ),
            # 当使用 Odin 定位时，仅启动 map_server 提供全局栅格
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'map_server_launch.py')),
                condition=LaunchConfigurationNotEquals('localization', 'slam_toolbox'), 
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': nav2_map_dir,
                    'params_file': nav2_params_file_dir,
                    'container_name': 'nav2_container'}.items())
        ]
    )

    # [底盘控制] RM 底盘小陀螺速度解算
    #bringup_fake_vel_transform_node = Node(
    #    package='fake_vel_transform',
    #    executable='fake_vel_transform_node',
    #    output='screen',
    #    parameters=[{'use_sim_time': use_sim_time, 'spin_speed': 0.0}]
    #)

    # [建图] 动态建图模式
    start_mapping = Node(
        condition=LaunchConfigurationEquals('mode', 'mapping'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': use_sim_time,}
        ],
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(rm_nav_bringup_dir, 'rviz', 'nav2.rviz'),
        description='Path to RViz2 config file'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )
    # [导航] Nav2 导航堆栈
    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_rm_navigation.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav2_map_dir,
            'params_file': nav2_params_file_dir,
            'nav_rviz': use_nav_rviz}.items()
    )
    # ===================== 4. 组装并返回 =====================
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(rviz_config_arg)
    ld.add_action(rviz_node)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_odin_launch)
    ld.add_action(odin_static_tf)
    #ld.add_action(base_to_odin_link_tf)
    ld.add_action(bringup_linefit_ground_segmentation_node)  # 新增地面分割
    ld.add_action(bringup_pointcloud_to_laserscan_node)      # 修正后的 2D 投影
    
    ld.add_action(start_localization_group)
    #ld.add_action(bringup_fake_vel_transform_node)
    ld.add_action(start_mapping)
    ld.add_action(start_navigation2)
    return ld