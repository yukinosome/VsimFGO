import os
import launch.launch_description_sources
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    config_common_path = LaunchConfiguration('config_common_path')
    use_vsim = LaunchConfiguration('use_vsim')

    ### GNSS PREPROCESSING
    default_config_gnss_preprocessing = os.path.join(
        get_package_share_directory('irt_gnss_preprocessing'),
        'config',
        'gnss_preprocessing.yaml'
    )

    declare_config_gnss_preprocessing_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_gnss_preprocessing,
        description='GNSS_Parameters')

    node_gnss_preprocessing = Node(
        package='irt_gnss_preprocessing',
        executable='node_gnss_preprocessing',
        namespace="irt_gnss_preprocessing",
        name="irt_gnss_preprocessing",
        output='screen',
        emulate_tty=True,
        prefix=['xterm -sl 10000 -hold -e '],
        #arguments=['--ros-args', '--log-level', logger],
        parameters=[
            config_common_path,  # Common parameter
            default_config_gnss_preprocessing
        ],
        remappings=[
            ('/user_estimation', '/deutschland/user_estimation'),
            #('/rtcmL1E1', '/sapos/rtcmL1E1')
        ]
    )

    ### VIRTUAL SIMULATION (VSim) NODE
    default_config_vsim = os.path.join(
        get_package_share_directory('online_fgo'),
        'config',
        'vsim.yaml'
    )

    declare_config_vsim_path_cmd = DeclareLaunchArgument(
        'config_vsim_path',
        default_value=default_config_vsim,
        description='VirtualSimulationParameters')

    declare_use_vsim_cmd = DeclareLaunchArgument(
        'use_vsim',
        default_value='false',
        description='Enable VSim Lat/Lon factor in GNSSLCIntegrator')

    node_vsim = Node(
        package='VisualSimulation',
        executable='test_node',
        name="vsim_node",
        namespace="vsim",
        output='screen',
        parameters=[
            config_common_path,
            default_config_vsim
        ],
        remappings=[
            ('/novatel/oem7/vsim', '/novatel/oem7/vsim'),  # Virtual simulation input
            ('/imu/data', '/imu/sensor_data'),            # IMU data input
            ('/gnss/fix', '/novatel/oem7/bestpos'),       # GNSS data input
        ]
    )

    ### FGO NODE for VSim-GNSS-IMU Fusion
    default_config_common = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/vsim_gnss_imu',
        'common.yaml'
    )
    default_config_integrator = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/vsim_gnss_imu',
        'integrator.yaml'
    )
    default_config_optimizer = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/vsim_gnss_imu',
        'optimizer.yaml'
    )
    
    declare_config_common_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_common,
        description='CommonParameters')

    declare_config_integrator_path_cmd = DeclareLaunchArgument(
        'config_integrator_path',
        default_value=default_config_integrator,
        description='IntegratorParameters')

    declare_config_optimizer_path_cmd = DeclareLaunchArgument(
        'config_optimizer_path',
        default_value=default_config_optimizer,
        description='OptimizerParameters')

    default_config_sensor_parameters = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/vsim_gnss_imu',
        'sensor_parameters.yaml'
    )
    declare_config_sensor_parameters_path_cmd = DeclareLaunchArgument(
        'config_sensor_parameters_path',
        default_value=default_config_sensor_parameters,
        description='SensorParameters')

    node_online_fgo = Node(
        package='online_fgo',
        executable='online_fgo_node',
        name="online_fgo",
        namespace="vsim_gnss_imu_fusion",
        output='screen',
        emulate_tty=True,
        #prefix=['xterm -sl 10000 -hold -e '],
        # arguments=['--ros-args', '--log-level', logger],
        parameters=[
            config_common_path,
            default_config_common,
            default_config_integrator,
            default_config_optimizer,
            default_config_sensor_parameters,
            {
                'GNSSFGO': {
                    'IRTPVALCIntegrator': {
                        'useVSimLatLonFactor': use_vsim,
                    }
                }
            }
        ],
        remappings=[
            ('/vsim/fix', '/novatel/oem7/vsim'),         # VSim input
            ('/imu/data', '/imu/sensor_data'),            # IMU data input
            ('/gnss/fix', '/novatel/oem7/bestpos'),       # GNSS data input
            ('/fgo/odom', '/vsim_gnss_imu/odometry')      # FGO output
        ]
    )

    ### ROSBAG for playing test data
    BAG_PATH = "/Data/ros2_db3/vsim_gnss_imu_test"  # Path to vsim-gnss-imu test dataset
    START_OFFSET = "0"  # Start immediately
    try:
        if(EnvironmentVariable('BAG_PATH')):
            BAG_PATH = EnvironmentVariable('BAG_PATH')
        if(EnvironmentVariable('START_OFFSET')):
            START_OFFSET = EnvironmentVariable('START_OFFSET')
    except:
        pass
    print("Current bag path: ", BAG_PATH)
    print("Current start offset: ", START_OFFSET)

    exec_play_rosbag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            BAG_PATH,
            "--clock",
            "--start-offset",
            START_OFFSET,
            ],
        output="screen",
        prefix=['xterm -sl 10000 -hold -e '],
    )

    ### RVIZ2 for visualization
    rviz_config_file = os.path.join(
        get_package_share_directory('online_fgo'),
        'rviz',
        'vsim_gnss_imu_fusion.rviz'
    )

    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ### MAPVIZ for 2D visualization
    mapviz_dir = get_package_share_directory("mapviz")
    mapviz_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(mapviz_dir + '/launch/mapviz.launch.py'))

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "log_level",
        default_value=["debug"],
        description="Logging level"))
    ld.add_action(declare_config_gnss_preprocessing_path_cmd)
    ld.add_action(declare_config_vsim_path_cmd)
    ld.add_action(declare_config_common_path_cmd)
    ld.add_action(declare_config_integrator_path_cmd)
    ld.add_action(declare_config_optimizer_path_cmd)
    ld.add_action(declare_config_sensor_parameters_path_cmd)
    ld.add_action(declare_use_vsim_cmd)
    
    ld.add_action(node_gnss_preprocessing)
    ld.add_action(node_vsim)
    ld.add_action(node_online_fgo)
    
    ld.add_action(mapviz_launch)
    ld.add_action(exec_play_rosbag)
    ld.add_action(node_rviz2)

    return ld