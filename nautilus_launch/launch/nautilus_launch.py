from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    sl.set_parameters('/aquabot/bt_navigator', parameters = {'odom_topic':'/aquabot/odom'})

    #sl.node('camera_control')

    #sl.node('camera_qrcode','opencv_decoder.py')
    
    # launch the EKF for the aquabot 
    sl.declare_arg('rviz', True)
    sl.declare_arg('unify',True)

    with sl.group(if_arg = 'rviz'):
        sl.rviz(sl.find('aquabot_ekf', 'ekf.rviz'))

    for link in ( 'imu_wamv_link', 'gps_wamv_link', 'receiver', 'right_engine_link', 'left_engine_link', 'main_camera_post_link', 'right_propeller_link', 'left_propeller_link'):
        sl.node('tf2_ros', 'static_transform_publisher', name='static_'+link,
                arguments = ['--frame-id', 'wamv/'+link, '--child-frame-id', 'aquabot/wamv/'+link])

    sl.node('aquabot_ekf','gps2pose',
            parameters={'unify': sl.arg('unify')})

    # run an EKF for wamv
    sl.node('robot_localization', 'ekf_node', name = 'ekf',
            parameters = [sl.find('aquabot_ekf', 'ekf.yaml')],
            namespace = 'aquabot',
            remappings = {'odometry/filtered': 'odom'},
            output='screen')

    # ceux la c'est juste pour que rviz2 soit content
    for link in ('imu_wamv_link', 'gps_wamv_link', 'receiver', 'right_engine_link', 'left_engine_link', 'main_camera_post_link', 'right_propeller_link', 'left_propeller_link', 'cpu_cases_link', 'left_battery_link', 'main_camera_link', 'main_camera_link_optical', 'main_camera_post_arm_link', 'right_battery_link'):
        sl.node('tf2_ros', 'static_transform_publisher', name='static_forrviz2_'+link,
                arguments = ['--frame-id', 'wamv/wamv/'+link, '--child-frame-id', 'wamv/'+link])

    sl.node('tf2_ros', 'static_transform_publisher', name='static_map_2', arguments=['--child-frame-id', 'world', '--frame-id', 'map'])
    sl.node('tf2_ros', 'static_transform_publisher', name='static_map_3', arguments=['--child-frame-id', 'wamv/wamv/base_link', '--frame-id', 'wamv/base_link'])
    sl.node('tf2_ros', 'static_transform_publisher', name='static_map_4', arguments=['--child-frame-id', 'base_link', '--frame-id', 'wamv/base_link'])
    sl.node('tf2_ros', 'static_transform_publisher', name='static_map_5', arguments=['--child-frame-id', 'aquabot/wamv/base_link', '--frame-id', 'wamv/base_link'])

    #sl.node('update_map','add_turbines')


    #run nav2 stack
    sl.include('nav2_bringup', 'bringup_launch.py',
               launch_arguments={
                                 'map':[sl.find('nautilus_launch', 'map_400.yaml')],
                                 'params_file' : [sl.find('nautilus_launch','nav2_params.yaml')]})

    #sl.node('py_pathfinding', 'pathfinding')

    #sl.node('diff_control', 'control_node.py')

    #sl.node('mission', 'mission.py')
    
    return sl.launch_description()
