from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)
    sl.declare_arg('rviz', True)
    sl.declare_arg('unify',False)

    with sl.group(if_arg = 'rviz'):
        sl.rviz(sl.find('aquabot_ekf', 'ekf.rviz'))

    for link in ('base_link', 'imu_wamv_link', 'gps_wamv_link', 'receiver', 'right_engine_link', 'left_engine_link', 'main_camera_post_link', 'right_propeller_link', 'left_propeller_link'):
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

    return sl.launch_description()
