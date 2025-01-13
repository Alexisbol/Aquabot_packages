from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    sl.set_parameters('/aquabot/bt_navigator', parameters = {'odom_topic':'/aquabot/odom'})

    #sl.node('camera_control')

    #sl.node('camera_qrcode','opencv_decoder.py')
    
    sl.include('aquabot_ekf', 'ekf_launch.py')
    
    sl.include('nav2_bringup', 'bringup_launch.py',
               launch_arguments={'namespace': 'aquabot',
                                 'use_namespace': 'true',
                                 'map':[sl.find('nautilus_launch', 'testmap.yaml')],
                                 'params_file':[sl.find('nautilus_launch', 'nav2_params.yaml')]})

    #sl.node('py_pathfinding', 'pathfinding')

    #sl.node('tracking')

    #sl.node('mission', 'mission.py')
    
    return sl.launch_description()
