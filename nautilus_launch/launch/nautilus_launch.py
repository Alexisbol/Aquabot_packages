from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    sl.node('camera_control')

    sl.node('camera_qrcode','opencv_decoder.py',respawn=True,respawn_delay=0.2)
    
    sl.include('aquabot_ekf', 'ekf_launch.py')

    sl.node('py_pathfinding', 'pathfinding')

    sl.node('tracking')

    sl.node('mission', 'mission.py')
    
    return sl.launch_description()
