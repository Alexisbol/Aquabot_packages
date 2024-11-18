from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    sl.node('camera_control')

    sl.node('camera_qrcode','opencv_decoder.py')

    sl.include('aquabot_ekf', 'ekf_launch.py')

    sl.node('py_pathfinding')
    
    return sl.launch_description()
