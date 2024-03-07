import subprocess
import sys
import os

# Run example: 
# python3 <path_to_this_file> <path_to_catkin_ws> <path_to_stereo_config_file>
# python3 '/home/pincho/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes/detectors_and_trackers/controller.py' /home/pincho/catkin_ws /home/pincho/catkin_ws/src/apple_fields_ignition/fields_ignition/config/stereoParamsSim.yaml

def launch_ros_process(folder_path, stereo_config_path):
    #folder_path = '/home/pincho/catkin_ws'
    #stereo_config_path = '/home/pincho/catkin_ws/src/apple_fields_ignition/fields_ignition/config/stereoParamsSim.yaml'
    command = ['roslaunch', 'fields_ignition', 'stereo_simulation.launch', f'folder_path:={folder_path}', f'stereo_config_path:={stereo_config_path}']
    process = subprocess.call(command) 

if __name__ == "__main__":
    folder_path = sys.argv[1]
    stereo_config_path = sys.argv[2]

    print('initiating process, press ctrl+c to stop')
    process = launch_ros_process(folder_path, stereo_config_path)

    print("ROS process stopped. Starting save_disparity_and_rgb.py")
    python_script_path = folder_path + '/src/apple_fields_ignition/fields_ignition/nodes/detectors_and_trackers/rgb_and_depth_processing.py'
    subprocess.run(['python3', python_script_path])
