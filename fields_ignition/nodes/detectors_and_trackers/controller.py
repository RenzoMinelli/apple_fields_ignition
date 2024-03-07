import subprocess
from rgb_and_depth_processing import track_filter_and_count
import sys

def launch_ros_node():
    # execute roslaunch fields_ignition stereo_simulation.launch folder_path:=/home/pincho/catkin_ws stereo_config_path:=/home/pincho/catkin_ws/src/apple_fields_ignition/fields_ignition/config/stereoParamsSim.yaml
    try:
        ros_node_process = subprocess.Popen(["roslaunch", "fields_ignition", "stereo_simulation.launch", "folder_path:=/home/pincho/catkin_ws", "stereo_config_path:=/home/pincho/catkin_ws/src/apple_fields_ignition/fields_ignition/config/stereoParamsSim.yaml"])
        ros_node_process.communicate()
    except KeyboardInterrupt as e:
        track_filter_and_count(working_directory=sys.argv[1])

if __name__ == "__main__":
    # working_directory = sys.argv[1]
    launch_ros_node()
    