import subprocess
from rgb_and_depth_processing import track_filter_and_count
import sys

# example of how to execute
# python3 <path_to_this_file> <path_to_working_directory> <path_to_stereo_config>

def launch_ros_node():
    # execute roslaunch fields_ignition stereo_simulation.launch folder_path:=/home/pincho/catkin_ws stereo_config_path:=/home/pincho/catkin_ws/src/apple_fields_ignition/fields_ignition/config/stereoParamsSim.yaml
    try:
        working_directory=sys.argv[1]
        stereo_config_path = sys.argv[2]
        ros_node_process = subprocess.Popen(["roslaunch", "fields_ignition", "stereo_simulation.launch", "folder_path:=" + working_directory, "stereo_config_path:=" + stereo_config_path])
        ros_node_process.communicate()
    except KeyboardInterrupt as e:
        track_filter_and_count(working_directory)

if __name__ == "__main__":
    launch_ros_node()
