import os
import glob
import shutil
import re
import numpy as np



def count_tracked(source_dir):
    # Create a regex pattern to match the folder names
    folder_pattern = re.compile(r'regression_val_(\d+).mp4')

    counts = {
        #"ocsort":{},
        #"bytetrack":{},
        "strongsort": {}
    }
    # Iterate over the folders in the source directory
    for folder in os.listdir(source_dir):
        folder_path = os.path.join(source_dir, folder)
        # Check if the current item is a directory and matches the pattern
        if os.path.isdir(folder_path) and folder_pattern.match(folder):
            for track_method in os.listdir(folder_path):
                if track_method != "strongsort":
                     continue
                
                track_file_name = folder.replace(".mp4", ".txt")
                file = os.path.join(source_dir,folder,track_method, folder+track_method, "tracks", track_file_name)

                count = len(np.unique(np.genfromtxt(file, delimiter=" ")[:,1]))
                frames = int(np.genfromtxt(file, delimiter=" ")[:,0][-1])
                counts[track_method][track_file_name] = [count, frames]
                
                

    return counts            


counts = count_tracked("/home/facundo/catkin_ws/src/apple_fields_ignition/fields_ignition/regression/YOLOv5StrongSortOsnet/Ejecuciones")


print(counts)