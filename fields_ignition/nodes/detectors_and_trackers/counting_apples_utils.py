import json 
import os 
WORLD_NAME = "stereo_trees_close"

def total_amount_apples(working_directory):
    dir_path = f"{working_directory}/src/apple_fields_ignition/fields_ignition/generated/{WORLD_NAME}/apple_field/"
    apple_amount = 0
    for root, dirs, files in os.walk(dir_path):
        for dir_name in dirs:
            if dir_name.startswith('apple_'):
                data_file = os.path.join(root, dir_name, 'markers.json')
                if os.path.exists(data_file):
                    with open(data_file, 'r') as f:
                        data = json.load(f)
                        apple_amount += data["count_apples"]
    return apple_amount

def total_amount_trees(working_directory):
    dir_path = f"{working_directory}/src/apple_fields_ignition/fields_ignition/generated/{WORLD_NAME}/apple_field/"
    tree_amount = 0
    for root, dirs, files in os.walk(dir_path):
        for dir_name in dirs:
            if dir_name.startswith('apple_'):
                tree_amount += 1
    return tree_amount

def total_amount_apples_for_trees_ids(working_directory, ids):
    dir_path = f"{working_directory}/src/apple_fields_ignition/fields_ignition/generated/{WORLD_NAME}/apple_field/"
    apple_amount = 0
    dir_names_wanted = [f"apple_{x}" for x in ids]
    for root, dirs, files in os.walk(dir_path):
        for dir_name in dir_names_wanted:
            data_file = os.path.join(root, dir_name, 'markers.json')
            if os.path.exists(data_file):
                with open(data_file, 'r') as f:
                    data = json.load(f)
                    apple_amount += data["count_apples"]
    return apple_amount
