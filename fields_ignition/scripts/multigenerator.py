# Script que genera múltiples instancias
# de simulador con los mismos parámetros

#!/usr/bin/env python3
import os
from subprocess import Popen, run, PIPE
import base64
import json 

def generate(content):

    fields_ignition_path = run(['rospack', 'find', 'fields_ignition'], stdout=PIPE).stdout.decode("utf-8").replace('\n', '')
    params = ""
    if content.get("world_name"):
        params += " --world_name {}".format(content.get("world_name"))
    
    if content.get("field_id"):
        params += " --out_path {}/generated/{}".format(fields_ignition_path, content.get("field_id"))

    if content.get("row_count"):
        params += " --row_count {}".format(content.get("row_count"))
    
    if content.get("row_length"):
        params += " --row_length {}".format(content.get("row_length"))
    
    if content.get("row_dist"):
        params += " --row_dist {}".format(content.get("row_dist"))
    
    if content.get("crop_dist"):
        params += " --crop_dist {}".format(content.get("crop_dist"))
    
    if content.get("origin_coordinates"):
        params += " --origin_coordinates \"{}\"".format(content.get("origin_coordinates"))

    if content.get("tree_properties"):
        params += " --tree_properties {}".format(base64.urlsafe_b64encode(json.dumps(content.get("tree_properties")).encode()).decode())

    if content.get("camera").get("position"):
        params += " --camera_position \"{}\"".format(content.get("camera").get("position"))
    
    if content.get("husky").get("initial_position"):
        params += " --husky_initial_position \"{}\"".format(content.get("husky").get("initial_position"))
    
    # app.logger.info('python3 {}}/scripts/field_generator.py {}'.format(params))
    p = Popen('python3 {}/scripts/field_generator.py {}'.format(fields_ignition_path, params),  shell=True) # something long running
    # ... do other stuff while subprocess is running
    p.wait()

    return {}


def get_params(field_id):
    return {
        "world_name": "apple_field",
        "field_id": field_id,
        "row_count": 1,
        "row_length": 5,
        "row_dist": 4,
        "crop_dist": 2,
        "origin_coordinates": "-2 0",
        "camera": {
            "position": "0.0012 0 1 0 -0.2618 1.57079632679"
        },
        "husky": {
            "initial_position": "-0.4 0 0 0 0 1.57"
        },
        "tree_properties": {
            "main": {
                "elements_offset_range": [0.5,0.6],
                "height_range": [2.2,2.3] 
            },
            "branches": {
                "count": 1,
                "radius_range": [0.05,0.07],
                "radius_decrease": 0.2,
                "brindillas": {
                    "offset_range": [0.5,0.6],
                    "length_range": [0.20, 1.0],
                    "distance_between": [0.02,0.03],
                    "radius_range": [0.006,0.01],
                    "radius_decrease": 0.2,
                    "element_offset": 0.1
                }
            },
            "fruits": {
                "count_proportion": 0.2,
                "size_range": [0.065, 0.07]
            },
            "leaves":   {
                "count_proportion": 0.3,
                "size_range": [0.1, 0.15]
            }     
        }
    }


for i in range(3):
    name = f"regression_val_{i}"
    print("######################")
    print(f"###{name}###")
    print("######################")
    params = get_params(name)
    generate(params)