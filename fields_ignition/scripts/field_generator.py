# Script que genera los campos de manzanas en base a los parámetros
# que se pasan en los argumentos al llamar al script.
# 

import subprocess
import os
import sys
import numpy as np
from cookiecutter.main import cookiecutter
from pathlib import Path
import json
import shutil
import base64

import argparse
from subprocess import run, PIPE


fields_ignition_path = run(['rospack', 'find', 'fields_ignition'], stdout=PIPE).stdout.decode("utf-8").replace('\n', '')
parser = argparse.ArgumentParser()

parser.add_argument(
    "-s", "--seed", dest="seed", type=int, default=np.random.randint(10000)
)

parser.add_argument("-f", "--file", dest="file", default=f"{fields_ignition_path}/one_leader_tree.json")
parser.add_argument('--world_name', dest="world_name", default='apple_field')
parser.add_argument("--model_name_prefix", dest="model_name_prefix", default="apple")
parser.add_argument("--out_path", dest="out_path", default=f'{fields_ignition_path}/generated/')
parser.add_argument("--row_count", dest="row_count", default='1')
parser.add_argument("--row_length", dest="row_length", default='3')
parser.add_argument("--row_dist", dest="row_dist", default='4.0')
parser.add_argument("--crop_dist", dest="crop_dist", default='2')
parser.add_argument("--origin_coordinates", dest="origin_coordinates", default="0 0")
parser.add_argument("--tree_properties", dest="tree_properties", default=None)
parser.add_argument("--camera_position", dest="camera_position", default="0.0012 0 0.45 0 -0.52 1.57079632679")
parser.add_argument("--husky_initial_position", dest="husky_position", default="0 0 0 0 0 0")


args = parser.parse_known_args(sys.argv)[0]

print(args)

TREE_CONFIG_FILE = args.file
SEED = int(args.seed)
WORLD_NAME = args.world_name 
MODEL_NAME_PREFIX = args.model_name_prefix 
OUT_PATH = Path(args.out_path).resolve()
ROW_COUNT = int(args.row_count)
ROW_LENGTH = int(args.row_length) 
ROW_DIST = float(args.row_dist)
CROP_DIST = float(args.crop_dist)
ORIGIN_COORDINATES = [int(i) for i in args.origin_coordinates.split(' ')]
TREE_PROPERTIES = json.loads(base64.urlsafe_b64decode(args.tree_properties).decode())
CAMERA_POSITION = args.camera_position
HUSKY_POSITION = args.husky_position

MODEL_TEMPLATE = Path(f'{fields_ignition_path}/templates/apple_model').resolve()
WORLD_TEMPLATE = Path(f'{fields_ignition_path}/templates/apple_world').resolve()

shutil.rmtree(OUT_PATH, ignore_errors=True)
np.random.seed(SEED)



# helper class to build the markers.json
class Markers:
    markers = []
    last_id = 0

    @staticmethod
    def next_id():
        Markers.last_id += 1
        return Markers.last_id

    @staticmethod
    def reset():
        Markers.markers = []
    
    @staticmethod
    def add_plant(x, y, z):
        id = Markers.next_id()
        Markers.markers.append({
            'marker_type': 'PLANT',
            'id': id,
            'translation': [x, y, z]
        })
        return id
    

    @staticmethod
    def add_fruit(x, y, z, plant_id):
        id = Markers.next_id()
        Markers.markers.append({
            'marker_type': 'FRUIT',
            'id': id,
            'translation': [x, y, z],
            'plant_id': plant_id
        })
        return id
    
    @staticmethod
    def dumps():
        return json.dumps(Markers.markers, indent=4)


models = {'list': []}

Markers.reset()

# Según la cantidad de filas y de columnas
# genera para cada posición un árbol de manzanas,
# que es agregado en el archivo sdf utilizando cookiecutter
for x in range(ROW_COUNT):
    for y in range(ROW_LENGTH):
        print(type(x), type(range(ROW_COUNT)[0]), type(y))
        model_name = 'apple_{}'.format(x * ROW_LENGTH + y)

        cookiecutter(str(MODEL_TEMPLATE),
             output_dir=str(OUT_PATH), 
             overwrite_if_exists=True, 
             no_input=True,
             extra_context={'world_name': WORLD_NAME, 'model_name': model_name})

        x_pos, y_pos, z_pos = ORIGIN_COORDINATES[0] + (x) * ROW_DIST, ORIGIN_COORDINATES[1] + (y) * CROP_DIST, 0
        models['list'].append({
            'model': model_name,
            'name': model_name,
            'pose': '{} {} 0 0 0 0'.format(x_pos, y_pos)
        })
        x_pos += np.random.uniform(-0.1, 0.1)
        y_pos += np.random.uniform(-0.1, 0.1)
        seed = np.random.randint(10000)
        dir = (OUT_PATH / WORLD_NAME / model_name).resolve()
        dir_blender = (f'{fields_ignition_path}/blender')
        blend = str(dir_blender + '/apple_tree.blend')
        script = str(dir_blender + '/apple_gen.py')


        # Si se pasan las propiedades del arbol se guardan en instancia generada y se 
        # define como archivo de configuracion para blender.
        if TREE_PROPERTIES:
            with open(OUT_PATH / WORLD_NAME / 'tree_properties.json', 'w') as outfile:
                json.dump(TREE_PROPERTIES, outfile, indent=4, sort_keys=True)
            
            TREE_CONFIG_FILE = f"{OUT_PATH}/{WORLD_NAME}/tree_properties.json"

        script_line = "blender {} --background --python {} -- --model_dir {} -f {}  --metadata --model".format(blend, script, dir, TREE_CONFIG_FILE)
        print("RUNNING ", script_line)
        subprocess.run(script_line, capture_output=True, shell=True, text=True).stdout
        plant_id = Markers.add_plant(x_pos, y_pos, z_pos)

        with open(dir / 'markers.json') as markers_file:
            plant_markers = json.load(markers_file).get("elements")
            for marker in plant_markers:
                if marker['marker_type'] == 'FRUIT':
                    Markers.add_fruit(
                        marker['translation'][0] + x_pos,
                        marker['translation'][1] + y_pos,
                        marker['translation'][2] + z_pos,
                        plant_id
                    )

cookiecutter(str(WORLD_TEMPLATE),
             output_dir=str(OUT_PATH), 
             overwrite_if_exists=True, 
             no_input=True,
             extra_context={'world_name': WORLD_NAME, 'camera_position': CAMERA_POSITION, 'husky_position': HUSKY_POSITION, 'models': models })

with open(OUT_PATH / WORLD_NAME / 'markers.json', 'w') as outfile:
    json.dump(Markers.markers, outfile, indent=4, sort_keys=True)

with open(OUT_PATH / WORLD_NAME / 'world_params.json', 'w') as outfile:
    json.dump(vars(args), outfile, indent=4, sort_keys=True)