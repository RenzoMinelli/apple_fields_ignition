import sys
import bpy
import numpy as np
import json
from pathlib import Path
import argparse
import os

print(sys.version_info)
dir = os.path.dirname(bpy.data.filepath)
if not dir in sys.path:
    sys.path.append(dir)

import utils

# this next part forces a reload in case you edit the source after you first start the blender session
import importlib

importlib.reload(utils)
from utils.clone_mesh import clone_mesh
from utils.tree import Tree

parser = argparse.ArgumentParser()
parser.add_argument(
    "-o", "--model_dir", dest="model_dir", default="generated/test_apple"
)
parser.add_argument(
    "-s", "--seed", dest="seed", type=int, default=np.random.randint(10000)
)

parser.add_argument("-f", "--file", dest="file")

parser.add_argument("--metadata", action='store_true')
parser.add_argument("--video", action='store_true')
parser.add_argument("--model", action='store_true')

if "--" in sys.argv:
    argv = sys.argv[sys.argv.index("--") + 1 :]
else:
    argv = []
args = parser.parse_known_args(argv)[0]
print(args)
np.random.seed(args.seed)


model_dir = Path(args.model_dir)
model_dir.mkdir(parents=True, exist_ok=True)


"""
    Comienza a contruir el arbol
"""

# Definimos una colección new_plant donde se guarda el nuevo arbol
if "new_plant" in bpy.data.collections:
    collection = bpy.data.collections["new_plant"]
    for c in collection.objects:
        bpy.data.objects.remove(c)
else:
    collection = bpy.data.collections.new("new_plant")
    bpy.context.scene.collection.children.link(collection)

bpy.ops.object.select_all(action="DESELECT")


# Cargamos el archivo de formato
# Opening JSON file
f = open(Path(args.file))

# returns JSON object as
# a dictionary
tree_properties = json.load(f)


tree = Tree("Apple Tree", tree_properties)
tree.build()
tree.render(collection)

for c in collection.objects:
    c.select_set(True)

bpy.context.view_layer.objects.active = collection.objects[0]
bpy.ops.object.join()

print("----- Estructura generada, agregando hojas y frutas")


#
# Agregamos las hojas y las manzanas
#

brindillas = []
for branch in tree.branches:
    if branch.brindillas:
        brindillas += branch.brindillas


output = {"count_apples": 0, "count_leaves": 0, "elements": []}

ob_fruit_groups = {}
ob_fruits = []
ob_meshes = []
ob_leaves = []

# En cada brindilla agrega manzanas y hojas
for branch in brindillas:

    # add fruits
    for node in branch.nodes[2:]:
        if np.random.uniform() < tree_properties["fruits"]["count_proportion"]:
            mesh_name = "apple"
            ob_m, ob_f, marker, texture = clone_mesh(
                collection,
                mesh_name,
                tree_properties["fruits"]["size_range"],
                (node.x, node.y, node.z),
                node.alpha,
            )
            ob_fruits += ob_f
            if texture in ob_fruit_groups:
                ob_fruit_groups[texture].append(ob_m)
            else:
                ob_fruit_groups[texture] = [ob_m]

            output["elements"].append(marker)
            output["count_apples"] += 1

    #add leaves
    for index, node in enumerate(branch.nodes[2:]):
        for z in np.random.uniform(branch.nodes[index - 1].z, node.z, 10):
            if np.random.uniform() < tree_properties["leaves"]["count_proportion"]: #0.5
                x, y = branch.get_x_y_given_z(z)
                mesh_name = "leaf"
                ob_m, ob_f, marker, choice = clone_mesh(
                    collection, mesh_name, tree_properties["leaves"]["size_range"], (x, y, z), node.alpha
                )
                ob_leaves.append(ob_m)
                output["elements"].append(marker)
                output["count_leaves"] += 1

    markers = []

    def add_marker(location, type="FRUIT"):
        marker = {
            "type": type,
            "translation": [location[0], location[1], location[2]],
        }
        markers.append(marker)


# Agrupa las frutas según la textura
for fruit_group in ob_fruit_groups:
    bpy.ops.object.join({
        'selected_objects': ob_fruit_groups[fruit_group],
        'selected_editable_objects': ob_fruit_groups[fruit_group],
        'active_object': ob_fruit_groups[fruit_group][0]
    })

bpy.ops.object.join({
        'selected_objects': ob_leaves,
        'selected_editable_objects': ob_leaves,
        'active_object': ob_leaves[0]
    })


for ob in ob_fruits:
    bpy.ops.object.parent_clear(
        {"selected_objects": [ob]}, type="CLEAR_KEEP_TRANSFORM"
    )
    add_marker(ob.location)
    
    bpy.ops.object.delete({"selected_objects": [ob]}, use_global=True)


bpy.ops.object.join({
    'selected_objects': collection.objects, 
    'selected_editable_objects': collection.objects,
    'active_object': collection.objects[0]
})


ob_merged = collection.objects[0]
bpy.ops.mesh.separate({
    'selected_editable_objects': [ob_merged]
}, type='MATERIAL')


for ob in collection.all_objects:
    print(ob.material_slots[0].material.name, ob.material_slots)
    ob.name = ob.material_slots[0].material.name

# Exportamos la metadata del arbol
if args.metadata:
    print("Generando metadata")
    m = open(os.path.join(model_dir, "markers.json"), "a")

    m.write(json.dumps(output, indent=4))
    m.close()

if args.video:
    print("----Generando video")
    bpy.data.scenes["Scene"].render.image_settings.file_format = "FFMPEG"
    bpy.data.scenes["Scene"].render.filepath = str(model_dir) + "/"
    bpy.data.scenes["Scene"].render.fps = 12
    bpy.data.scenes["Scene"].render.image_settings.compression = 50
    # Set output format
    bpy.data.scenes["Scene"].render.ffmpeg.format = "MPEG4"

    # Set the codec
    bpy.data.scenes["Scene"].render.ffmpeg.codec = "H264"

    # Export video
    bpy.ops.render.render(animation=True, write_still=True)

if args.model:
    print("----Generando modelo")
    for c in collection.objects:
        # remove the material (it will be added in the SDF file)
        c.data.materials.clear()
        c.select_set(True)
    bpy.ops.wm.collada_export(
        filepath=str(model_dir / "meshes/apple.dae"),
        check_existing=True,
        selected=True,
    )
