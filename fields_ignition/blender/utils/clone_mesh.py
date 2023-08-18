import bpy
import numpy as np
from mathutils import Euler


def select_objects(object_name, ob_pool):
    ob_merged = None
    ob_fruits = []

    def is_match(ob):
        return ob.name.split(".")[0] == object_name

    for ob in ob_pool:
        if is_match(ob):
            ob_merged = ob
        elif ob.parent is not None and is_match(ob.parent):
            if ob.name.startswith("apple"):
                ob_fruits.append(ob)
    if ob_merged is None:
        raise LookupError('Can\'t find object with name "{}"'.format(object_name))

    return ob_merged, ob_fruits


def clone_mesh(collection, object_name, scale_range, location=None, yaw=None):
    # Función que clonea un objeto de la colección
    # Se utiliza para clonar el objeto "hoja" y "manzanas"

    coll_sub_branches = bpy.data.collections["Collection"]
    ob_merged, ob_fruits = select_objects(object_name, coll_sub_branches.all_objects)
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.duplicate(
        {
            "selected_objects": [ob_merged] + ob_fruits,
            "active_object": ob_merged,
        }
    )
    ob_merged, ob_fruits = select_objects(object_name, bpy.context.selected_objects)
    ob_merged.select_set(False)
    mesh = ob_merged.data
    if object_name == "leaf":
        ob_merged.scale = np.random.uniform(*scale_range, 3)  # La hoja mide 1 metro
        ob_merged.rotation_euler = Euler(
            np.random.uniform(-np.pi / 2, np.pi / 2, 3), "XYZ"
        )
        choice = ""
        label = {
            "marker_type": "LEAF",
            "scale": list(ob_merged.scale),
            "rotation": list(ob_merged.rotation_euler),
        }
    if object_name == "apple":
        ob_merged.scale = np.random.uniform(*scale_range, 3)  # La hoja mide 1 metro
        choice = np.random.choice(
            ["apple_texture_1", "apple_texture_2", "apple_texture_4"]
        )
        mat = bpy.data.materials.get(choice)
        try:
            mesh.materials.pop()
            mesh.materials.pop()
            mesh.materials.pop()
        except:
            pass
        mesh.materials.append(mat)
        label = {
            "marker_type": "FRUIT",
            "texture": choice,
            "scale": list(ob_merged.scale),
            "rotation": list(ob_merged.rotation_euler),
        }
    if location is not None:
        ob_merged.location = location
        label["translation"] = list(location)
    if yaw is not None:
        ob_merged.rotation_euler[2] = yaw
        label["rotation"][2] = yaw

    collection.objects.link(ob_merged)
    coll_sub_branches.objects.unlink(ob_merged)
    for ob in ob_fruits:
        collection.objects.link(ob)
        coll_sub_branches.objects.unlink(ob)

    return ob_merged, ob_fruits, label, choice


if __name__ == "__main__":
    if "test_copy" in bpy.data.collections:
        collection = bpy.data.collections["test_copy"]
        for c in collection.objects:
            bpy.data.objects.remove(c)
    else:
        collection = bpy.data.collections.new("test_copy")
        bpy.context.scene.collection.children.link(collection)

    a, b, c = clone_mesh(collection, "apple", (0.1, 0.15), (0, 0, 0), 1.0)
