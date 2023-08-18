##
# WIP: Generar los recorridos en base a la posición de los árboles siguiente el recorrido.
## NOTA: Los módulos de ROS y los de IGN parecen no estar sincronizados, y hay delay
##       entonces en la rotación, el robot no para exactamente en la posición, por lo que el
##       recorrido no es exacto.
##
import json
import numpy as np
import matplotlib.pyplot as plt
import itertools


def generate_track(markers, world_params):
    row_dist = int(world_params["row_dist"])
    row_count = int(world_params["row_count"])
    row_length = int(world_params["row_length"])
    crop_dist = int(world_params["crop_dist"])

    coordinates = np.array([np.array(plant["translation"])[0:2] for plant in markers])
    plt.scatter(coordinates[:,0], coordinates[:,1], color="red")
    
    # Sort by x
    coordinates = np.around(coordinates).astype(np.int32)
    x_pos = [-round(row_dist / 2) + row_dist*(i) for i in range(row_count)]
    y_pos = [-round(crop_dist / 2) + crop_dist*(i) for i in range(row_length + 1)]
    
    track_coordinates = list(zip(*itertools.product(x_pos, y_pos)))
    plt.scatter(track_coordinates[0], track_coordinates[1], color="green")
    plt.show()
    


f = open("/home/facundo/catkin_ws/src/fields_ingnition/generated/exp1/apple_field/markers.json")
markers = json.load(f)
f = open("/home/facundo/catkin_ws/src/fields_ingnition/generated/exp1/apple_field/world_params.json")
world_params = json.load(f)
generate_track(markers, world_params)



