import numpy as np
import math
from utils.branch import Branch


#
# Clase Tree, que genera el esqueleto del árbol a partir de las
# propiedades dadas.
#
class Tree:

    branches = []

    # Recibe un nombre y un diccionario de propiedades.
    def __init__(self, name, properties):
        self.name = name
        self.properties = properties


    # Función que genera las ramas primarias y secundarias del árbol
    def build(self):
        print("Building ", self.name)

        print("--- Generating leaders")
        main_poperties = self.properties["main"]
        leaders_properties = self.properties["branches"]

        # Agrupar de a pares las ramas principales para 
        # posteriormente tengan un nodo en común.
        groups = np.full(round(leaders_properties["count"] / 2), 2)
        if leaders_properties["count"] % 2 != 0:
            groups = np.append(groups, 1)

        # Altura aleatoria del árbol, en el rango dado
        tree_height = np.random.uniform(
            main_poperties["height_range"][0], main_poperties["height_range"][1], 1
        ).round(3)[0]


        # Generar ramas principales
        # NOTA: la implementación no está optimizada para
        # más de una rama principal.
        for index, group_size in enumerate(groups):
            main_branch = Branch(
                "gr_{}_main".format(index),
                origin=[0, 0, 0],
                height=tree_height,
                radius_range=leaders_properties["radius_range"],
                radius_decrease=leaders_properties["radius_decrease"],
                z_offset=0,
                angle=[0, 0, 0],
                min_leaf_fruit_height=0.5,
                brindillas_properties=leaders_properties.get("brindillas", None),
                index="",
            )

            main_branch_coordinates = main_branch.build()
            self.branches += [main_branch]

            # Generar ramas secundarias
            # TODO: Al agregar ramas principales, la posición de las frutas
            # y hojas no es la correcta en relación a la posición de las brindillas.
            # La solución temporal es definir brindillas del largo de las ramas secundarias.
            secondary_leaders_properties = leaders_properties.get("branches")
            if secondary_leaders_properties:

                # Altura a partir de la cual están las ramas secundarias.
                offset_range = max(main_poperties["elements_offset_range"])

                # Se calcula la cantidad de ramas a partir de la altura del árbol,
                # el offset y la propiedad de distancia entre ramas.
                cant_secondary_leaders = math.floor(
                    (tree_height - offset_range)
                    / max(secondary_leaders_properties["distance_between"])
                )

                # Generar las posiciones en el eje z de las ramas.
                # Que se guardan en el array z_positions
                z_positions = []
                for i in range(cant_secondary_leaders):  # range(1):
                    if i == 0:
                        z_positions += [
                            offset_range
                            + np.random.uniform(
                                *secondary_leaders_properties["distance_between"]
                            )
                        ]
                    else:
                        z_positions += [
                            z_positions[i - 1]
                            + np.random.uniform(
                                *secondary_leaders_properties["distance_between"]
                            )
                        ]


                # Para cada rama, dada por su posición en el eje z:
                previous_direction = None
                for sec_index, new_leader_z_offset in enumerate(z_positions):
                    branch_length = np.random.uniform(
                        secondary_leaders_properties["length_range"][0],
                        secondary_leaders_properties["length_range"][1],
                        1,
                    ).round(3)[0]

                    # Elegir la dirección a la que va la rama en los ejes x,y
                    if previous_direction == -np.pi / 2:
                        new_direction = np.random.choice(
                            [-np.pi / 2, np.pi / 2], p=[0.1, 0.9]
                        )
                    else:
                        new_direction = np.random.choice(
                            [-np.pi / 2, np.pi / 2], p=[0.9, 0.1]
                        )
                    previous_direction = new_direction

                    # Elegir aleatoriamente el radio de la rama secundaria
                    max_radius = min(
                        secondary_leaders_properties["radius_range"][0],
                        main_branch.get_radius_given_z(new_leader_z_offset),
                    )

                    # Generar la rama
                    secondary_branch = Branch(
                        "gr_{}_secondary_{}".format(index, sec_index),
                        origin=[*main_branch.get_x_y_given_z(new_leader_z_offset), 0],
                        height=branch_length,
                        radius_range=(
                            secondary_leaders_properties["radius_range"][0],
                            max_radius,
                        ),
                        radius_decrease=secondary_leaders_properties["radius_decrease"],
                        z_offset=new_leader_z_offset,
                        angle=[0, new_direction, 0],  #
                        min_leaf_fruit_height=0.5,
                        index="",
                        brindillas_properties=secondary_leaders_properties.get(
                            "brindillas", None
                        ),
                    )
                    secondary_branch_coordinates = secondary_branch.build()
                    self.branches += [secondary_branch]


    # Función que renderiza las ramas del árbol agregándolas a la colección
    def render(self, collection):
        return_data = []
        for branch in self.branches:
            return_data += [branch.render(collection)]

        return return_data