import bpy
import numpy as np
import math
from utils.helpers import Node
from utils.helpers import create_ring_verts

from utils import transforms

class Branch:
    """
    Clase que genera las ramas del árbol.
    """
    verts = []
    faces = []
    brindillas = None
    DIV = 23  # Cantidad de rectangulos en cada nodo

    # Recibe el tipo, la altura, el radio inicial y el final y el angulo de inclinacion
    def __init__(
        self,
        type,
        origin,
        height,
        radius_range,
        radius_decrease,
        z_offset,
        angle,
        min_leaf_fruit_height,
        index,
        brindillas_properties=None,
    ):
        self.type = type
        self.origin = origin
        self.height = height
        self.r_start = np.random.uniform(radius_range[0], radius_range[1])
        self.r_end = self.r_start * radius_decrease
        self.z_offset = z_offset
        self.angle = angle
        self.min_leaf_fruit_height = min_leaf_fruit_height
        self.index = index
        self.brindillas_properties = brindillas_properties

    def build(self):
        # Función que construye la rama, generando los nodos que la definen.

        node_count = int(np.ceil(self.height / 0.1))  # Cada 10cm un nodo
        nodes = [Node("{}_clear".format(self.type), 0, 0, 0, 0, self.r_start)]
        
        node_heights = [x * 0.1 for x in range(node_count)]

        for index, z in enumerate(node_heights):
            prev = nodes[-1]

            # Genera la inclinación del nodo a partir de la inclinación del anterior
            alpha = prev.alpha + np.pi * (1.0 / 6.0) + np.random.normal(0, 0.1)

            # Genera las coordenadas x,y a partir de las del nodo anterior,
            # permitiendo una leve inclinación
            x = prev.x + np.random.uniform(-0.03, 0.03)
            y = prev.y + np.random.uniform(-0.03, 0.03)

            # Genera el radio del nodo a partir del radio inicial y final, y la altura de la rama.
            r = self.r_start - (self.r_start - self.r_end) * (z / self.height)

            # Crea una nueva instancia del nodo con las propiedades (x,y,z,alpha, r) calculadas
            nodes.append(
                Node(
                    "{}_{}".format(
                        self.type, "clear" if z < self.min_leaf_fruit_height else ""
                    ),
                    x,
                    y,
                    z,
                    alpha,
                    r,
                )
            )

        self.nodes = nodes
        
        # En cada nodo agrega ramas más cortas, llamadas brindillas:
        if self.brindillas_properties:
            self.generate_brindillas()

        self.nodes = self.nodes

        return self.nodes

    def render(self, collection):
        # Función renderiza la rama como objeto de blender.
        # Para cada nodo genera los vértices para que tenga forma cilíndrica,
        # luego agrega las brindillas y aplica transformaciones geométricas
        # para posicionarlas en el lugar correspondiente.

        self.generate_verts()

        brindillas_rendered = []
        if self.brindillas:
            for index, sub_branch in enumerate(self.brindillas):
                sub_branch.generate_verts()
                brindillas_faces = sub_branch.faces
                brindillas_verts = sub_branch.verts
                brindillas_verts = transforms.rotate_points(
                    brindillas_verts, self.angle[0], "x"
                )
                brindillas_verts = transforms.rotate_points(
                    brindillas_verts, self.angle[2], "z"
                )
                brindillas_verts = transforms.rotate_points(
                    brindillas_verts, self.angle[1], "y"
                )
                brindillas_verts = transforms.translate_points(
                    brindillas_verts, [0, 0, self.z_offset]
                )

                # Mover la rama al origen
                brindillas_verts = transforms.translate_points(
                    brindillas_verts, self.origin
                )

                ## Transformaciones geométricas
                brindillas_nodes = [(node.x, node.y, node.z) for node in self.nodes]
                brindillas_nodes = transforms.rotate_points(
                    brindillas_nodes, self.angle[0], "x"
                )
                brindillas_nodes = transforms.rotate_points(
                    brindillas_nodes, self.angle[2], "z"
                )
                brindillas_nodes = transforms.rotate_points(
                    brindillas_nodes, self.angle[1], "y"
                )
                brindillas_nodes = transforms.translate_points(
                    brindillas_nodes, [0, 0, self.z_offset]
                )

                # Mover nuevamente al origen
                brindillas_nodes = transforms.translate_points(
                    brindillas_nodes, self.origin
                )

                for index, node in enumerate(self.nodes):
                    node.x = brindillas_nodes[index][0]
                    node.y = brindillas_nodes[index][1]
                    node.z = brindillas_nodes[index][2]

                mesh = bpy.data.meshes.new("brindilla")
                mesh.from_pydata(brindillas_verts, [], brindillas_faces)
                mesh.validate()
                mesh.update(calc_edges=True)

                self.faces_centers = []
                for face in brindillas_faces:
                    center = self.get_center(face, brindillas_verts)
                    if center is not None:
                        self.faces_centers.append(center)

                object_built = bpy.data.objects.new(
                    self.type + "_brindilla_{}".format(index), mesh
                )

                collection.objects.link(object_built)

                mat = bpy.data.materials.get("Branch1")
                object_built.data.materials.append(mat)

                brindillas_rendered.append(object_built)

        # Define mesh and object
        mesh = bpy.data.meshes.new(self.type)

        # Create mesh
        mesh.from_pydata(self.verts, [], self.faces)
        mesh.validate()
        mesh.update(calc_edges=True)

        object_built = bpy.data.objects.new(self.type, mesh)

        collection.objects.link(object_built)

        # Asigna como material la textura "Branch1"
        mat = bpy.data.materials.get("Branch1")
        object_built.data.materials.append(mat)

        return object_built, self.nodes


    def generate_brindillas(self):
        # Función que genera las ramas más cortas (Brindillas, en las que van a estar las frutas y hojas)

        # Calcula la cantidad de brindillas a partir del tamaño de la rama, el offset y la distancia entre estas.
        cant_brindillas = math.floor(
            (self.height - max(self.brindillas_properties["offset_range"]))
            / (sum(self.brindillas_properties["distance_between"]) / 2)
        )

        # Genera las posiciones aleatorias de cada brindilla en la rama
        z_positions = []
        for i in range(cant_brindillas):  # range(1):
            if i == 0:
                z_positions += [
                    np.random.uniform(*self.brindillas_properties["offset_range"])
                    + np.random.uniform(*self.brindillas_properties["distance_between"])
                ]
            else:
                z_positions += [
                    z_positions[i - 1]
                    + np.random.uniform(*self.brindillas_properties["distance_between"])
                ]


        branches = []
        for sec_index, new_leader_z_offset in enumerate(z_positions):

            # Define el largo de la brindilla a partir de las propiedades
            branch_length = np.random.uniform(
                self.brindillas_properties["length_range"][0],
                self.brindillas_properties["length_range"][1],
                1,
            ).round(3)[0]

            # Elige la diracción de la rama
            new_direction_y = np.random.uniform(-np.pi / 2, np.pi / 2)
            new_direction_x = np.random.uniform(-np.pi / 2, np.pi / 2)

            # Define el radio a partir de las propiedades y del largo de la rama
            max_radius = min(
                self.brindillas_properties["radius_range"][0],
                self.get_radius_given_z(new_leader_z_offset),
            )

            # Crea la nueva rama
            brindilla = Branch(
                "{}_brindilla_{}".format(self.type, sec_index),
                origin=[*self.get_x_y_given_z(new_leader_z_offset), 0],
                height=branch_length,
                radius_range=(
                    self.brindillas_properties["radius_range"][0],
                    max_radius,
                ),
                radius_decrease=self.brindillas_properties["radius_decrease"],
                z_offset=new_leader_z_offset,
                angle=[new_direction_x, new_direction_y, 0],  #
                min_leaf_fruit_height=0.5,
                index="",
            )

            secondary_branch_coordinates = brindilla.build()
            branches += [brindilla]
        self.brindillas = branches

    def generate_verts(self):
        # Generar anillos
        prev_ring = create_ring_verts(self.nodes[1], self.DIV)
        self.verts = self.verts + prev_ring

        # Cerrar el primer anillo (punta de la rama);
        self.verts.append((self.nodes[1].x, self.nodes[1].y, self.nodes[0].z - 0.02))
        for i in range(self.DIV):
            vi = 1 + i
            vip = 1 + ((i + 1) % self.DIV)
            self.faces.append((vi, vip, len(self.verts) - 1))

        for node in self.nodes[1:]:
            #    r = r_start - (r_start - r_end) * (z_step / z_steps)
            vert_start = len(self.verts) - self.DIV
            ring = create_ring_verts(node, self.DIV)
            self.verts = self.verts + ring
            prev_ring = ring

            # Arma cada rectangulo, que corresponde a una cara
            for i in range(self.DIV):
                vi = vert_start + i
                vip = vert_start + ((i + 1) % self.DIV)
                self.faces += [(vi, vip, vip + self.DIV, vi + self.DIV)]

        # Cerrar el anillo
        last_ring_start = len(self.verts) - self.DIV
        self.verts.append((self.nodes[-1].x, self.nodes[-1].y, self.nodes[-1].z + 0.01))
        for i in range(self.DIV):
            vi = last_ring_start + i
            vip = last_ring_start + ((i + 1) % self.DIV)
            self.faces.append((vi, vip, len(self.verts) - 1))

        # Rotations
        self.verts = transforms.rotate_points(self.verts, self.angle[0], "x")
        self.verts = transforms.rotate_points(self.verts, self.angle[2], "z")
        self.verts = transforms.rotate_points(self.verts, self.angle[1], "y")
        self.verts = transforms.translate_points(self.verts, [0, 0, self.z_offset])

        # Move to origin
        self.verts = transforms.translate_points(self.verts, self.origin)

        ## Transformaciones 
        brindillas_nodes = [(node.x, node.y, node.z) for node in self.nodes]
        brindillas_nodes = transforms.rotate_points(
            brindillas_nodes, self.angle[0], "x"
        )
        brindillas_nodes = transforms.rotate_points(
            brindillas_nodes, self.angle[2], "z"
        )
        brindillas_nodes = transforms.rotate_points(
            brindillas_nodes, self.angle[1], "y"
        )
        brindillas_nodes = transforms.translate_points(
            brindillas_nodes, [0, 0, self.z_offset]
        )

        # Mover al origen
        brindillas_nodes = transforms.translate_points(brindillas_nodes, self.origin)

        for index, node in enumerate(self.nodes):
            node.x = brindillas_nodes[index][0]
            node.y = brindillas_nodes[index][1]
            node.z = brindillas_nodes[index][2]


    def get_radius_given_z(self, z):
        # Encuentra el radio correspondiente al valor z más cercano en la lista de nodos.
        # La función recibe un valor z y busca el nodo cuyo valor z se acerque más al valor dado.
        # Luego devuelve el radio del nodo.

        best_node = self.nodes[0]
        for node in self.nodes[1:]:
            if np.abs(node.z - z) < np.abs(best_node.z - z):
                best_node = node

        return best_node.r

    def get_x_y_given_z(self, z):
        # Encuentra las coordenadas x,y correspondiente al valor z más cercano en la lista de nodos.
        # La función recibe un valor z y busca el nodo cuyo valor z se acerque más al valor dado.
        # Luego devuelve las coordenadas x,y del nodo.

        best_node = self.nodes[0]
        for node in self.nodes[1:]:
            if np.abs(node.z - z) < np.abs(best_node.z - z):
                best_node = node

        return best_node.x, best_node.y

    def get_center(self, face, vertices):
        # Calcula el centro de una cara de 4 vértices utilizando las coordenadas de los vértices.
        # La función recibe una lista de índices "face" que especifica los vértices de la cara
        # y una lista de coordenadas "vertices" que contiene los puntos de los vértices.
        # Retorna el centro de la cara o None si la cara no tiene 4 vértices o si ocurre una excepción.
        try:
            if len(face) == 4:
                vertices_coord = [vertices[i] for i in face]
                p = [
                    (
                        vertices_coord[0][i]
                        + vertices_coord[1][i]
                        + vertices_coord[2][i]
                        + vertices_coord[3][i]
                    )
                    / 4
                    for i in range(3)
                ]
                return p
        except Exception as e:
            pass
        return None

   
    def __str__(self):
        return "{}: ({}, {}, {}, {})".format(
            self.type,
            self.height,
            self.r_start,
            self.r_end,
            self.z_offset,
            self.angle,
            self.min_leaf_fruit_height,
        )





