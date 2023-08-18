import numpy as np
from utils.helpers import Node


def rotation_matrix(alpha, axis):
    # Genera una matriz de rotación tridimensional en función de un ángulo y un eje de rotación.
    # La función recibe el ángulo de rotación "alpha" en radianes y el eje de rotación "axis" ("x", "y" o "z").
    # Dependiendo del eje especificado, se construye la matriz de rotación correspondiente.
    # Para el eje "x", se utiliza la matriz de rotación en el plano XY.
    # Para el eje "y", se utiliza la matriz de rotación en el plano YZ.
    # Para el eje "z", se utiliza la matriz de rotación en el plano ZX.
    # Retorna la matriz de rotación tridimensional como un array NumPy.

    if axis == "x":
        rot = [
            [1, 0, 0],
            [0, np.cos(alpha), -np.sin(alpha)],
            [0, np.sin(alpha), np.cos(alpha)],
        ]
    elif axis == "y":
        rot = [
            [np.cos(alpha), 0, np.sin(alpha)],
            [0, 1, 0],
            [-np.sin(alpha), 0, np.cos(alpha)],
        ]
    else:
        rot = [
            [np.cos(alpha), -np.sin(alpha), 0],
            [np.sin(alpha), np.cos(alpha), 0],
            [0, 0, 1],
        ]
    return np.array(rot)


def rotate_nodes(nodes, alpha, axis):
    # Rota los nodos en función de un ángulo y un eje de rotación.
    # La función recibe una lista de nodos "nodes", el ángulo de rotación "alpha" en radianes
    # y el eje de rotación "axis" ("x", "y" o "z").
    # Itera sobre cada nodo en la lista y realiza una transformación de rotación en función de la matriz de rotación correspondiente.
    # Crea nuevos nodos rotados con las coordenadas actualizadas y los demás atributos sin cambios.
    # Retorna la lista de nodos rotados.

    rotated_nodes = []
    # Transform nodes
    for node in nodes:
        rotated = node.vector() @ rotation_matrix(alpha, axis)
        rotated_nodes.append(
            Node(node.type, rotated[0], rotated[1], rotated[2], node.alpha, node.r)
        )

    return rotated_nodes


def rotate_points(points, alpha, axis):
    # Rota los puntos en función de un ángulo y un eje de rotación.
    # La función recibe una lista de puntos "points", el ángulo de rotación "alpha" en radianes
    # y el eje de rotación "axis" ("x", "y" o "z").
    # Itera sobre cada punto en la lista y realiza una transformación de rotación en función de la matriz de rotación correspondiente.
    # Crea nuevos puntos rotados con las coordenadas actualizadas y los agrega a una lista de puntos rotados.
    # Retorna la lista de puntos rotados.

    rotated_points = []
    for point in points:
        rotated = point @ rotation_matrix(alpha, axis)
        rotated_points.append(rotated)

    return rotated_points


def translate_nodes(nodes, distances):
    # Traslada los nodos en una cantidad especificada en cada eje.
    # La función recibe una lista de nodos "nodes" y una lista de distancias [dx, dy, dz] en cada eje.
    # Construye una matriz de translación en función de las distancias especificadas.
    # Retorna la lista de nodos trasladados.

    translation_matrix = np.array(
        [
            [1, 0, 0, distances[0]],
            [0, 1, 0, distances[1]],
            [0, 0, 1, distances[2]],
            [0, 0, 0, 1],
        ]
    )
    translated_nodes = []
    # Transform nodes
    for node in nodes:
        translated = translation_matrix @ np.append(node.vector(), 1)
        translated_nodes.append(
            Node(
                node.type,
                translated[0],
                translated[1],
                translated[2],
                node.alpha,
                node.r,
            )
        )

    return translated_nodes


def translate_points(points, distances):
    # Traslada los puntos en una cantidad especificada en cada eje.
    # Construye una matriz de translación en función de las distancias especificadas.
    # Retorna la lista de puntos trasladados.

    translation_matrix = np.array(
        [
            [1, 0, 0, distances[0]],
            [0, 1, 0, distances[1]],
            [0, 0, 1, distances[2]],
            [0, 0, 0, 1],
        ]
    )
    translated_points = []
    for point in points:
        translated = translation_matrix @ np.append(point, 1)
        translated_points.append(translated[0:3])
    return translated_points


def apply_to_nodes(nodes, functions):
    # Función que aplica una función a cada coordenada de cada uno de los nodos
    applied = np.array(
        [
            Node(
                node.type,
                functions[0](node.x, node.y, node.z),
                functions[1](node.x, node.y, node.z),
                functions[2](node.x, node.y, node.z),
                node.alpha,
                node.r,
            )
            for node in nodes
        ]
    )

    return applied


def normalize(nodes, maximos, minimos):
    # Normaliza las coordenadas de los nodos en función de los valores máximos y mínimos especificados.
    
    origs = np.copy(nodes)
    max = [origs[0].x, origs[0].y, origs[0].z]
    min = [origs[0].x, origs[0].y, origs[0].z]
    for orig in origs[1:]:
        if orig.x > max[0]:
            max[0] = orig.x
        if orig.y > max[1]:
            max[1] = orig.y
        if orig.z > max[2]:
            max[2] = orig.z
        if orig.x < min[0]:
            min[0] = orig.x
        if orig.y < min[1]:
            min[1] = orig.y
        if orig.z < min[2]:
            min[2] = orig.z
    print(max, min)
    for node in nodes:
        print(node)
        if maximos[0]:
            node.x = (node.x - min[0]) / (max[0] - min[0]) * (
                maximos[0] - minimos[0]
            ) + minimos[0]
        if maximos[1]:
            node.y = (node.y - min[1]) / (max[1] - min[1]) * (
                maximos[1] - minimos[1]
            ) + minimos[1]
        if maximos[2]:
            node.z = (node.z - min[2]) / (max[2] - min[2]) * (
                maximos[2] - minimos[2]
            ) + minimos[2]
        print(node)
        print("####")
    return origs
