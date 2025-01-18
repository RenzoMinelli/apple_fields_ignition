import os
import numpy as np
import re
import configparser

mundos = ["1x5", "3x5", "real"]
filtros = ["sin_filtrado", "filas_posteriores", "kmeans", "punto_medio", "plano"]
tipos_camara = ["stereo", "depth"]
recorridos = ["completo", "mitad"] 

ground_truth_total = {
    "1x5_completo":350,
    "1x5_mitad":175, 
    "3x5_completo":340,
    "3x5_mitad":170,
    "real_mitad":2554,
    "real_completo":5109
}

detecciones = {

}

CWD = os.getcwd()

# Directorio donde se encuentran los archivos .ini
# (asegurarse de que estan solo los resultados para la 
# evaluacion de filtrado, es decir el output de eval_filtrado.py)
directory = f"{CWD}/src/apple_fields_ignition/fields_ignition/nodes/detectors_and_trackers/experimento_recorrido"


# Expresión regular para capturar las partes del nombre del archivo
pattern = re.compile(r"(?P<tipo>(.*?))_(?P<filtrado>(\w*(|_)\w*))_(?P<mundo>(1x5|3x5|real))_(?P<recorrido>(mitad|completo))_\d+\.ini")

# Recorre todos los archivos en el directorio
for filename in os.listdir(directory):
    # Verifica si el archivo coincide con el patrón esperado
    if filename.endswith(".ini"):
        match = pattern.match(filename)
        if match:
            # Extrae las partes del nombre del archivo
            tipo = match.group('tipo')
            filtrado = match.group('filtrado')
            mundo = match.group('mundo')
            recorrido = match.group('recorrido')

            print(f"datos extraidos del nombre: {tipo}, {filtrado}, {mundo}, {recorrido}")
            # Ruta completa del archivo
            file_path = os.path.join(directory, filename)

            # Crea un parser para leer el archivo .ini
            config = configparser.ConfigParser()

            # Lee el archivo .ini
            config.read(file_path)

            # Extrae el valor de apple_count de la sección [RESULTS]
            try:
                apple_count = config.getint('RESULTS', 'apple_count')

                # Organiza el diccionario según las partes extraídas
                if tipo not in detecciones:
                    detecciones[tipo] = {}
                if filtrado not in detecciones[tipo]:
                    detecciones[tipo][filtrado] = {}
                if mundo not in detecciones[tipo][filtrado]:
                    detecciones[tipo][filtrado][mundo] = {}
                if recorrido not in detecciones[tipo][filtrado][mundo]:
                    detecciones[tipo][filtrado][mundo][recorrido] = {}

                # Almacena el conteo de manzanas
                detecciones[tipo][filtrado][mundo][recorrido]["conteo"] = apple_count

            except (configparser.NoSectionError, configparser.NoOptionError, ValueError) as e:
                print(f"Error leyendo el archivo {filename}: {e}")
                continue
        else:
            print(f"Error: el nombre del archivo {filename} no tiene un nombre adecuado\n \
                  para los archivos resultados de la evaluacion de tracking.")
            continue

# Calculo de error relativo
for tipo in tipos_camara:
    for filtrado in filtros:
        for mundo in mundos:
            if tipo == "depth" and mundo == "real":
                continue 

            for recorrido in recorridos:
                conteo = detecciones[tipo][filtrado][mundo][recorrido]["conteo"]

                er = abs(conteo - ground_truth_total[f"{mundo}_{recorrido}"])/ground_truth_total[f"{mundo}_{recorrido}"]
                detecciones[tipo][filtrado][mundo][recorrido]["ER"] = round(er,4)

# Guardar el diccionario en un archivo .json
import json
print(f"Guardando resultados en {directory}/resultados_error.json")
with open(f"{directory}/resultados_error.json", "w") as f:
    json.dump(detecciones, f, indent=4)
