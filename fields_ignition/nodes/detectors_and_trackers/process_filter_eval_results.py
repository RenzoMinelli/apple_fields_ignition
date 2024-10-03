import os
import configparser
import re

ground_truth_total = {
    "1":151, # test_4, apple_0, apple_1
    "2":123, # test_4, apple_2, apple_3
    "3":126, # test_4, apple_4, apple_5
    "4":137, # test_4, apple_6, apple_7
    "5":135, # test_4, apple_8, apple_9
    "6":150, # test_5, apple_0, apple_1
    "7":139, # test_5, apple_2, apple_3
    "8":140, # test_5, apple_5, apple_6
    "9":130, # test_5, apple_7, apple_8
    "10":145 # test_5, apple_10, apple_11
}

CWD = os.getcwd()

# Directorio donde se encuentran los archivos .ini
# (asegurarse de que estan solo los resultados para la 
# evaluacion de filtrado, es decir el output de eval_filtrado.py)
directory = f"{CWD}/results"

# Diccionario para almacenar los resultados
apple_counts = {}

# Expresión regular para capturar las partes del nombre del archivo
pattern = re.compile(r"(?P<tipo>(.*?))_(?P<filtrado>(\w*(|_)\w*))_filtrado(?P<n>\d+)_(?P<lado>(izq|der))_(?P<variante>(sin|con))_hojas_\d+\.ini")

# Recorre todos los archivos en el directorio
for filename in os.listdir(directory):
    # Verifica si el archivo coincide con el patrón esperado
    if filename.endswith(".ini"):
        match = pattern.match(filename)
        if match:
            # Extrae las partes del nombre del archivo
            tipo = match.group('tipo')
            filtrado = match.group('filtrado')
            n = match.group('n')
            lado = match.group('lado')
            variante = match.group('variante')

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
                if tipo not in apple_counts:
                    apple_counts[tipo] = {}
                if filtrado not in apple_counts[tipo]:
                    apple_counts[tipo][filtrado] = {}
                if n not in apple_counts[tipo][filtrado]:
                    apple_counts[tipo][filtrado][n] = {}
                if variante not in apple_counts[tipo][filtrado][n]:
                    apple_counts[tipo][filtrado][n][variante] = {}

                # Almacena el conteo de manzanas
                apple_counts[tipo][filtrado][n][variante][lado] = apple_count

            except (configparser.NoSectionError, configparser.NoOptionError, ValueError) as e:
                print(f"Error leyendo el archivo {filename}: {e}")
                break
        else:
            print(f"Error: el nombre del archivo {filename} no tiene un nombre adecuado\n \
                  para los archivos resultados de la evaluacion de tracking.")
            break
# Imprime el diccionario de resultados
print("Resultados:")
print(apple_counts)

conteo_calculado = {}

for camera in apple_counts:
    conteo_calculado[camera] = {}
    for filtrado in apple_counts[camera]:
        conteo_calculado[camera][filtrado] = {}
        for n in apple_counts[camera][filtrado]:
            conteo_calculado[camera][filtrado][n] = {}
            for variante in apple_counts[camera][filtrado][n]:
                suma_lados = 0
                for lado in apple_counts[camera][filtrado][n][variante]:
                    suma_lados+=apple_counts[camera][filtrado][n][variante][lado]
                  
                conteo_calculado[camera][filtrado][n][variante] = suma_lados

print('Con lados sumados:')
print(conteo_calculado)

# Guardar el diccionario en un archivo .json
import json
print(f"Guardando resultados en {CWD}/results/apple_counts.json")
with open(f"{CWD}/results/apple_counts.json", "w") as f:
    json.dump(conteo_calculado, f, indent=4)
