# importar configuraciones
import configparser
import argparse
import os
import numpy as np
import os
from sklearn.linear_model import LinearRegression
import joblib
import json

cameras = [
  'depth',
  'stereo'
]

filters = [
  'sin_filtrado',
  'filas_posteriores',
  'kmeans',
  'plano',
  'punto_medio'
]

ground_truth = {
    "train_1" : 346,
    "train_2" : 365,
    "train_3" : 378,
    "train_4" : 376,
    "train_5" : 369,
    "train_6" : 375,
    "train_7" : 367,
    "train_8" : 390,
    "train_9" : 327,
    "train_10": 394,
    "train_11": 373,
    "train_12": 353,
    "train_13": 388,
    "train_14": 379,
    "train_15": 373,
    "train_16": 380,
    "train_17": 365,
    "train_18": 366,
    "train_19": 324,
    "train_20": 351
}

def quitar_timestamps(folder_path):
    for filename in os.listdir(folder_path):
        parts = filename.split("_")

        if parts[-1].replace(".ini", "").isdigit() and len(parts[-1].replace(".ini", "")) == 10:

            # Remover el timestamp y rearmar el nombre.
            new_filename = "_".join(parts[:-1]) + ".ini"
            
            # Get full paths for renaming
            old_file = os.path.join(folder_path, filename)
            new_file = os.path.join(folder_path, new_filename)
            
            # Rename the file
            os.rename(old_file, new_file)
            print(f"Archivo de resultado renombrado: \n{filename} -> {new_filename}")

    print("Todos los archivos de resultados fueron renombrados.")

def coeficiente_de_ajuste(results_path, config):
    coeficientes = {}
    
    # Crear una carpeta para guardar los coeficientes si no existe
    coef_dir = os.path.join(results_path, "coeficientes")
    if not os.path.exists(coef_dir):
        os.makedirs(coef_dir)
    
    for c in cameras:
        coeficientes[c] = {}
        for f in filters:
            coeficientes[c][f] = []
            for i in range(1, 21):
                # Leer el resultado
                config.read(f"{results_path}/{c}_{f}_train_{i}.ini")
                apple_count = config.getint('RESULTS', 'apple_count')

                # Calcular y agregar el coeficiente a la lista
                current_coef = ground_truth[f"train_{i}"] / apple_count
                coeficientes[c][f].append(current_coef)
            
            # Calcular el promedio de los coeficientes calculados para c y f
            coeficientes[c][f] = sum(coeficientes[c][f]) / len(coeficientes[c][f])
        
        # Guardar los coeficientes en un archivo JSON por cámara
        coef_filename = os.path.join(coef_dir, f"coeficientes_{c}.json")
        with open(coef_filename, 'w') as coef_file:
            json.dump(coeficientes[c], coef_file, indent=4)
        print(f"Coeficientes guardados: {coef_filename}")

    print(f"Promedios calculados:\ndepth: {coeficientes['depth']}\nstereo: {coeficientes['stereo']}")


def ajuste_por_regresion_lineal(results_path, config):
    # Crear una carpeta para guardar los modelos si no existe
    models_dir = os.path.join(results_path, "modelos_regresion")
    if not os.path.exists(models_dir):
        os.makedirs(models_dir)

    # Recorrer las cámaras y los filtros para entrenar y guardar los modelos
    for c in cameras:
        for f in filters:
            x_vals = []
            y_vals = []

            for i in range(1, 21):
                # Leer el resultado predicho
                config.read(f"{results_path}/{c}_{f}_train_{i}.ini")
                apple_count = config.getint('RESULTS', 'apple_count')

                # Agregar el valor predicho (x) y el valor real (y) a las listas
                x_vals.append(apple_count)
                y_vals.append(ground_truth[f"train_{i}"])

            # Convertir las listas en arreglos numpy para sklearn
            x_vals = np.array(x_vals).reshape(-1, 1)  # reshaped para asegurarse de que sea un vector columna
            y_vals = np.array(y_vals)

            # Crear el modelo de regresión lineal y ajustarlo con los datos
            model = LinearRegression()
            model.fit(x_vals, y_vals)

            # Guardar el modelo utilizando joblib
            model_filename = os.path.join(models_dir, f"modelo_{c}_{f}.pkl")
            joblib.dump(model, model_filename)
            print(f"Modelo guardado: {model_filename}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--results_path", required=True)
    args = parser.parse_args()

    # renombro los archivos para quitarles el timestamp
    quitar_timestamps(args.results_path)

    config = configparser.ConfigParser()

    # Calcular los coeficientes de ajuste para cada tipo de camara y cada tipo de filtrado
    coeficiente_de_ajuste(args.results_path, config)

    # Calcular rectas y graficar regresion lineal
    ajuste_por_regresion_lineal(args.results_path, config)

# pip install scikit-learn