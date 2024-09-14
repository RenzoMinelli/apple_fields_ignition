# importar configuraciones
import configparser
import argparse
import os

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
    
    for c in cameras:
        coeficientes[c] = {}
        for f in filters:
            coeficientes[c][f] = []
            for i in range(1, 21):
                # Leer el resutltado
                config.read(f"{results_path}/{c}_{f}_train_{i}.ini")

                apple_count = config.getint('RESULTS', 'apple_count')

                # Calcular y agregar el coeficiente a la lista
                current_coef = ground_truth[f"train_{i}"]/apple_count
                coeficientes[c][f].append(current_coef)
            
            # Calcular el promedio de los coeficientes calculados para c y f
            coeficientes[c][f] = sum(coeficientes[c][f])/len(coeficientes[c][f])
    print(f"Promedios calculados:\ndepth: {coeficientes['depth']}\nstereo: {coeficientes['stereo']}")

def ajuste_por_regresion_lineal(results_path, config):
    print('Not implemented yet')

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



# for c in cameras:
#     for f in filters:
        