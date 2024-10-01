import subprocess
import os 
import argparse
import re

CWD = os.getcwd()

CONFIG_FILES = [
    "plano.ini",
    "sin_filtro.ini",
    "kmeans.ini",
    "filas_posteriores.ini",
    "punto_medio.ini",
]

def correr_evaluacion(my_env, bag_name, tipo):
    for config_file in CONFIG_FILES:
        print(f"||||||||||||||||||||||||||||| Evaluando con {config_file} |||||||||||||||||||||||||")
        subprocess.run([
            "python",
            "-m", 
            "detectors_and_trackers.track_and_filter", 
            "--config", 
            f"{CWD}/src/apple_fields_ignition/fields_ignition/nodes/detectors_and_trackers/experiments_configs/{tipo}/{config_file}",
            f"--bag_name",
            f"{bag_name}"
        ], env=my_env)

def launch_and_run(bag_file_path, launch_file, tipo):
    command = f"source {CWD}/devel/setup.bash && roslaunch fields_ignition {launch_file} bag_file_path:={bag_file_path} folder_path:={CWD}"
    print("||||||||||||||||||||||||||||||||||||||")
    print(f"Ejecutando comando: {command}")
    print("||||||||||||||||||||||||||||||||||||||")
    subprocess.run(["bash", "-c", command], env=my_env)  # Use bash to run the command
    bag_name = bag_file_path.split("/")[-1].split(".")[0]
    correr_evaluacion(my_env, bag_name, tipo)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path_variantes", required=False, type=str, default=f"{CWD}")
    args = parser.parse_args()

    # seteamos la variable de entorno PYTHONPATH para que las clses de python puedan ser importadas
    my_env = os.environ.copy()
    my_env["PYTHONPATH"] = f"{CWD}/src/apple_fields_ignition/fields_ignition/nodes"

class NombreDeBagIncorrecto(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)

# Extraer indice y lado a partir del nombre del bag
# Se retorna en orden <indice, lado>
# def parse_bag_filename(bagname):
#     pattern = r"filtrado(\d{1,2})_(der|izq)\.bag"
    
#     # Buscamos coincidencias en el string
#     match = re.match(pattern, bagname)
    
#     if match:
#         # Extraemos el número y la dirección (der o izq)
#         number = int(match.group(1))
#         direction = match.group(2)
#         return number, direction
#     else:
#         raise NombreDeBagIncorrecto(f"El nombre del bag '{bagname}' no es correcto.")

if args.path_variantes:
    for variant in ["con", "sin"]:
        for bag in os.listdir(f"{args.path_variantes}/eeval_filtrado_{variant}_hojas"):
            # index, side = parse_bag_filename(bag)

            print(f"processando bag {bag}")

            bag_path = f"{args.path_variantes}/eeval_filtrado_{variant}_hojas/{bag}"
            launchfile = "depth_sim_bag.launch"
            tipo_camara = "depth"
            launch_and_run(bag_path, launchfile, tipo_camara)
