import subprocess
import os 
import argparse

CWD = os.getcwd()

CONFIG_FILES = [
    "plano.ini",
    "sin_filtro.ini",
    "kmeans.ini",
    "filas_posteriores.ini",
    "punto_medio.ini",
]

def correr_experimentos(my_env, bag_name, tipo):
    for config_file in CONFIG_FILES:
        print(f"||||||||||||||||||||||||||||| Corriendo experimento con {config_file} |||||||||||||||||||||||||")
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
    print(f"|||||||||||||||||||||||||||||||||||||| Ejecutando comando: {command}  ||||||||||||||||||||||||||||||||||||||||")
    subprocess.run(["bash", "-c", command], env=my_env)  # Use bash to run the command
    bag_name = bag_file_path.split("/")[-1].split(".")[0]
    correr_experimentos(my_env, bag_name, tipo)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--carpeta_simulador", required=False, type=str, default=None)
    parser.add_argument("--carpeta_real", required=False, type=str, default=None)
    args = parser.parse_args()

    # seteamos la variable de entorno PYTHONPATH para que las clses de python puedan ser importadas
    my_env = os.environ.copy()
    my_env["PYTHONPATH"] = f"{CWD}/src/apple_fields_ignition/fields_ignition/nodes"

    if args.carpeta_simulador:
        for bag in os.listdir(args.carpeta_simulador):
            bag_path = f"{args.carpeta_simulador}/{bag}"
            for tipo_camara in ["depth", "stereo"]:
                mundo = bag.split("_")[0]
                launchfile = f"{tipo_camara}_sim_bag.launch"
                launch_and_run(bag_path, launchfile, tipo_camara)

    if args.carpeta_real:
        for bag in os.listdir(args.carpeta_real):
            bag_path = f"{args.carpeta_real}/{bag}"
            launchfile = "stereo_real_bag.launch"
            launch_and_run(bag_path, launchfile, "real")

                           
# python3 -m detectors_and_trackers.run_experiments 
# --carpeta_simulador /home/renzo/catkin_ws/bags_simulador --carpeta_real home/renzo/catkin_ws/bags_real 

# Los nombres de los bags del simualdor deben tener el prefijo 3x5 o 1x5
