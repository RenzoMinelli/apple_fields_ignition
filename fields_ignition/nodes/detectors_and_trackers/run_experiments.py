import subprocess
import os 
import argparse

CWD = os.getcwd()

CONFIG_FILES = [
    "plano.ini",
    "sin_filtro.ini",
    "kmeans.ini",
    "filas_posteriores.ini",
]

def correr_experimentos(my_env, bag_name):
    for config_file in CONFIG_FILES:
        print(f"||||||||||||||||||||||||||||| Corriendo experimento con {config_file} |||||||||||||||||||||||||")
        subprocess.run([
            "python",
            "-m", 
            "detectors_and_trackers.track_and_filter", 
            "--config", 
            f"{CWD}/src/apple_fields_ignition/fields_ignition/nodes/detectors_and_trackers/experiments_configs/{config_file}",
            f"--bag_name",
            f"{bag_name}"
        ], env=my_env)

def launch_and_run(bag_file_path, launch_file):
    command = f"source {CWD}/devel/setup.bash && roslaunch fields_ignition {launch_file} bag_file_path:={bag_file_path} folder_path:={CWD}"
    print(f"|||||||||||||||||||||||||||||||||||||| Ejecutando comando: {command}  ||||||||||||||||||||||||||||||||||||||||")
    subprocess.run(["bash", "-c", command], env=my_env)  # Use bash to run the command
    bag_name = bag_file_path.split("/")[-1].split(".")[0]
    correr_experimentos(my_env, bag_name)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--carpeta_bags_reales", required=True)
    parser.add_argument("--carpeta_bags_simulador_stereo", required=True)
    parser.add_argument("--carpeta_bags_simulador_depth", required=True)
    args = parser.parse_args()

    # primero corremos los experimentos con los bags de simulacion stereo
    # obtenemos los bags en la carpeta correspondiente
    my_env = os.environ.copy()
    my_env["PYTHONPATH"] = f"{CWD}/src/apple_fields_ignition/fields_ignition/nodes"
    

    for bag in os.listdir(args.carpeta_bags_simulador_stereo):
        bag_path = f"{args.carpeta_bags_simulador_stereo}/{bag}"
        launch_and_run(bag_path, "stereo_sim_bag.launch")

    for bag in os.listdir(args.carpeta_bags_simulador_depth):
        bag_path = f"{args.carpeta_bags_simulador_depth}/{bag}"
        launch_and_run(bag_path, "stereo_sim_bag_depth.launch")

    for bag in os.listdir(args.carpeta_bags_reales):
        bag_path = f"{args.carpeta_bags_reales}/{bag}"
        launch_and_run(bag_path, "stereo_real_bag.launch")
