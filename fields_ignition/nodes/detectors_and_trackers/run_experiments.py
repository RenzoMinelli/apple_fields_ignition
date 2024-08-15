import subprocess
import os 

CWD = os.getcwd()

CONFIG_FILES = [
    "plano.ini",
    "sin_filtro.ini",
    "kmeans.ini",
    "filas_posteriores.ini",
]

if __name__ == "__main__":
    my_env = os.environ.copy()
    my_env["PYTHONPATH"] = f"{CWD}/src/apple_fields_ignition/fields_ignition/nodes"

    for config_file in CONFIG_FILES:
        subprocess.run([
            "python",
            "-m", 
            "detectors_and_trackers.track_and_filter", 
            "--config", 
            f"{CWD}/src/apple_fields_ignition/fields_ignition/nodes/detectors_and_trackers/experiments_configs/{config_file}"
        ], env=my_env)