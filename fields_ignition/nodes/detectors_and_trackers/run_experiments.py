import subprocess

CONFIG_FILES = [
    "plano.ini",
    "sin_filtro.ini",
    "kmeans.ini",
    "filas_posteriores.ini",
]

if __name__ == "__main__":
    for config_file in CONFIG_FILES:
        subprocess.run([
            "python",
            "-m", 
            "detectors_and_trackers.track_and_filter", 
            "--config", 
            f"/home/renzo/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes/experiments_configs/{config_file}"
        ])