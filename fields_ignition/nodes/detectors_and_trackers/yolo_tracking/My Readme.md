Crear carpeta data con la estructura:


en el root del proyecto correr:
python val_utils/scripts/run_mot_challenge.py --BENCHMARK Manzanas --SPLIT_TO_EVAL train --TRACKERS_TO_EVAL BoxMOT --METRICS HOTA CLEAR Identity VACE --USE_PARALLEL False --NUM_PARALLEL_CORES 1  --DO_PREPROC False
