import os
import numpy as np

ground_truth_total = {
    "1":392, # test_1
    "2":344, # test_2
    "3":378, # test_3
    "4":672, # test_4
    "5":331, # test_5
    "6":481, # test_6
}

detecciones = {
    "resultados": {
        "depth": {
            "sin_filtrado": {},
            "filas_posteriores": {},
            "kmeans": {},
            "punto_medio": {},
            "plano": {}
        },
        "stereo": {
            "sin_filtrado": {},
            "filas_posteriores": {},
            "kmeans": {},
            "punto_medio": {},
            "plano": {}
        }
    },
    "stereo": {
        "1": {
            "sin_filtrado": {
                "normal": 872,
                "coef": 407,
                "reg": 396
            },
            "filas_posteriores": {
                "normal": 833,
                "coef": 407,
                "reg": 400
            },
            "kmeans": {
                "normal": 470,
                "coef": 409,
                "reg": 391
            },
            "punto_medio": {
                "normal": 473,
                "coef": 398,
                "reg": 385
            },
            "plano": {
                "normal": 654,
                "coef": 420,
                "reg": 403
            }
        },
        "2": {
            "sin_filtrado": {
                "normal": 763,
                "coef": 356,
                "reg": 359
            },
            "filas_posteriores": {
                "normal": 734,
                "coef": 358,
                "reg": 360
            },
            "kmeans": {
                "normal": 398,
                "coef": 346,
                "reg": 354
            },
            "punto_medio": {
                "normal": 412,
                "coef": 347,
                "reg": 354
            },
            "plano": {
                "normal": 539,
                "coef": 346,
                "reg": 352
            }
        },
        "3": {
            "sin_filtrado": {
                "normal": 806,
                "coef": 376,
                "reg": 374
            },
            "filas_posteriores": {
                "normal": 774,
                "coef": 378,
                "reg": 376
            },
            "kmeans": {
                "normal": 438,
                "coef": 381,
                "reg": 375
            },
            "punto_medio": {
                "normal": 462,
                "coef": 389,
                "reg": 380
            },
            "plano": {
                "normal": 609,
                "coef": 391,
                "reg": 383
            }
        },
        "4": {
            "sin_filtrado": {
                "normal": 1402,
                "coef": 655,
                "reg": 578
            },
            "filas_posteriores": {
                "normal": 1341,
                "coef": 655,
                "reg": 609
            },
            "kmeans": {
                "normal": 750,
                "coef": 652,
                "reg": 532
            },
            "punto_medio": {
                "normal": 789,
                "coef": 664,
                "reg": 546
            },
            "plano": {
                "normal": 977,
                "coef": 627,
                "reg": 544
            }
        },
        "5": {
            "sin_filtrado": {
                "normal": 698,
                "coef": 326,
                "reg": 337
            },
            "filas_posteriores": {
                "normal": 662,
                "coef": 323,
                "reg": 330
            },
            "kmeans": {
                "normal": 390,
                "coef": 339,
                "reg": 350
            },
            "punto_medio": {
                "normal": 408,
                "coef": 344,
                "reg": 352
            },
            "plano": {
                "normal": 502,
                "coef": 322,
                "reg": 336
            }
        },
        "6": {
            "sin_filtrado": {
                "normal": 1191,
                "coef": 556,
                "reg": 505
            },
            "filas_posteriores": {
                "normal": 1110,
                "coef": 542,
                "reg": 514
            },
            "kmeans": {
                "normal": 661,
                "coef": 575,
                "reg": 487
            },
            "punto_medio": {
                "normal": 661,
                "coef": 557,
                "reg": 481
            },
            "plano": {
                "normal": 806,
                "coef": 517,
                "reg": 469
            }
        }
    },
    "depth": {
        "1": {
            "sin_filtrado": {
                "normal": 893,
                "coef": 407,
                "reg": 394
            },
            "filas_posteriores": {
                "normal": 892,
                "coef": 407,
                "reg": 394
            },
            "kmeans": {
                "normal": 490,
                "coef": 404,
                "reg": 386
            },
            "punto_medio": {
                "normal": 505,
                "coef": 398,
                "reg": 386
            },
            "plano": {
                "normal": 656,
                "coef": 410,
                "reg": 393
            }
        },
        "2":{
            "sin_filtrado": {
                "normal": 785,
                "coef": 358,
                "reg": 360
            },
            "filas_posteriores": {
                "normal": 780,
                "coef": 356,
                "reg": 359
            },
            "kmeans": {
                "normal": 427,
                "coef": 352,
                "reg": 358
            },
            "punto_medio": {
                "normal": 448,
                "coef": 353,
                "reg": 358
            },
            "plano": {
                "normal": 558,
                "coef": 349,
                "reg": 356
            }
        }, 
        "3":{
            "sin_filtrado": {
                "normal": 828,
                "coef": 377,
                "reg": 374
            },
            "filas_posteriores": {
                "normal": 828,
                "coef": 378,
                "reg": 374
            },
            "kmeans": {
                "normal": 464,
                "coef": 382,
                "reg": 375
            },
            "punto_medio": {
                "normal": 499,
                "coef": 393,
                "reg": 383
            },
            "plano": {
                "normal": 612,
                "coef": 383,
                "reg": 376
            }
        },
        "4":{
            "sin_filtrado": {
                "normal": 1415,
                "coef": 645,
                "reg": 553
            },
            "filas_posteriores": {
                "normal": 1414,
                "coef": 645,
                "reg": 554
            },
            "kmeans": {
                "normal": 778,
                "coef": 641,
                "reg": 515
            },
            "punto_medio": {
                "normal": 826,
                "coef": 651,
                "reg": 545
            },
            "plano": {
                "normal": 1054,
                "coef": 659,
                "reg": 542
            }
        },
        "5":{
            "sin_filtrado": {
                "normal": 712,
                "coef": 324,
                "reg": 338
            },
            "filas_posteriores": {
                "normal": 712,
                "coef": 325,
                "reg": 338
            },
            "kmeans": {
                "normal": 368,
                "coef": 303,
                "reg": 332
            },
            "punto_medio": {
                "normal": 396,
                "coef": 312,
                "reg": 332
            },
            "plano": {
                "normal": 511,
                "coef": 320,
                "reg": 338
            }
        },
        "6":{
            "sin_filtrado": {
                "normal": 1025,
                "coef": 467,
                "reg": 434
            },
            "filas_posteriores": {
                "normal": 1011,
                "coef": 461,
                "reg": 430
            },
            "kmeans": {
                "normal": 595,
                "coef": 490,
                "reg": 433
            },
            "punto_medio": {
                "normal": 603,
                "coef": 475,
                "reg": 435
            },
            "plano": {
                "normal": 706,
                "coef": 441,
                "reg": 411
            }
        }
    }
}

filtros = ["sin_filtrado", "filas_posteriores", "kmeans", "punto_medio", "plano"]
CWD = os.getcwd()

# Directorio donde se encuentran los archivos .ini
# (asegurarse de que estan solo los resultados para la 
# evaluacion de filtrado, es decir el output de eval_filtrado.py)
directory = f"{CWD}/src/apple_fields_ignition/fields_ignition/nodes/detectors_and_trackers/results_eval_coef_reg"


# Calculo de error relativo
for tipo in ["stereo", "depth"]:
    for filtrado in filtros:
        for n in range(1, 7):
            n = f"{n}"
            normal = detecciones[tipo][n][filtrado]["normal"]
            coef = detecciones[tipo][n][filtrado]["coef"]
            reg = detecciones[tipo][n][filtrado]["reg"]

            er_normal = abs(normal - ground_truth_total[f"{n}"])/ground_truth_total[f"{n}"]
            er_coef = abs(coef - ground_truth_total[f"{n}"])/ground_truth_total[f"{n}"]
            er_reg = abs(reg - ground_truth_total[f"{n}"])/ground_truth_total[f"{n}"]
            
            detecciones[tipo][n][filtrado]["ER_NORMAL"] = er_normal
            detecciones[tipo][n][filtrado]["ER_COEF"] = er_coef
            detecciones[tipo][n][filtrado]["ER_REG"] = er_reg

# Calculo promedios de error
for tipo in ["stereo", "depth"]:
    for filtrado in filtros:
        suma_error_normal = 0
        suma_error_coef = 0
        suma_error_reg = 0
        for n in range(1, 7):
            n = f"{n}"
            suma_error_normal += detecciones[tipo][n][filtrado]["ER_NORMAL"]
            suma_error_coef += detecciones[tipo][n][filtrado]["ER_COEF"]
            suma_error_reg += detecciones[tipo][n][filtrado]["ER_REG"]

        detecciones["resultados"][tipo][filtrado]["avg_error_normal"] = round(suma_error_normal/6,4)
        detecciones["resultados"][tipo][filtrado]["avg_error_coef"] = round(suma_error_coef/6,4)
        detecciones["resultados"][tipo][filtrado]["avg_error_reg"] = round(suma_error_reg/6,4)

# calculo de desviacion estandar

for tipo in ["stereo", "depth"]:
    for filtrado in filtros:
        desviacion_normal_datos = []
        desviacion_coef_datos = []
        desviacion_reg_datos = []

        for n in range(1, 7):
            n = f"{n}"
            desviacion_normal_datos.append(detecciones[tipo][n][filtrado]["ER_NORMAL"])
            desviacion_coef_datos.append(detecciones[tipo][n][filtrado]["ER_COEF"])
            desviacion_reg_datos.append(detecciones[tipo][n][filtrado]["ER_REG"])

        desviacion_normal = np.std(desviacion_normal_datos, ddof=1)
        desviacion_coef = np.std(desviacion_coef_datos, ddof=1)
        desviacion_reg = np.std(desviacion_reg_datos, ddof=1)

        detecciones["resultados"][tipo][filtrado]["std_normal"] = round(desviacion_normal,4)
        detecciones["resultados"][tipo][filtrado]["std_coef"] = round(desviacion_coef,4)
        detecciones["resultados"][tipo][filtrado]["std_reg"] = round(desviacion_reg,4)

# Guardar el diccionario en un archivo .json
import json
print(f"Guardando resultados en {directory}/apple_counts.json")
with open(f"{directory}/apple_counts.json", "w") as f:
    json.dump(detecciones, f, indent=4)
