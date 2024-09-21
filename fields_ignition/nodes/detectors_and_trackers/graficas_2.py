import os
import configparser
import matplotlib.pyplot as plt
import argparse

FILTROS = ["sin_filtrado", "filas_posteriores", "kmeans", "punto_medio", "plano"]
AJUSTES = {"apple_count_original": "blue", "apple_count_coef": "orange", "apple_count_reg": "green"}
CURRENT_PATH = os.path.dirname(os.path.realpath(__file__))

# Función para identificar el filtro y el mundo a partir del nombre del archivo
def identificar_filtro_y_mundo(nombre_archivo, path):
    filtro = next((filtro for filtro in FILTROS if filtro in nombre_archivo), None)
    
    if not filtro:
        return None, None

    depth_o_stereo = obtener_depth_o_stereo(nombre_archivo)
    nombre_restante = nombre_archivo.replace(f"{depth_o_stereo}_{filtro}_", "")
    nombre_restante = nombre_restante.split("_")[:-1]
    
    depth_o_stereo = depth_o_stereo[0].upper() + depth_o_stereo[1:]

    return filtro, f"Mundo {depth_o_stereo} {nombre_restante[0]} {nombre_restante[1]}"

# Función para obtener el apple_count desde un archivo .ini
def obtener_apple_count(ruta_archivo):
    config = configparser.ConfigParser()
    config.read(ruta_archivo)
    try:
        apple_count_original = int(config['RESULTS']['apple_count'])
        apple_count_coef = float(config['RESULTS_WITH_COEFFICIENT']['apple_count'])
        apple_count_reg = float(config['RESULTS_WITH_REGRESSION']['apple_count'])
        return {
            "apple_count_original": apple_count_original, 
            "apple_count_coef": apple_count_coef, 
            "apple_count_reg": apple_count_reg
        }
    except KeyError:
        print(f"apple_count no encontrado en {ruta_archivo}")
        return None
    
def obtener_depth_o_stereo(ruta_archivo):
    if "depth" in ruta_archivo:
        return "depth"
    elif "stereo" in ruta_archivo:
        return "stereo"
    else:
        return None

# Función para procesar la carpeta y generar los datos
def procesar_carpeta(carpeta):
    mundos = {
        "Mundo Depth test 1": {},
        "Mundo Depth test 2": {},
        "Mundo Depth test 3": {},
        "Mundo Depth test 4": {},
        "Mundo Depth test 5": {},
        "Mundo Stereo test 1": {},
        "Mundo Stereo test 2": {},
        "Mundo Stereo test 3": {},
        "Mundo Stereo test 4": {},
        "Mundo Stereo test 5": {},
    }

    for archivo in os.listdir(carpeta):
        if archivo.endswith(".ini"):
            filtro, mundo = identificar_filtro_y_mundo(archivo, carpeta)
            if mundo and filtro:
                ruta_archivo = os.path.join(carpeta, archivo)
                apple_count_hash = obtener_apple_count(ruta_archivo)
                if apple_count_hash is not None:
                    mundos[mundo][filtro] = apple_count_hash
    
    return mundos

# Función principal para generar gráficos
def generar_graficos(carpeta, mundos_reales):
    mundos = procesar_carpeta(carpeta)

    # Agregar valores reales a los datos
    for mundo, datos in mundos.items():
        if mundo in mundos_reales:
            lista_reales = mundos_reales[mundo]
            datos["real"] = lista_reales[0]

    # Crear gráficos individuales para cada mundo
    for mundo, datos in mundos.items():
        if datos:  # Si hay datos para este mundo
            filtros_en_datos = [key for key in FILTROS if key in datos]
            num_filtros = len(filtros_en_datos)

            # Crear una nueva figura con subplots para cada filtro
            fig, axs = plt.subplots(1, num_filtros, figsize=(5 * num_filtros, 6))

            if num_filtros == 1:  # Si solo hay un filtro, axs no es un array
                axs = [axs]

            for ax, filtro in zip(axs, filtros_en_datos):
                apple_counts_para_filtro_hash = datos[filtro]
                valores = [apple_counts_para_filtro_hash[key] for key in AJUSTES]
                colores = [AJUSTES[key] for key in AJUSTES]

                # Crear las barras
                bars = ax.bar(AJUSTES.keys(), valores, color=colores)
                
                # Añadir etiquetas encima de cada barra
                for bar in bars:
                    yval = bar.get_height()
                    ax.text(bar.get_x() + bar.get_width()/2, yval, round(yval, 2), 
                            ha='center', va='bottom', fontsize=10)

                if "real" in datos:
                    ax.axhline(y=datos["real"], color='purple', linestyle='--', label=f"Real: {datos['real']}")
                    
                ax.set_title(f"{mundo} {filtro}")
                ax.set_ylabel("Valor")
                ax.set_xlabel("Tipo ajuste")
                ax.legend()

            plt.tight_layout()
            nombre_archivo = f"{CURRENT_PATH}/graficas_experimento/{mundo.replace(' ', '_')}.png"
            plt.savefig(nombre_archivo)  # Guardar la gráfica
            plt.close()  # Cerrar la figura después de guardarla

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generar gráficos desde archivos .ini en una carpeta.")
    parser.add_argument("--carpeta", type=str, help="Ruta a la carpeta que contiene los archivos .ini")

    args = parser.parse_args()

    # Cantidades reales para cada mundo
    mundos_reales = {
        "Mundo Depth test 1": [392],
        "Mundo Depth test 2": [344],
        "Mundo Depth test 3": [378],
        "Mundo Depth test 4": [672],
        "Mundo Depth test 5": [331],
        "Mundo Stereo test 1": [392],
        "Mundo Stereo test 2": [344],
        "Mundo Stereo test 3": [378],
        "Mundo Stereo test 4": [672],
        "Mundo Stereo test 5": [331],
    }

    generar_graficos(args.carpeta, mundos_reales)
