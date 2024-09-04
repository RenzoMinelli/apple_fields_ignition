import os
import configparser
import matplotlib.pyplot as plt
import argparse

FILTROS = { "sin_filtrado": "blue", "filas_posteriores": "orange", "kmeans":"green", "plano": "red" }

# Función para identificar el filtro y el mundo a partir del nombre del archivo
def identificar_filtro_y_mundo(nombre_archivo):
    # Buscar el filtro en el nombre del archivo
    filtro = next((filtro for filtro in FILTROS if filtro in nombre_archivo), None)
    
    if not filtro:
        return None, None

    # Remover el filtro del nombre para identificar el mundo
    nombre_restante = nombre_archivo.replace(f"{filtro}_", "")

    # Mapear el nombre restante al mundo correspondiente
    if "3x5_completo_stereo" in nombre_restante:
        return filtro, "Mundo Stereo 3x5 Completo"
    elif "3x5_mitad_stereo" in nombre_restante:
        return filtro, "Mundo Stereo 3x5 Mitad"
    elif "1x5_completo_stereo" in nombre_restante:
        return filtro, "Mundo Stereo 1x5 Completo"
    elif "3x5_completo_depth" in nombre_restante:
        return filtro, "Mundo Depth 3x5 Completo"
    elif "3x5_mitad_depth" in nombre_restante:
        return filtro, "Mundo Depth 3x5 Mitad"
    elif "1x5_completo_depth" in nombre_restante:
        return filtro, "Mundo Depth 1x5 Completo"
    elif "completo_real" in nombre_restante:
        return filtro, "Mundo Real Completo"
    elif "mitad_real" in nombre_restante:
        return filtro, "Mundo Real Mitad"
    else:
        return None, None

# Función para obtener el apple_count desde un archivo .ini
def obtener_apple_count(ruta_archivo):
    config = configparser.ConfigParser()
    config.read(ruta_archivo)
    try:
        return int(config['FILTRADO_FILAS_POSTERIORES']['apple_count'])
    except KeyError:
        print(f"apple_count no encontrado en {ruta_archivo}")
        return None

# Función para procesar la carpeta y generar los datos
def procesar_carpeta(carpeta):
    mundos = {
        "Mundo Depth 1x5 Completo": {},
        "Mundo Stereo 1x5 Completo": {},
        "Mundo Depth 3x5 Mitad": {},
        "Mundo Depth 3x5 Completo": {},
        "Mundo Stereo 3x5 Mitad": {},
        "Mundo Stereo 3x5 Completo": {},
        "Mundo Real Mitad": {},
        "Mundo Real Completo": {},
    }

    for archivo in os.listdir(carpeta):
        if archivo.endswith(".ini"):
            filtro, mundo = identificar_filtro_y_mundo(archivo)
            if mundo and filtro:
                ruta_archivo = os.path.join(carpeta, archivo)
                apple_count = obtener_apple_count(ruta_archivo)
                if apple_count is not None:
                    mundos[mundo][filtro] = apple_count
    
    return mundos

# Función principal para generar gráficos
def generar_graficos(carpeta, mundos_reales):
    mundos = procesar_carpeta(carpeta)

    # Agregar valores reales a los datos
    for mundo, datos in mundos.items():
        if mundo in mundos_reales:
            datos["real"] = mundos_reales[mundo]

    # Crear gráficos
    fig, axs = plt.subplots(4, 2, figsize=(15, 20))
    fig.subplots_adjust(hspace=0.4)

    for ax, (mundo, datos) in zip(axs.flat, mundos.items()):
        if datos:  # Si hay datos para este mundo
            filtros_en_datos = [key for key in FILTROS if key in datos]
            valores = [datos[key] for key in filtros_en_datos]
            colores = [FILTROS[key] for key in filtros_en_datos]
            if len(valores) == 0:
                continue

            ax.bar(filtros_en_datos, valores, color=colores)
            if "real" in datos:
                ax.axhline(y=datos["real"], color='purple', linestyle='--', label=f"Real: {datos['real']}")
            ax.set_title(mundo)
            ax.set_ylabel("Valor")
            ax.set_xlabel("Filtro")
            ax.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generar gráficos desde archivos .ini en una carpeta.")
    parser.add_argument("--carpeta", type=str, help="Ruta a la carpeta que contiene los archivos .ini")

    args = parser.parse_args()

    # Cantidades reales para cada mundo
    mundos_reales = {
        "Mundo Depth 1x5 Completo": 350,
        "Mundo Stereo 1x5 Completo": 350,
        "Mundo Depth 3x5 Mitad": 170,
        "Mundo Depth 3x5 Completo": 340,
        "Mundo Stereo 3x5 Mitad": 170,
        "Mundo Stereo 3x5 Completo": 340,
        "Mundo Real Mitad": 2554,
        "Mundo Real Completo": 5109,
    }

    generar_graficos(args.carpeta, mundos_reales)
