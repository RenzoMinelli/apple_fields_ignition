import os
import configparser
import matplotlib.pyplot as plt
import argparse

FILTROS = { "sin_filtrado": "blue", "filas_posteriores": "orange", "kmeans":"green", "punto_medio": "pink", "plano": "red" }

# Función para identificar el filtro y el mundo a partir del nombre del archivo
def identificar_filtro_y_mundo(nombre_archivo, path):
    # Buscar el filtro en el nombre del archivo
    filtro = next((filtro for filtro in FILTROS if filtro in nombre_archivo), None)
    
    if not filtro:
        return None, None

    # Remover el filtro del nombre para identificar el mundo
    depth_o_stereo = obtener_depth_o_stereo(nombre_archivo)
    nombre_restante = nombre_archivo.replace(f"{depth_o_stereo}_{filtro}_", "")

    # Mapear el nombre restante al mundo correspondiente
    if "3x5_completo" in nombre_restante:
        if depth_o_stereo == "depth":
            return filtro, "Mundo Depth 3x5 Completo"
        else:
            return filtro, "Mundo Stereo 3x5 Completo"
    elif "3x5_mitad" in nombre_restante:
        if depth_o_stereo == "depth":
            return filtro, "Mundo Depth 3x5 Mitad"
        else:
            return filtro, "Mundo Stereo 3x5 Mitad"
    elif "1x5_completo" in nombre_restante:
        if depth_o_stereo == "depth":
            return filtro, "Mundo Depth 1x5 Completo"
        else:
            return filtro, "Mundo Stereo 1x5 Completo"
    elif "1x5_mitad" in nombre_restante:
        if depth_o_stereo == "depth":
            return filtro, "Mundo Depth 1x5 Mitad"
        else:
            return filtro, "Mundo Stereo 1x5 Mitad"
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
        return int(config['RESULTS']['apple_count'])
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
        "Mundo Depth 1x5 Mitad": {},
        "Mundo Depth 1x5 Completo": {},
        "Mundo Stereo 1x5 Mitad": {},
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
            filtro, mundo = identificar_filtro_y_mundo(archivo, carpeta)
            if mundo and filtro:
                ruta_archivo = os.path.join(carpeta, archivo)
                apple_count = obtener_apple_count(ruta_archivo)
                if apple_count is not None:
                    mundos[mundo][filtro] = apple_count
    
    return mundos

# Función principal para generar gráficos separados
def generar_graficos_separados(carpeta, mundos_reales):
    mundos = procesar_carpeta(carpeta)

    # Agregar valores reales a los datos
    for mundo, datos in mundos.items():
        if mundo in mundos_reales:
            lista_reales = mundos_reales[mundo]
            if len(lista_reales) == 1:
                datos["real"] = lista_reales[0]
            else:
                datos["mitad real"] = lista_reales[0]
                datos["real"] = lista_reales[1]

    # Separar mundos simulados y reales
    mundos_simulados = {k: v for k, v in mundos.items() if "Real" not in k}
    mundos_reales_only = {k: v for k, v in mundos.items() if "Real" in k}

    # Crear gráficos para mundos simulados
    fig_simulado, axs_simulado = plt.subplots(len(mundos_simulados) // 2, 2, figsize=(15, 20))
    fig_simulado.subplots_adjust(hspace=0.364, top=0.971)  # Espacio entre subgráficas y margen superior

    for ax, (mundo, datos) in zip(axs_simulado.flat, mundos_simulados.items()):
        if datos:  # Si hay datos para este mundo
            filtros_en_datos = [key for key in FILTROS if key in datos]
            valores = [datos[key] for key in filtros_en_datos]
            colores = [FILTROS[key] for key in filtros_en_datos]
            if len(valores) == 0:
                continue

            bars = ax.bar(filtros_en_datos, valores, color=colores)
            for bar in bars:
                yval = bar.get_height()
                ax.text(bar.get_x() + bar.get_width() / 2, yval, round(yval, 2), 
                        ha='center', va='bottom', fontsize=10)
                
            if "real" in datos:
                ax.axhline(y=datos["real"], color='purple', linestyle='--', label=f"Real: {datos['real']}")
            if "mitad real" in datos:
                ax.axhline(y=datos["mitad real"], color='yellow', linestyle='-.', label=f"Mitad Real: {datos['mitad real']}")

            ax.set_title(mundo, fontsize=12)  # Mantener título del gráfico individual
            ax.set_ylabel("Valor")
            ax.set_xlabel("Filtro")
            ax.legend()

    plt.tight_layout()
    plt.show()

    # Crear gráficos para mundos reales
    fig_real, axs_real = plt.subplots(len(mundos_reales_only) // 2, 2, figsize=(15, 20))
    fig_real.subplots_adjust(hspace=0.7, top=0.95)  # Espacio entre subgráficas y margen superior

    for ax, (mundo, datos) in zip(axs_real.flat, mundos_reales_only.items()):
        if datos:  # Si hay datos para este mundo
            filtros_en_datos = [key for key in FILTROS if key in datos]
            valores = [datos[key] for key in filtros_en_datos]
            colores = [FILTROS[key] for key in filtros_en_datos]
            if len(valores) == 0:
                continue

            bars = ax.bar(filtros_en_datos, valores, color=colores)
            for bar in bars:
                yval = bar.get_height()
                ax.text(bar.get_x() + bar.get_width() / 2, yval, round(yval, 2), 
                        ha='center', va='bottom', fontsize=10)
            if "real" in datos:
                ax.axhline(y=datos["real"], color='purple', linestyle='--', label=f"Real: {datos['real']}")
            if "mitad real" in datos:
                ax.axhline(y=datos["mitad real"], color='yellow', linestyle='-.', label=f"Mitad Real: {datos['mitad real']}")
            
            ax.set_title(mundo, fontsize=12)  # Mantener título del gráfico individual
            ax.set_ylabel("Valor")
            ax.set_xlabel("Filtro")
            ax.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generar gráficos separados desde archivos .ini en una carpeta.")
    parser.add_argument("--carpeta", type=str, help="Ruta a la carpeta que contiene los archivos .ini")

    args = parser.parse_args()

    # Cantidades reales para cada mundo
    mundos_reales = {
        "Mundo Depth 1x5 Mitad": [175, 350],
        "Mundo Depth 1x5 Completo": [350],
        "Mundo Stereo 1x5 Mitad": [175, 350],
        "Mundo Stereo 1x5 Completo": [350],
        "Mundo Depth 3x5 Mitad": [170, 340],
        "Mundo Depth 3x5 Completo": [340],
        "Mundo Stereo 3x5 Mitad": [170, 340],
        "Mundo Stereo 3x5 Completo": [340],
        "Mundo Real Mitad": [2554, 5109],
        "Mundo Real Completo": [5109],
    }

    generar_graficos_separados(args.carpeta, mundos_reales)
