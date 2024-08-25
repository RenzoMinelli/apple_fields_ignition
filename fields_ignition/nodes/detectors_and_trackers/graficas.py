import matplotlib.pyplot as plt

# Actualización de los datos con las cantidades reales
mundos = {
    "Mundo Depth 1x5": {"sin_filtro": 399, "filas_posteriores": 398, "kmeans": 178, "plano": 0, "real": 350},
    "Mundo Depth 3x5 Mitad": {"sin_filtro": 508, "filas_posteriores": 374, "kmeans": 299, "plano": 290, "real": 170},
    "Mundo Depth 3x5 Completo": {"sin_filtro": 1269, "filas_posteriores": 794, "kmeans": 613, "plano": 718, "real": 340},
    "Mundo Stereo 1x5": {"sin_filtro": 357, "filas_posteriores": 348, "kmeans": 253, "plano": 320, "real": 350},
    "Mundo Stereo 3x5 Mitad": {"sin_filtro": 402, "filas_posteriores": 334, "kmeans": 290, "plano": 286, "real": 170},
    "Mundo Stereo 3x5 Completo": {"sin_filtro": 1101, "filas_posteriores": 804, "kmeans": 590, "plano": 670, "real": 340},
    "Mundo Real Mitad": {"sin_filtro": 4082, "filas_posteriores": 3852, "kmeans": 3387, "plano": 1938, "real": 2554},
    "Mundo Real Completo": {"sin_filtro": 8049, "filas_posteriores": 7376, "kmeans": 6449, "plano": 4192, "real": 5109},
}

# Crear gráficos
fig, axs = plt.subplots(4, 2, figsize=(15, 20))
fig.subplots_adjust(hspace=0.4)

for ax, (mundo, datos) in zip(axs.flat, mundos.items()):
    ax.bar(list(datos.keys())[:-1], list(datos.values())[:-1], color=['blue', 'orange', 'green', 'red'])
    ax.axhline(y=datos["real"], color='purple', linestyle='--', label=f"Real: {datos['real']}")
    ax.set_title(mundo)
    ax.set_ylabel("Valor")
    ax.set_xlabel("Filtro")
    ax.legend()

plt.tight_layout()
plt.show()
