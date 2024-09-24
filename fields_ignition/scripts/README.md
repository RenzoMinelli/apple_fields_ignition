# Conteo de frutos de pepita utilizando Deep Learning Y Visión de profundidad

## Instalación

# TODO EL SETUP DE CATKIN Y ROS Y ESAS MRDS SACAR DE LO QUE HICIERON LOS PIBES
# CLONAR EL REPO DE GITLAB (CUANDO LO HAYAMOS SUBIDO)

## Utilización

Este pipeline procesa archivos .bag, por lo que para ejecutarlo simplemente se necesitará un archivo con dicha estensión.

### Recomendación

Se **recomienda leer todo este README** antes de intentar ejecutar el proyecto. Esto te ayudará a entender los requisitos, la configuración adecuada y los pasos necesarios para evitar errores durante la ejecución.

### Remapeo de Tópicos

Se hará uso del archivo `fields_ignition/launch/stereo_real_bag.launch` y `fields_ignition/launch/stereo_sim_bag.launch`
Estos archivos ejecutan el flujo completo para reproducir datos grabados de cámaras estéreo desde un rosbag, remapear los tópicos de imágenes y cámara info, procesar las imágenes para generar mapas de disparidad usando stereo_image_proc, y finalmente, guardar los resultados.

Al utilizar estos archivos `.launch`, es posible que necesites remapear los tópicos de tu archivo `rosbag` para que coincidan con los tópicos esperados por los nodos en el sistema. A continuación se describen los pasos para hacerlo:

1. **Identifica los Tópicos en tu `rosbag`**:
   - Utiliza el comando `rosbag info` para listar los tópicos disponibles en tu archivo `rosbag`. Por ejemplo:
     ```bash
     rosbag info /home/usuario/catkin_ws/datos_apple.bag
     ```

2. **Modifica el Archivo `.launch` a utilizar**:
   - Dentro del archivo `.launch`, busca las líneas que definen los tópicos de entrada para las imágenes y la información de la cámara. Por defecto, se ven así:
     ```xml
     <arg name="left_camera_image_topic" default="/zed_lateral/zed_lateral/left/image_rect_color"/>
     <arg name="right_camera_image_topic" default="/zed_lateral/zed_lateral/right/image_rect_color"/>
     <arg name="left_camera_info_topic" default="/zed_lateral/zed_lateral/left/camera_info"/>
     <arg name="right_camera_info_topic" default="/zed_lateral/zed_lateral/right/camera_info"/>
     ```

3. **Actualiza los Tópicos**:
   - Cambia los valores de los parámetros para que correspondan a los tópicos en tu archivo `rosbag`. Por ejemplo:
     ```xml
     <arg name="left_camera_image_topic" default="/mi_camara/izquierda/image_raw"/>
     <arg name="right_camera_image_topic" default="/mi_camara/derecha/image_raw"/>
     <arg name="left_camera_info_topic" default="/mi_camara/izquierda/camera_info"/>
     <arg name="right_camera_info_topic" default="/mi_camara/derecha/camera_info"/>
     ```

4. **Ejecuta el Archivo `.launch`**:
   - Una vez que hayas actualizado los tópicos en el archivo `.launch`, ejecuta el archivo con el comando `roslaunch`, asegurándote de utilizar los parámetros adecuados. Seguir leyendo para saber cuales son.

### Ejemplo de Remapeo

Si tus tópicos son diferentes, por ejemplo:
- Izquierda: `/mi_camara/izquierda/image_raw`
- Derecha: `/mi_camara/derecha/image_raw`
- Información de la cámara izquierda: `/mi_camara/izquierda/camera_info`
- Información de la cámara derecha: `/mi_camara/derecha/camera_info`

Tu archivo `.launch` se vería así:

```xml
<arg name="left_camera_image_topic" default="/mi_camara/izquierda/image_raw"/>
<arg name="right_camera_image_topic" default="/mi_camara/derecha/image_raw"/>
<arg name="left_camera_info_topic" default="/mi_camara/izquierda/camera_info"/>
<arg name="right_camera_info_topic" default="/mi_camara/derecha/camera_info"/>
```

---
### 1 - Extraccion de imagenes RGB y datos de profundidad del bag.

### Caso de bag con datos reales (no simulados)

Se hará uso del archivo `fields_ignition/launch/stereo_real_bag.launch`. 

Para la ejecucion de este archivo se tienen una serie de parametros con sus respectivos valores por defecto
#### Parámetros del archivo `.launch` y sus valores por defecto:

- **`bag_file_path`**: 
  - **Descripción**: Ruta del archivo `.bag` que contiene los datos grabados de ROS (rosbag).
  - **Valor por defecto**: `/home/user/catkin_ws/bag_mas_lento.bag`
  
- **`folder_path`**: 
  - **Descripción**: Carpeta de trabajo donde se encuentran los archivos de configuración y donde se guardarán los resultados del procesamiento. Es la ruta a la carpeta catkin_ws.
  - **Valor por defecto**: `/home/user/catkin_ws`

- **`post_procesamiento`**: 
  - **Descripción**: Indica si se debe realizar o no post-procesamiento en los datos obtenidos. El post-procesamiento incluye la **detección**, **trackeo**, **filtrado** y **conteo de las manzanas**.
  - **Valor por defecto**: `true`

- **`config`**: 
  - **Descripción**: Ruta al archivo de configuración (`config.ini`) utilizado para el procesamiento posterior, que define ciertos parámetros relacionados con el procesamiento de los datos.
  - **Valor por defecto**: `$(folder_path)/src/apple_fields_ignition/fields_ignition/nodes/config.ini`

- **`bag_playback_speed`**: 
  - **Descripción**: Velocidad de reproducción del archivo `rosbag` durante la ejecución. Este parámetro controla qué tan rápido o lento se reproduce el archivo de datos. El valor adecuado dependerá del hardware. Creado con la intención de no perder datos durante la lectura del bag.
  - **Valor por defecto**: `0.05`


#### Ejemplo de ejecución

```
roslaunch fields_ignition stereo_real_bag.launch \
bag_file_path:=/home/usuario/catkin_ws/datos_apple.bag \
folder_path:=/home/usuario/catkin_ws \
post_procesamiento:=true \
config:=/home/usuario/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes/mi_config.ini \
bag_playback_speed:=0.1
```

### Caso de bag con datos simulados

Se hará uso del archivo `fields_ignition/launch/stereo_sim_bag.launch`. 

Para la ejecución de este archivo se tienen una serie de parámetros con sus respectivos valores por defecto.

#### Parámetros del archivo `.launch` y sus valores por defecto:

- **`bag_file_path`**: 
  - **Descripción**: Ruta del archivo `.bag` que contiene los datos generados por el simulador.
  - **Valor por defecto**: `/home/user/catkin_ws/bag_simulado.bag`
  
- **`folder_path`**: 
  - **Descripción**: Carpeta de trabajo donde se encuentran los archivos de configuración y donde se guardarán los resultados del procesamiento. Es la ruta a la carpeta `catkin_ws`.
  - **Valor por defecto**: `/home/user/catkin_ws`

- **`post_procesamiento`**: 
  - **Descripción**: Indica si se debe realizar o no post-procesamiento en los datos obtenidos. El post-procesamiento incluye la **detección**, **trackeo**, **filtrado** y **conteo de las manzanas**.
  - **Valor por defecto**: `true`

- **`config`**: 
  - **Descripción**: Ruta al archivo de configuración (`config.ini`) utilizado para el procesamiento posterior, que define ciertos parámetros relacionados con el procesamiento de los datos.
  - **Valor por defecto**: `$(folder_path)/src/apple_fields_ignition/fields_ignition/nodes/config.ini`

- **`bag_playback_speed`**: 
  - **Descripción**: Velocidad de reproducción del archivo `rosbag` durante la ejecución. Este parámetro controla qué tan rápido o lento se reproduce el archivo de datos. El valor adecuado dependerá del hardware. Se creó con la intención de no perder datos durante la lectura del `bag`.
  - **Valor por defecto**: `0.05`

#### Ejemplo de ejecución

```bash
roslaunch fields_ignition stereo_sim_bag.launch \
bag_file_path:=/home/usuario/catkin_ws/datos_simulacion.bag \
folder_path:=/home/usuario/catkin_ws \
post_procesamiento:=true \
config:=/home/usuario/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes/mi_config.ini \
bag_playback_speed:=0.1
```

---

### 2 - Post procesado de los datos

Para llevar a cabo el post procesamiento de los datos, se debe ejecutar el archivo `fields_ignition/nodes/detectors_and_trackers/track_and_filter.py`. Este script se encarga de realizar las siguientes tareas:

- **Detección y Trackeo**: Identifica las manzanas en las imágenes procesadas y sigue su movimiento a través de los cuadros de video.
- **Filtrado**: Elimina las manzanas que no deberían ser contadas en determinado, frame, por ejemplo aquellas que estén demasiado lejos.
- **Conteo**: Calcula la cantidad total de manzanas detectadas y ademas guarda los resultados.

Los resultados de cada ejecución se guardan en un archivo `.ini` en la carpeta `results` dentro de `carkin`. Si la carpeta no existe, se crea automáticamente. El archivo de resultados contiene el contenido del archivo de configuración utilizado, seguido de los resultados del conteo de manzanas.

El formato del archivo `.ini` es el siguiente:

- Se incluye el contenido del archivo de configuración original.
- Se añaden tres secciones al final del archivo:
  - `[RESULTS]`: Contiene el conteo de manzanas sin ajustes.
  - `[RESULTS_WITH_REGRESSION]`: Contiene el conteo ajustado usando un modelo de regresión.
  - `[RESULTS_WITH_COEFFICIENT]`: Contiene el conteo ajustado por un coeficiente.

---

El archivo lee un archivo de configuración en formato `.ini`, el cual describe cómo procesar los datos.

Cuando se ejecute el archivo `track_and_filter.py`, se deben pasar dos parámetros:

- **`--config`**: Este parámetro es obligatorio y debe especificar la ruta al archivo de configuración en formato `.ini`, que describe cómo procesar los datos.
  
- **`--bag_name`**: Este parámetro es opcional y tiene un valor por defecto de "unknown". Se puede usar para nombrar el archivo de datos que se está procesando, lo que puede ser útil para la organización y el seguimiento de los resultados.

#### Archivo de configuracion

Dentro del archivo de configuracion cuyo path se pasa como parametro, irán definidas los siguientes parametros:

Los parámetros del archivo de configuración son los siguientes:

- **`image_height`**: Altura de las imagenes generadas por las cámaras en píxeles. Se utiliza para definir las dimensiones de las imágenes que se procesan.
  
- **`image_width`**: Ancho de las imagenes generadas por las cámaras en píxeles. Junto con `image_height`, establece el tamaño de las imágenes de entrada.

- **`count_threshold`**: Umbral de conteo. Este valor se utiliza para determinar cuántas detecciones de una manzana particular deben estar presentes para considerarla para el conteo final.

- **`tracking_method`**: Método de seguimiento. Define el algoritmo que se usará para el seguimiento de las manzanas a través de los frames.

- **`yolo_weights`**: Nombre del archivo de modelo YOLO (You Only Look Once) utilizados para la detección de objetos. Esto es esencial para el rendimiento del modelo de detección. Es importante que este archivo se encuentre en el directorio `/home/user/catkin_ws/weights`

- **`track`**: Bandera booleana que indica si se debe realizar el seguimiento de las manzanas. Si se establece en `true`, se activará el seguimiento. ⚠️ **Atención**: El tracking agrega un overhead importante, mientras se está experimentando con algún bag, no es necesario ejecutar el tracking en cada ejecución, luego de la primera con este parametro en `true`, se lo puede apagar, los datos de tracking quedarán guardados. En caso de que se ejecute con esta bandera en `true` nuevamente, los datos ya guardados del tracking anterior serán eliminados.

- **`gen_imagenes_tracker`**: Bandera booleana que indica si se deben generar imágenes del seguimiento. Esto es útil para la visualización y el análisis.

- **`generar_imagen_plano`**: Bandera booleana que indica si, al utilizar el filtrado por plano, se deben generar imágenes que representen el plano. Esto es útil para visualizar el proceso de filtrado.

- **`rotar_imagenes`**: Bandera booleana que indica si las imágenes deben ser rotadas para alinearse correctamente. En el caso por ejemplo, de que la grabación de haya realizado con la camara en vertical.

- **`verbose`**: Bandera booleana que, cuando se establece en `true`, habilita la salida de algunos logs para el seguimiento del proceso.

- **`debug_plano`**: Bandera booleana que indica si se deben generar imágenes con los puntos utilizados para realizar la interpolación del plano. En estas imágenes, los puntos que aparezcan en negro corresponden a troncos que fueron descartados por estar demasiado lejos y no se utilizaron para generar el plano. Esto es útil para depurar y visualizar el proceso de creación del plano.

- **`method`**: Indica el tipo de filtrado que se utilizará en el procesamiento de los datos. Los valores posibles para este parámetro son:
  - `kmeans`: Utiliza un algoritmo basado en **K-Means** para realizar el filtrado.
  - `plano`: Aplica un **filtrado por plano**.
  - `sin_filtrado`: No se aplica ningún tipo de filtrado.
  - `filas_posteriores`: Filtra unicamente aquellas manzanas pertencientes a **filas posteriores**.
  - `punto_medio`: Realiza un filtrado basado en el **punto medio**.

- **`coeficiente_de_ajuste`**: Es un coeficiente que se utiliza para multiplicar el resultado final del conteo de manzanas. Este coeficiente es generado a partir de cálculos matemáticos para ajustar el resultado del conteo.

- **`modelo_de_regresion`**: En lugar de utilizar el `coeficiente_de_ajuste`, este parámetro emplea una recta obtenida a partir de una regresión lineal para ajustar el conteo final de manzanas.

- **`modelo_tronco`**: Nombre del achivo de modelo YOLO utilizado para detectar los troncos en las imágenes. El modelo se carga desde un archivo `.pt` que está ubicado en la carpeta de pesos (`weights`).

- **`offset_horizontal`**: Valor entero que se utiliza para **ignorar una franja sin datos de profundidad** que aparece en el lado izquierdo de la imagen.

- **`margen_plano`**: Parámetro que **mueve el plano hacia adelante o hacia atrás** para ajustarlo al centro del árbol. En esencia, permite refinar la posición del plano si no se está calculando correctamente en el centro de los troncos. Se recomienda utilizar un valor de `0` a menos que se detecte que el plano no está alineado con el centro de los troncos, en cuyo caso este valor puede ajustarse.

- **`threshold_margin`**: Parámetro utilizado en el filtrado K-Means. Este margen permite ajustar la sensibilidad del filtro, ayudand a ajustar qué tan cerca deben estar los puntos para ser considerados para el conteo final. Es análogo al `margen_plano`, pero específico para los métodos de filtrado K-Means y punto medio.

- **`FILTRADO_PUNTO_MEDIO.distancia_filtro`**: Analogo al anterior pero para el filtrado de punto medio.

- **`FILTRADO_FILAS_POSTERIORES.distancia_filtro`**: Parámetro que define la **distancia máxima** permitida para los puntos a ser considerados válidos durante el filtrado en el filtrado de filas posteriores. Este parametro deberia ser configurado con una distancia mas lejanas que todas las manzanas de la fila inmediata al robot pero mas cercana a todas las manzanas de las filas posteriores.

## Archivos auxiliares

Dentro del directorio `fields_ignition/nodes/detectors_and_trackers` hemos construido algunos archivos para ayudar a la depuración y a la generación de resultados.

- **`plot_filtered_apples.py`**: 
  El archivo `plot_filtered_apples.py` utiliza el filtro configurado en el archivo de configuración para generar copias de los frames, donde las manzanas que quedan fuera del conteo están señalizadas con un punto rojo y las que quedan dentro, con un punto verde.

  El parámetro --config es requerido para ejecutar el script, y debe contener la ruta al archivo de configuración.

- **`run_experiments.py`**:
  Este archivo tiene como objetivo correr experimentos de forma automática. Ejecuta el archivo `track_and_filter.py` para diferentes archivos de configuración definidos en `fields_ignition/nodes/detectors_and_trackers/experiments_configs`. El archivo `track_and_filter` se encarga de guardar los resultados correspondientes a cada experimento lanzado con un nombre adecuado y un timestamp.

- **`calculate_coefficient_and_LR.py`**
  tiene como objetivo procesar los resultados de los experimentos guardados previamente. Realiza dos tareas principales:

  1. **Calcular coeficientes de ajuste**: Se calculan coeficientes para cada tipo de cámara y método de filtrado.
  2. **Ajuste por regresión lineal**: Se generan rectas y gráficos para ajustar los conteos utilizando regresión lineal para cada tipo de cámara y método de filtrado.

  Esto es realizado a partir de los resultados de los experimentos. En nuestro caso, los experimentos consistieron de corridas en 20 mundos para cada combinación de filtro/cámara.

  El parámetro `--results_path` es requerido y debe apuntar a la carpeta donde se encuentran los archivos de resultados. Los coeficientes y modelos de regresión generados se crearán en el directorio especificado también por este parámetro.


  