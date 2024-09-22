# Conteo de frutos de pepita utilizando Deep Learning Y Visión de profundidad

## Instalación

# TODO EL SETUP DE CATKIN Y ROS Y ESAS MRDS SACAR DE LO QUE HICIERON LOS PIBES
# CLONAR EL REPO DE GITLAB (CUANDO LO HAYAMOS SUBIDO)

## Utilización

Este pipeline procesa archivos .bag, por lo que para ejecutarlo simplemente se necesitará un archivo con dicha estensión.

### Recomendación

Se **recomienda leer todo este README** antes de intentar ejecutar el proyecto. Esto te ayudará a entender los requisitos, la configuración adecuada y los pasos necesarios para evitar errores durante la ejecución.

### Remapeo de Tópicos

Al utilizar este archivo `.launch`, es posible que necesites remapear los tópicos de tu archivo `rosbag` para que coincidan con los tópicos esperados por los nodos en el sistema. A continuación se describen los pasos para hacerlo:

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
     <arg name="left_camera_image_topic" default="/mi_camera/izquierda/image_raw"/>
     <arg name="right_camera_image_topic" default="/mi_camera/derecha/image_raw"/>
     <arg name="left_camera_info_topic" default="/mi_camera/izquierda/camera_info"/>
     <arg name="right_camera_info_topic" default="/mi_camera/derecha/camera_info"/>
     ```

4. **Ejecuta el Archivo `.launch`**:
   - Una vez que hayas actualizado los tópicos en el archivo `.launch`, ejecuta el archivo con el comando `roslaunch`, asegurándote de utilizar los parámetros adecuados. Seguir leyendo para saber cuales son.

### Ejemplo de Remapeo

Si tus tópicos son diferentes, por ejemplo:
- Izquierda: `/mi_camera/izquierda/image_raw`
- Derecha: `/mi_camera/derecha/image_raw`
- Información de la cámara izquierda: `/mi_camera/izquierda/camera_info`
- Información de la cámara derecha: `/mi_camera/derecha/camera_info`

Tu archivo `.launch` se vería así:

```xml
<arg name="left_camera_image_topic" default="/mi_camera/izquierda/image_raw"/>
<arg name="right_camera_image_topic" default="/mi_camera/derecha/image_raw"/>
<arg name="left_camera_info_topic" default="/mi_camera/izquierda/camera_info"/>
<arg name="right_camera_info_topic" default="/mi_camera/derecha/camera_info"/>
```

## Extraccion y procesamiento de de datos RGB y profundidad del bag.

### Caso de bag con datos reales (no simulados)

Se hará uso del archivo `fields_ignition/launch/stereo_real_bag.launch`. 
Este archivo ejecuta el flujo completo para reproducir datos grabados de cámaras estéreo desde un rosbag, remapear los tópicos de imágenes y cámara info, procesar las imágenes para generar mapas de disparidad usando stereo_image_proc, y finalmente, guardar los resultados en formato de imagen.

Para la ejecucion de este archivo se tienen una serie de parametros con sus respectivos valores por defecto
#### Parámetros del archivo `.launch` y sus valores por defecto:

- **`bag_file_path`**: 
  - **Descripción**: Ruta del archivo `.bag` que contiene los datos grabados de ROS (rosbag).
  - **Valor por defecto**: `/home/user/catkin_ws/bag_mas_lento.bag`
  
- **`folder_path`**: 
  - **Descripción**: Carpeta de trabajo donde se encuentran los archivos de configuración y donde se guardarán los resultados del procesamiento. Es la ruta a la carpeta catkin_ws.
  - **Valor por defecto**: `/home/user/catkin_ws`

- **`post_procesamiento`**: 
  - **Descripción**: Indica si se debe realizar o no post-procesamiento en las imágenes obtenidas además de simplemente guardarlas en disco. El post-procesamiento incluye la **detección**, **trackeo**, **filtrado** y **conteo de las manzanas**.
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
post_procesamiento:=false \
config:=/home/usuario/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes/mi_config.ini \
bag_playback_speed:=0.1
```

### Caso de bag con datos simulados

Se hará uso del archivo `fields_ignition/launch/stereo_sim_bag.launch`. Este archivo ejecuta el flujo completo para reproducir datos generados por un simulador de cámaras estéreo desde un rosbag, remapear los tópicos de imágenes y cámara info, procesar las imágenes para generar mapas de disparidad usando `stereo_image_proc`, y finalmente, guardar los resultados en formato de imagen.

Para la ejecución de este archivo se tienen una serie de parámetros con sus respectivos valores por defecto.

#### Parámetros del archivo `.launch` y sus valores por defecto:

- **`bag_file_path`**: 
  - **Descripción**: Ruta del archivo `.bag` que contiene los datos generados por el simulador.
  - **Valor por defecto**: `/home/user/catkin_ws/bag_simulado.bag`
  
- **`folder_path`**: 
  - **Descripción**: Carpeta de trabajo donde se encuentran los archivos de configuración y donde se guardarán los resultados del procesamiento. Es la ruta a la carpeta `catkin_ws`.
  - **Valor por defecto**: `/home/user/catkin_ws`

- **`post_procesamiento`**: 
  - **Descripción**: Indica si se debe realizar o no post-procesamiento en las imágenes obtenidas, además de simplemente guardarlas en disco. El post-procesamiento incluye la **detección**, **trackeo**, **filtrado** y **conteo de las manzanas**.
  - **Valor por defecto**: `true`

- **`camera_model`**: 
  - **Descripción**: Especifica el modelo de la cámara que se está utilizando. Puede tener dos valores posibles:
    - **`depth`**: Utiliza un modelo de cámara que genera imágenes de profundidad.
    - **`stereo`**: Utiliza un modelo de cámara estéreo que requiere dos imágenes para generar mapas de disparidad.
  - **Valor por defecto**: `stereo`

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
camera_model:=stereo \
config:=/home/usuario/catkin_ws/src/apple_fields_ignition/fields_ignition/nodes/mi_config.ini \
bag_playback_speed:=0.1
```
