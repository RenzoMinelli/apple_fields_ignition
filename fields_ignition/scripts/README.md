# Conteo de frutos de pepita utilizando Deep Learning Y Visión de profundidad

## Instalación

# TODO EL SETUP DE CATKIN Y ROS Y ESAS MRDS SACAR DE LO QUE HICIERON LOS PIBES
# CLONAR EL REPO DE GITLAB (CUANDO LO HAYAMOS SUBIDO)

## Utilización

Este pipeline procesa archivos .bag, por lo que para ejecutarlo simplemente se necesitará un archivo con dicha estensión.

### 1 - Extraccion de imagenes RGB y datos de profundidad del bag.

### Recomendación

Se **recomienda leer todo este README** antes de intentar ejecutar el proyecto. Esto te ayudará a entender los requisitos, la configuración adecuada y los pasos necesarios para evitar errores durante la ejecución.

#### Caso de bag con datos reales (no simulados)

Se hará uso del archivo fields_ignition/launch/stereo_real_bag.launch. 
Este archivo .launch ejecuta el flujo completo para reproducir datos grabados de cámaras estéreo desde un rosbag, remapear los tópicos de imágenes y cámara info, procesar las imágenes para generar mapas de disparidad usando stereo_image_proc, y finalmente, guardar los resultados en formato de imagen.

Para la ejecucion de este archivo se tienen una serie de parametros con sus respectivos valores por defecto
### Parámetros del archivo `.launch` y sus valores por defecto:

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