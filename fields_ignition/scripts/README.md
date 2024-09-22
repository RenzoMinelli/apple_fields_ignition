# Conteo de frutos de pepita utilizando Deep Learning Y Visión de profundidad

## Instalación

# TODO EL SETUP DE CATKIN Y ROS Y ESAS MRDS SACAR DE LO QUE HICIERON LOS PIBES
# CLONAR EL REPO DE GITLAB (CUANDO LO HAYAMOS SUBIDO)

## Utilización

Este pipeline procesa archivos .bag, por lo que para ejecutarlo simplemente se necesitará un archivo con dicha estensión.

### 1 - Extraccion de imagenes RGB y datos de profundidad del bag.

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
  - **Descripción**: Indica si se debe realizar o no post-procesamiento en las imágenes obtenidas. Este valor se pasa como argumento al script que guarda las imágenes.
  - **Valor por defecto**: `true`

- **`config`**: 
  - **Descripción**: Ruta al archivo de configuración (`config.ini`) utilizado para el procesamiento posterior, que define ciertos parámetros relacionados con el procesamiento de las imágenes.
  - **Valor por defecto**: `$(arg folder_path)/src/apple_fields_ignition/fields_ignition/nodes/config.ini`



roslaunch fields_ignition stereo_real_bag.launch bag_file_path:='/home/user/catkin_ws/mitad_real.bag'  folder_path:=/home/pincho/catkin_ws
