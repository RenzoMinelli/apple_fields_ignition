# Generación de campos de manzanas

Los campos de manzanas se generan ejecutando `field_generator.py`.

## Forma de uso

```comand
comando:
        python3 scripts/field_generator.py
            --world_name {WorldName}
            --out_path {OUTPUT_PATH}
            --field_id {FIELD_IDENTIFIER}
            --row_count {ROW_COUNT}
            --row_length {ROW_LENGTH}
            --row_dist {ROW_DIST}
            --crop_dist {CROP_DIST}
            --origin_coordinates {ORIGIN_COORDINATES}
            --tree_properties {BASE_64_TREE_PROPERTIES}
            --camera_position {CAMERA_POSITION}
            --husky_initial_position {HUSKY_INITIAL_POSITION}

parámetros:
    --world_name                    Nombre del mundo
    --out_path                      Directorio de destino
    --field_id                      Identificador de la instancia
    --row_count                     Número de filas de árboles.
    --row_length                    Cantidad de árboles por fila.
    --row_dist                      Distancia entre las filas.
    --crop_dist                     Distancia entre árboles de la fila.
    --origin_coordinates            Coordenadas del origen para el posicionamiento de los árboles.
    --tree_properties               Propiedades del árbol en Base64.
    --camera_position               Posición de la cámara.
    --husky_initial_position        Posición inicial del robot.

```

## Estructura de archivos

Cada campo de manzanas generado tiene una estructura específica generada dinámicamente en función de los parámetros con los que se genera.

Se utiliza la herramienta [Cookiecutter](https://cookiecutter.readthedocs.io/en/stable/) para generar dicha estructura en base a los templates que se encuentran en el directorio `templates`.

Cada instancia generada tiene la siguiente estructura de archivos:

```
.
└── apple_field
    ├── apple_{id}
    │   ├── markers.json
    │   ├── materials
    │   │   └── textures
    │   │       ├── apple_2.jpg
    │   │       ├── apple_3.jpg
    │   │       ├── apple.jpg
    │   │       ├── brn1.png
    │   │       └── leaf.png
    │   ├── meshes
    │   │   └── apple.dae
    │   ├── model.config
    │   └── model.sdf
    ├── ground
    │   ├── meshes
    │   │   ├── ...
    │   ├── model.config
    │   └── model.sdf
    ├── husky
    │   ├── ...
    ├── apple_field.sdf
    ├── markers.json
    ├── tree_properties.json
    └── world_params.json
```

Los directorios `apple_{id}` incluyen el modelo de los árboles y el archivo SDF para incluirlos en Gazebo.

El directorio `ground` incluye el sdf y el modelo del suelo.

El directorio `husky` incluye el modelo del robot y el sdf modificado para utilizar la cámara con parámetros similares a la ZED2 (no estereo).

El archivo `apple_field.sdf` incluye los modelos definidos previamente.

El archivo `markers.json` es un json que define la posición de los árboles en el mundo.

El archivo `world_params.json` incluye los parámetros que se le pasaron al script para generar la instancia.

El archivo `tree_properties.json` incluye los parámetros específicos para los árboles, según el patrón definido en [Generación de Árboles](./blender/README.md).
