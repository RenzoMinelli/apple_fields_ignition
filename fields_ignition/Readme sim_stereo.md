Para ejecutar la simulacion con las camaras stereo se debe ejecutar el siguiente comando:

roslaunch fields_ignition fields_ignition.launch

Luego seleccionar el mundo sim_stereo

Para ver imagenes de profundidad se tienen que ejecutar primero un nodo que rectifique las imagenes de la camara y luego el nodo que genere las imagenes de profundidad a partir de las imagenes rectificadas.

ROS_NAMESPACE=costar_husky_sensor_config_1 rosrun stereo_image_proc stereo_image_proc

rosrun image_view stereo_view stereo:=/costar_husky_sensor_config_1 image:=image_rect_color