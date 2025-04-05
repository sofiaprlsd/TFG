# Instrucciones para la ejecución de los programas

> [!NOTE]
> Los scripts de python están en el directorio `/scripts`.


## Publicador de los datos de frecuencia y amplitud

El script de python scroll_publisher.py publica los datos de frecuencia y amplitud, que se actualizan con los sliders, en el topic `/SliderParameters` en formato `Float32MultiArray`.

Para ejecutar el programa:

```bash
cd ros2_ws/main/scripts
python3 scroll_publisher.py
```

Saldrá una ventana emergente con los siguientes elementos:
* 1 slider para ejecutar la frecuencia
* 1 slider para ejecutar la amplitud
* 1 botón para actualizar los datos
* 1 botón para salir del programa

![Captura desde 2025-04-03 20-41-01](https://github.com/user-attachments/assets/d229970c-69f2-4cd1-b8d0-16f99c4700ae)



## Suscriptor y gráfico de una señal

El script de python signal_plotter.py grafica una señal de tipo sinuidal o pulso a partir del mensaje que publica main_node.cpp en el topic `/PositionReference`.

Para ejecutar el programa primero se debe ejecutar el nodo del main para que haya datos en el topic y se pueda graficar la señal.

```bash
ros2 run main main_node
cd ros2_ws/main/scripts
python3 signal_plotter.py
```

![Captura desde 2025-03-24 20-38-19](https://github.com/user-attachments/assets/eb3d617d-03d8-4aae-8d49-4062595efb0e)



## Suscriptor y juego del Flappy Bird

El script de python flappy_bird.py grafica dos señales idénticas desplazadas verticalmente sumando/restando un *offset* a partir del mensaje que publica main_node.cpp en el topic `/PositionReference`.

Además, se grafica la posición del brazo de una persona y su movimiento vertical. El movimiento horizontal siempre es el mismo (x = x0 + 0.25).

Para ejecutar el programa primero se debe ejecutar el nodo del main para que haya datos en el topic y se pueda graficar la señal.

```bash
ros2 run main main_node
cd ros2_ws/main/scripts
python3 flappy_bird.py
```

![Captura desde 2025-03-29 22-44-30](https://github.com/user-attachments/assets/a3889129-d23e-4e3b-aefa-fedf086ebc88)

