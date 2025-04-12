## Semana 9, 31/03/25 - 6/04/25

- [x] Arreglar publicador del topic */PositionReference*.
- [x] Crear juego a partir de 2 señales para gaming.
- [x] Crear *umlet* con los nodos, scripts, publicadores, suscriptores y topics.

## Conceptos aprendidos S9

En vez de publicar los datos de la señal en el topic `/PsitionReference` a través de *main_node*, publico desde el script de python *scroller_publisher* en un nuevo topic `/LimitReference` para que no haya conflicto entre dos publicadores y así actualizo los valores desde una ventana gráfica con dos sliders, uno para la frecuencia y otro para la amplitud de la señal.

### Graficar una señala partir de los datos publicados en el topic */PsitionReference*

En este topic se publican mensajes de tipo **Float32** y el formato es el siguiente:

```cpp
if (is_pulse_) {
    message.data = (std::fmod(2 * M_PI * frequency_ * time_, 2 * M_PI) < M_PI) ? amplitude_ : -amplitude_;
} else {
    message.data = amplitude_ * std::sin(2 * M_PI * frequency_ * time_);
}
```

Para ejecutar el programa:

```bash
cd <workspace>
ros2 run neurobot_graphics main_node
python3 signal_plotter.py
```

Desde la terminal del nodo *main_node* se debe especificar el tipo de señal `{SINWAVE, PULSE}`, la frecuencia `F0.5`, y la amplitud `A1.0`.

Resultado:

![Captura desde 2025-03-29 22-41-07](https://github.com/user-attachments/assets/7d477878-6cb9-42c3-9801-b4625356d572)

### Juego Flappy Bird

Este juego es un pájaro (posición del brazo del paciente) que tiene que pasar entre dos tuberías (señal sinuidal desplazada verticalmente hacia arriba y abajo por un offset, esto es lo mismo que dos señales idénticas pero una positiva y otra negativa) sin colisionar con ellas.

Para crear las señales me creo un publicador y un topic llamado `/LimitReference` de tipo **Float32** y formato similar al de *main_node*:

```py
msg.data = self.ampl * math.sin(2 * math.pi * self.freq * self.time)
```

> [!NOTE]
> Para graficar dos señales idénticas pero desplazadas en el eje Y, desplazo verticalmente la señal sumando/restando un *offset*.

Para ejecutar el programa:

```bash
cd <ruta_a_la_carpeta_scrpits>
python3 scroll_publisher.py
python3 flappy_bird.py
```

Escenario:

![Captura desde 2025-03-28 16-10-03](https://github.com/user-attachments/assets/a534299f-db26-4426-97dc-0008eef75df9)

Añadimos el jugador (punto rojo):

![Captura desde 2025-03-29 22-44-30](https://github.com/user-attachments/assets/843cc189-1569-4d75-a366-5d7a35c2fc27)

> [!NOTE]
> De momento, para simular el movimiento del jugador uso las flechas hacia arriba y abajo del teclado.

Además, a medida que pasa el tiempo se va reduciendo el offset de las señales para que se acerquen cada vez más y se complique el juego, es decir, subes de nivel. Si hay colisión el juego se queda en el último nivel alcanzado hasta que la persona corrija su posición y esté dentro de los límites.

![Captura desde 2025-03-29 22-45-40](https://github.com/user-attachments/assets/d20a7e3b-2cde-485e-bdaf-d92229873404)

### Conexiones entre nodos

![Captura desde 2025-03-29 23-10-01](https://github.com/user-attachments/assets/084b40d6-8d05-479c-917c-0455d794503f)

> [!IMPORTANT]
> Cambios:
>
> scroll_publisher.py publica un `Float32MultiArray` con la frecuencia y la amplitud en el topic `/SliderParameters` para que main_node.cpp cree una señal y la publique en el topic `/PositionReference` al que se suscriben tanto signal_plotter.py como flappy_bird.py.

El esquema de todo el proyecto con lo que llevamos todos los distribuidores queda de la siguiente forma:

![Captura desde 2025-04-03 17-15-33](https://github.com/user-attachments/assets/2c49e528-dd20-47f9-93b9-76410a1b0961)


## Semana 10, 7/04/25 - 13/04/25

- [x] Crear script para visualizar los límites, la posición del paciente y el error.
- [x] Crear interfaz para ingresar los datos del paciente y volcarlos a un fichero.
- [x] Añadir inicialización de las variables del juego desde un fichero.

## Conceptos aprendidos S10

He creado una interfaz gráfica con la librería [Tkinter](https://docs.python.org/es/3.13/library/tkinter.html) en python (database.py) para ingresar los datos de un paciente y crear un fichero .csv si no existe y actualizar el fichero si existe. Los ficheros se guardan bajo el nombre del NIF del paciente en un directorio llamado `~/database` en `$HOME`. Los datos que se pueden ingresar son:
* Nombre (string)
* Apellido (string)
* NIF (int)
* Parámetros de juego (frecuencia y amplitud) (int)
* Progreso (%) (int)
* Nota del médico (string)

> [!IMPORTANT]
> Es obligatorio rellenar el campo NIF, el resto son opcionales.

![Captura desde 2025-04-12 22-12-58](https://github.com/user-attachments/assets/1541114d-b5dc-4102-94e4-c3c39b079aac)


Y ahora si se pasa el path de un fichero que exista, los valores de frecuencia y amplitud se cargan del .csv:

```bash
$ python3 scroll_publisher.py ~/database/0123345.csv 
[INFO] [1744487924.049367243] [scroll_publisher_node]: Publishing F0.30 A2.00
```

Si los campos de frecuencia y amplitud no están rellenos o el fichero no existe o no se pasa ningún path, se inicializan a frecuencia=0.5 y amplitud=1.0:

```bash
$ python3 scroll_publisher.py ~/database/4455566.csv 
Error reading file could not convert string to float: ''
[INFO] [1744488042.942339401] [scroll_publisher_node]: Publishing F0.50 A1.00
```

```bash
$ python3 scroll_publisher.py ./4455566.csv 
File './4455566.csv' does not exist
[INFO] [1744488105.984249440] [scroll_publisher_node]: Publishing F0.50 A1.00
```

```bash
$ python3 scroll_publisher.py
[INFO] [1744488696.220746427] [scroll_publisher_node]: Publishing F0.50 A1.00
```

También he creado un nuevo script (flappy_bird_viewer.py) para que el médico pueda ver la progresión y error del paciente durante el juego.

Para ello he tenido que modificar el juego (flappy_bird.py) para crear un publicador de tipo `Float32MultiArray` que publique en un nuevo topic `/PlayerPosition` los datos de:
* Posición X del paciente
* Posición Y del paciente
* Offset de la señal
* Tiempo

Mi nuevo script se suscribe tanto a este topic como a `/PositionReference` para poder graficar las señales. Además, muestra una pantalla en negro indicando que está esperando al jugador si no se detecta ningún dato en el topic `/PlayerPosition`.

![Captura desde 2025-04-05 14-14-09](https://github.com/user-attachments/assets/f252d854-1376-4f41-88f4-5fdc8c8a59b3)


### Video de funcionamiento

[FlappyBirdViewer.webm](https://github.com/user-attachments/assets/4971ca4e-7b7a-42b2-81ed-abfee1537a5b)


### Conexiones entre nodos

![Captura desde 2025-04-05 15-31-54](https://github.com/user-attachments/assets/976b0373-5a0e-43b5-b1b5-8d314cd8aeb3)


## Semana 11, 14/04/25 - 20/04/25

*[SEMANA SANTA]*

## Semana 12, 21/04/25 - 27/04/25

- [ ] Añadir imágenes al juego (tuberías y pájaro).
- [ ] Avanzar con la memoria (objetivos).

## Conceptos aprendidos S12
