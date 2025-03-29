## Semana 9, 31/03/25 - 6/04/25

- [x] Arreglar publicador del topic */PositionReference*.
- [x] Crear posibilidad de representar 1 o 2 señales para gaming.
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


## Semana 10, 7/04/25 - 13/04/25

- [ ] Crear interfaz para ingresar los datos del paciente.
- [ ] Avanzar con la memoria (objetivos).

## Conceptos aprendidos S10
