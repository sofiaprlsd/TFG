## Semana 9, 31/03/25 - 6/04/25

- [x] Arreglar publicador del topic */PositionReference*.
- [x] Crear posibilidad de representar 1 o 2 señales para gaming.
- [x] Crear *umlet* con los nodos, scripts, publicadores, suscriptores y topics.

## Conceptos aprendidos S9

En vez de publicar los datos de la señal en el topic `/PsitionReference` a través de *main_node*, publico desde el script de python *scroller_publisher* en un nuevo topic `/LimitReference` para que no haya conflicto entre dos publicadores y así actualizo los valores desde una ventana gráfica con dos sliders, uno para la frecuencia y otro para la amplitud de la señal.

### Ejecución de los scripts

```bash
cd <ruta_a_la_carpeta_scrpits>
python3 scroll_publisher.py
python3 signal_plotter.py
```

![Captura desde 2025-03-28 16-08-58](https://github.com/user-attachments/assets/02ab94e6-cd18-4e4f-bc97-e3c7f7f95844)

Para graficar dos señales idénticas pero desplazadas en el eje Y, desplazo verticalmente la señal sumando/restando un *offset*.

![Captura desde 2025-03-28 16-10-03](https://github.com/user-attachments/assets/a534299f-db26-4426-97dc-0008eef75df9)

Para seleccionar entre graficar 1 o 2 señales uso un booleano.

```py
self.dual_mode = True   # 2 signals
self.dual_mode = False  # 1 signal
```

### Conexiones entre nodos

![Captura desde 2025-03-28 17-25-06](https://github.com/user-attachments/assets/d8993eb0-9514-48e1-a372-6f6ddf657eba)


## Semana 10, 7/04/25 - 13/04/25

- [ ] Crear interfaz para ingresar los datos del paciente.
- [ ] Avanzar con la memoria (objetivos).

## Conceptos aprendidos S10
