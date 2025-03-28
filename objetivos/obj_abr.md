## Semana 9, 31/03/25 - 6/04/25

- [x] Arreglar publicador del topic /PositionReference.
- [x] Crear posibilidad de representar 1 o 2 señales para gaming.
- [ ] Crear umlet con los nodos, scripts, publicadores, suscriptores y topics.

## Conceptos aprendidos S9

En vez de publicar los datos de la señal a través de *main_node*, publico desde el script de python *scroller_publisher* para que no haya conflicto entre dos publicadores y así actualizo los valores desde una ventana gráfica con dos sliders, uno para la frecuencia y otro para la amplitud de la señal.



Para graficar dos señales idénticas pero desplazadas en el eje Y, desplazo verticalmente la señal sumando/restando un *offset*.



Para seleccionar entre graficar 1 o 2 señales uso un booleano.

```py
self.dual_mode = True   # 2 signals
self.dual_mode = False  # 1 signal
```

## Semana 10, 7/04/25 - 13/04/25

- [ ] Crear interfaz para ingresar los datos del paciente.
- [ ] Avanzar con la memoria (objetivos)

## Conceptos aprendidos S9
