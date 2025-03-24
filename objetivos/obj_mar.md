# Mes 2, marzo

## Semana 5, 3/03/25 - 9/03/25

- [x] Crear rama en el repositorio del neurobot
- [x] Leer y probar el código de ros2_ws
- [x] Definir pasos a seguir

## Conceptos aprendidos S5

1. Crear los siguientes nodos de ROS2:
* Nodo de monitoreo y control: recibir datos de los sensores y ajustar los actuadores en base a las necesidades del paciente.
* Nodo de interfaz gráfica para ajustar los parámetros.
* Nodo de simulación de movimientos: perturbaciones tipo paso, trayectoria y mantenimiento de la posición.

2. Comunicación entre nodos:
* Suscriptores: para leer datos de los sensores.
* Publicadores: publicar datos en la interfaz gráfica.

3. Desarrollo de la interfaz gráfica:
* Python: PyQt

4. Implementación del protocolo gaming.
5. Validación y pruebas.

## Semana 6, 10/03/25 - 16/03/25

- [x] Hacer pruebas con el código de la rama de Pablo.
- [x] Actualizar repo.

## Conceptos aprendidos S6

El nodo principal del workspace se ejecuta de la siguiente forma:
```bash
ros2 run main main_node
```

Este nodo recive por la entrada estándar el tipo de función (sinuidal o pulso) y sus parámetros (frecuencia, amplitud y fase).

## Semana 7, 17/03/25 - 23/03/25

- [ ] Redactar introcucción memoria.
- [ ] Leer artículos médicos.

## Conceptos aprendidos S7



## Semana 8, 24/03/25 - 30/03/25

- [x] Crear nodo para generar las señales de trayectoria.
- [ ] Leer artículos médicos.

## Conceptos aprendidos S8

He usado la librería *matplotlib* de python y he creado un script de python que crea un suscriptor al topic `/PositionReference` para recibir los datos de frecuencia y amplitud que se publican en formato `msg.data`.

```txt
Frecuencia: eje X
Amplitud: eje Y
```

> [!NOTE]
> He fijado un tamaño mínimo y máximo de ventana para que sea estática, pero no se muestran los datos actualizados de la señal una vez llega al tamaño máximo de la ventana x.
