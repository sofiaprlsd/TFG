# Mes 4, mayo

## Semana 13, 28/04/25 - 4/05/25

- [x] Incluir mejoras propuestas por el equipo durante las reuniones del V25 y X30 (mirar S12 en abril).
- [ ] Hacer la introducción de la memoria.

## Conceptos aprendidos S13

Después de incluir lo siguiente:

**flappy_bird.py**
- Añadir la señal de la trayectoría deseada (línea continua).

**flappy_bird_viewer.py**
- Mostrar la misma pantalla que ve el jugador.
- Mostrar una pantalla con el error de la trayectoria del paciente.
- Volcar los datos de la terapia a un .csv para su posterior análisis en un nuevo directorio dentro de `~/database/ID/metrics`, *ID-fecha-metrics_index.csv*.

La pantalla de la derecha es lo que ve el paciente y la de la izquierda la del médico:

![Captura desde 2025-05-01 19-06-20](https://github.com/user-attachments/assets/d107398b-cacf-4f1a-912a-f8391263e598)


> [!IMPORTANT]
> Si no se especifica el fichero con los datos del paciente da error.

**scroll_publisher**
- Permitir la modificación de los parámetros de la perturbación (fuerza, duración, periodo entre perturbaciones).
- Caja para elegir el tipo de señal y perturbación (sinuidal o escalón) de forma independiente.
- Crear un slider para definir el offset de la señal principal.
- Volcar los updates de los parámetros de la terapia a un .csv para su posterior análisis en un nuevo directorio dentro de `~/database/ID/config`, *ID-fecha-config_index.csv*.

![Captura desde 2025-05-02 13-34-22](https://github.com/user-attachments/assets/ba85b81c-984d-42fc-9a08-5b8937fde873)


> [!IMPORTANT]
> Si no se especifica el fichero con los datos del paciente da error.

**database.py**
- Añadir al .csv los datos de la perturbación y el nivel.
- Cambiar el nombre y directorio en el que se guarda el .csv, ID.csv.

Nuevos campos añadidos:

![Captura desde 2025-05-01 18-41-56](https://github.com/user-attachments/assets/be9a6298-6f76-4997-ba7e-7aef582e7f2e)


Estructura del directirio en `$HOME`:
```
~/database
|-- {ID}/
|---- {ID}.csv
|---- config/
|------ {ID}-{date}-config_{index}.csv
|---- metrics/
|------ {ID}-{date}-metrics_{index}.csv
```

## Semana 14, 5/05/25 - 11/05/25

- [x] Incluir mejoras propuestas por el equipo durante las reuniones del V25 y X30 (mirar S12 en abril).
- [x] Resolver las correcciones de la memoria.

## Conceptos aprendidos S14



## Semana 15, 12/05/25 - 15/05/25

- [ ] Añadir la perturbación al juego.
- [x] Crear misiones para cada nivel y hacer el juego más mono.
- [ ] Hacer esquema con la información a incluir en cada capítulo de la memoria.

## Conceptos aprendidos S15

Misiones de los niveles del juego (flappy_bird.py):
* N1. Permanecer dentro de los límites 20 segundos.
* N2. Recolectar 3 estrellas.
* N3. ?
* N4. Permanecer dentro de los límites 40 segundos.
* N5. ?
* N6. Recolectar 5 estrellas.
* N7. Permanecer dentro de los límites 60 segundos.
* N8. ?
* N9. Gravedad invertida (arriba es abajo y abajo es arriba).
* N10. Control del offset.
