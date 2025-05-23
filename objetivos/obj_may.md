# Mes 4, mayo

## Semana 13, 28/04/25 - 4/05/25

- [x] Incluir mejoras propuestas por el equipo durante las reuniones del V25 y X30 (mirar S12 en abril).
- [x] Hacer el capítulo 4 de la memoria.

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



## Semana 15, 12/05/25 - 18/05/25

- [x] Crear misiones para cada nivel.
- [x] Añadir la perturbación al juego.
- [x] Añadir elemento visual para indicar la perturbación
- [x] Hacer esquema con la información a incluir en cada capítulo de la memoria.

## Conceptos aprendidos S15

**flappy_bird.py**

Objetivo principal: Seguir la trayectoria

Misiones:
* N1. Permanecer dentro de los límites 20 segundos.
![Captura desde 2025-05-17 17-27-03](https://github.com/user-attachments/assets/8b2fdcd1-884e-4fca-9f7a-fb00e02dfffd)

* N2. Recolectar 3 estrellas.
![Captura desde 2025-05-17 17-19-43](https://github.com/user-attachments/assets/c003d421-82d6-44e4-a67e-c301ad64ea35)

* N3. Esquivar 4 asteroides.
![Captura desde 2025-05-17 17-19-59](https://github.com/user-attachments/assets/bcb02742-6a2d-4de0-9dc2-e56dfb90a577)

* N4. Permanecer dentro de los límites 40 segundos.
![Captura desde 2025-05-17 17-20-23](https://github.com/user-attachments/assets/d8a79eda-2476-46c0-9af8-d8e04bada8e9)

* N5. Gravedad invertida (arriba es abajo y abajo es arriba) durante 10 segundos.
![Captura desde 2025-05-17 17-21-38](https://github.com/user-attachments/assets/4968f536-7ad3-4eb3-ba7a-faad2176f7ff)

* N6. Recolectar 5 estrellas.
![Captura desde 2025-05-17 17-22-08](https://github.com/user-attachments/assets/4c6f09e9-25c4-4795-89aa-38756220c8b0)

* N7. Permanecer dentro de los límites 60 segundos.
![Captura desde 2025-05-17 17-22-48](https://github.com/user-attachments/assets/8d7be9a4-3a24-4e5c-afa5-24d8c2305fa7)

* N8. Esquivar 6 asteroides.
![Captura desde 2025-05-17 17-25-26](https://github.com/user-attachments/assets/04ea349f-2a8d-4b47-a42c-0149bdd62d48)

* N9. Gravedad invertida (arriba es abajo y abajo es arriba) durante 15 segundos.
![Captura desde 2025-05-17 17-26-04](https://github.com/user-attachments/assets/c2756784-182e-4adc-b46b-40c7ebb01aa4)

* N10. Juego libre (ej. control con distintos offsets).
![Captura desde 2025-05-17 17-26-30](https://github.com/user-attachments/assets/f0a58955-e31f-4c1d-be13-6a1cc2718b32)


Recompensas:
* Cada estrella recolectada suma 10 puntos.
* Cada asteroide esquivado suma 10 puntos.
* Cada vez que se completa una misión se suman 10 puntos.

Penalizaciones:
* Colisionar con un asteroide resta 5 puntos.

Elementos visuales (perturbación):

He añadido un triángulo naranja que indica el momento en el que comienza la perturbación.

Elementos auditivos:

He añadido sonidos cuando se choca con un asteroide, cuando se consigue una estrella y cuando se sube de nivel.

### Esquema de nodos

![Captura desde 2025-05-23 13-13-17](https://github.com/user-attachments/assets/f7e2ce62-37b2-4041-8e3f-24ef35c41f88)

