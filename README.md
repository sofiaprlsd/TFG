# TFG
Trabajo Fin de Grado

# NeuroBot

El NeuroBot es un brazo robótico destinado a la rehabilitación de un paciente tras sufrir un ictus.

Mi TFG consiste en desarrollar una ramificación de estrategia para la plataforma robótica y diseñar una interfaz de usuario para poder adaptar los ejercicios de la terapia a cada paciente. Todo debe desarrollarse en un entorno simulado. Para comprobar el correcto funcionamiento del software de monitoreo aplicaré un protocolo de prueba gaming.

Las actividades para la terapia que se ofrecen son:
* Perturbación tipo paso
* Mantener la posición
* Trayectoria de posición

## Requisitos

Para el desarrollo del trabajo utilizaré ROS2 y Python.

### ROS2-humble

Los suscriptores se suscriben a los sensores para recibir los datos que están siendo publicados y los publicadores publican los datos obtenidos de los sensores en una interfaz gráfica y modifican la intensidad de los actuadores para ajustar el movimiento a la necesidad del paciente.

### Python

Visualización en tiempo real del movimiento del paciente.

Es necesario que se permita ajustar manualmente los siguientes datos:
* Rango de posición
* Frecuencia
* Duración
* Ciclos de la función sinusoide
* Fuerza máxima y mínima de las perturbaciones

Cálculo del error del paciente durante el juego.

Interfaz para el médico para ingresar los datos del paciente.
