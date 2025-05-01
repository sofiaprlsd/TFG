# TFG
Trabajo Fin de Grado

# NeuroBot

El NeuroBot es un brazo robótico destinado a la rehabilitación de un paciente tras sufrir un ictus.

Mi TFG consiste en desarrollar una ramificación de estrategia para la plataforma robótica y diseñar una interfaz de usuario para poder adaptar los ejercicios de la terapia a cada paciente. Todo debe desarrollarse en un entorno simulado. Para comprobar el correcto funcionamiento del software de monitoreo aplicaré un protocolo de prueba gaming.

Las actividades para la terapia que se ofrecen son:
* Perturbación tipo paso
* Mantener la posición
* Trayectoria de posición

Para el desarrollo del trabajo utilizaré ROS2 y Python.

**ROS2-humble**

Los suscriptores se suscriben a los sensores para recibir los datos que están siendo publicados y los publicadores publican los datos obtenidos de los sensores en una interfaz gráfica y modifican la intensidad de los actuadores para ajustar el movimiento a la necesidad del paciente.

**Python**

Visualización en tiempo real del movimiento del paciente.

Es necesario que se permita ajustar manualmente los siguientes datos:
* Rango de posición
* Frecuencia
* Duración
* Ciclos de la función sinusoide
* Fuerza máxima y mínima de las perturbaciones

Cálculo del error del paciente durante el juego.

Interfaz para el médico para ingresar los datos del paciente.

Configuración para ejecutar los scripts del proyecto Flappy Bird
================================================================

Este proyecto contiene scripts en Python que dependen de ROS 2 Humble y algunas librerías externas.

Pasos para ejecutar correctamente los scripts en otro ordenador:

## 1. REQUISITOS PREVIOS

Tener instalado:
  - Python 3.8 o superior
  - ROS 2 Humble Hawksbill (con entorno correctamente configurado)
  - pip (gestor de paquetes de Python)

## 2. INSTALAR DEPENDENCIAS DE PYTHON

Ejecutar los siguientes comandos en una terminal:
```bash
sudo apt update
sudo apt install python3-tk
pip3 install -r requirements.txt
```

*Esto instalará las siguientes librerías necesarias:*
- matplotlib
- numpy

## 3. ESTRUCTURA DE DIRECTORIOS

El script crea automáticamente la carpeta ~/database para almacenar los datos de los pacientes y las configuraciones.

## 4. USO DE LOS SCRIPTS

**1. Crear o registrar paciente:**

Ejecutar el script `patient_database.py` (interfaz gráfica) para crear un nuevo paciente y registrar su configuración inicial.

**2. Lanzar la GUI para jugar y controlar los parámetros:**

Ejecutar `scroll_publisher.py`, pasando como argumento el archivo CSV del paciente recién creado.

Ejemplo:
```bash
python3 scroll_publisher.py ~/database/0001/0001.csv
```

**3. Visualizar el juego (viewer):**

Ejecutar `flappy_bird_viewer.py`, pasando también el mismo CSV como argumento.

Ejemplo:
```bash
python3 flappy_bird_viewer.py ~/database/0001/0001.csv
```

> [!IMPORTANT]
> Para los puntos 2 y 3, si no se pasa el argumento CSV o se pasa incorrectamente, el script mostrará un error y terminará.

**4. Juego (player):**

Ejecutar `flappy_bird.py`.

Ejemplo:
```bash
python3 flappy_bird.py
```

## 5. EJECUCIÓN

No olvides dar permisos de ejecución a los scripts si lo necesitas:
```bash
chmod +x scroll_publisher.py
chmod +x flappy_bird_viewer.py
chmod +x patient_database.py
```

Para ejecutarlos:
```bash
python3 scroll_publisher.py ~/ruta/al/paciente.csv
```

## 6. TRAZAS

Si tienes problemas, asegúrate de que ROS2 esté correctamente inicializado en tu terminal:
```bash
source /opt/ros/humble/setup.bash
```

> [!NOTE]
> Revisa los logs para entender lo que se hace con cada acción.
