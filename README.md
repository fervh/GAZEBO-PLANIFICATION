# Proyecto de Planificación de Rutas en GAZEBO

Este proyecto incluye herramientas para generar mapas tipo laberinto y convertirlos a un formato compatible con el entorno de simulación Gazebo. Y contine distintos algoritmos para la planificación de rutas en Gazebo.

**Autor: Fernando Vela Hidalgo** (GitHub: [github.com/fervh](https://github.com/fervh))
Este proyecto es parte de la asignatura "Simuladores de Robots" en la Universidad Carlos III de Madrid (UC3M).

## Generador de Mapas

El generador de mapas funciona mediante la creación de un laberinto a partir del nombre y apellido proporcionados, junto con una densidad de obstáculos especificada.

### Uso
Para generar un laberinto, ejecuta el script `generate_wbt_obstacle_density.py` con los siguientes parámetros:

- `--map`: Nombre del archivo CSV para guardar el laberinto.
- `--name`: Tu nombre para determinar las dimensiones del laberinto.
- `--surname`: Tu apellido para determinar las dimensiones del laberinto.
- `--obstacle-density`: Densidad de obstáculos en el laberinto (valor entre 0 y 1).
- `--multiplication`: Factor de multiplicación para las dimensiones del laberinto.

Ejemplo:
```bash
python generate_wbt_obstacle_density.py --map map1.csv --name Fernando --surname Vela --obstacle-density 0.3 --multiplication 5
```

El script crea un archivo de mundo Gazebo (.xml) que representa el laberinto. Cada obstáculo en el laberinto se convierte en un objeto de caja sólida en el mundo Gazebo.

##
##
## Algoritmos:
##

## Algoritmo A*
Este algoritmo permite que un robot en Gazebo se mueva hacia una meta utilizando una ruta óptima calculada con el algoritmo A*.

[astar_gazebo.webm](https://github.com/fervh/GAZEBO-PLANIFICATION/assets/55854056/445b4b23-e17b-43ac-8b31-c49b433a8688)

