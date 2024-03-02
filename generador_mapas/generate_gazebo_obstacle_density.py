"""
Autor: Fernando Vela Hidalgo (https://github.com/fervh)
Asignatura: Simuladores de Robots
Universidad: Universidad Carlos III de Madrid (UC3M)
Fecha: Febrero 2024

Generador de laberintos para Gazebo.
"""

from lxml import etree
from numpy import genfromtxt
import argparse
import random
import csv
from numpy import genfromtxt

# VARIABLES
altura_caja_píxel = 1.0
resolution = 1  # Just to make similar to MATLAB [pixel/meter]
metro_por_píxel = 1 / resolution  # [meter/pixel]

# Función para generar un laberinto con las dimensiones basadas en el nombre y apellido dados.
def generate_maze(name, surname, obstacle_density, multiplication):
    rows = len(name) * multiplication - 2
    cols = len(surname) * multiplication - 2
    maze = []
    maze.append([1] * (cols + 2))
    for _ in range(rows):
        row = [1]
        for _ in range(cols):
            if random.random() < obstacle_density:
                row.append(1)  # Obstacle
            else:
                row.append(0)  # Empty space
        row.append(1)
        maze.append(row)
    maze.append([1] * (cols + 2))
    return maze

# Función para guardar el laberinto en un archivo CSV.
def save_maze_to_csv(maze, filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for row in maze:
            writer.writerow(row)


def generar_mapa_gazebo(archivo_entrada):
    # This function is made by: Author: Juan G Victores
    # CopyPolicy: released under the terms of the LGPLv2.1
    # URL: <https://github.com/roboticslab-uc3m/webots-tools>

    #-- Cargar datos del archivo CSV
    inFile = genfromtxt(archivo_entrada, delimiter=',')
    print(inFile)

    nX = inFile.shape[0]
    nY = inFile.shape[1]

    Ez = altura_caja_píxel
    Ex = metro_por_píxel
    Ey = metro_por_píxel

    sdf = etree.Element("sdf", version="1.4")
    world = etree.SubElement(sdf, "world", name="default")
    light = etree.SubElement(world, "light", name="sun", type="directional")
    cast_shadows = etree.SubElement(light, "cast_shadows").text="1"
    diffuse = etree.SubElement(light, "diffuse").text="0.8 0.8 0.8 1"
    specular = etree.SubElement(light, "specular").text="0.1 0.1 0.1 1"
    attenuation = etree.SubElement(light, "attenuation")
    _range = etree.SubElement(attenuation, "range").text="1000"
    constant = etree.SubElement(attenuation, "constant").text="0.9"
    linear = etree.SubElement(attenuation, "linear").text="0.01"
    quadratic = etree.SubElement(attenuation, "quadratic").text="0.001"
    direction = etree.SubElement(light, "direction").text="-0.5 0.5 -1"

    #-- Create Floor
    floorEx = Ex * nX
    floorEy = Ey * nY
    floorEz = altura_caja_píxel / 8.0  # arbitrary

    model = etree.SubElement(world, "model", name="floor")
    pose = etree.SubElement(model, "pose").text=str(floorEx/2.0)+" "+str(floorEy/2.0)+" "+str(-floorEz/2.0)+" 0 0 0"
    static = etree.SubElement(model, "static").text="true"
    link = etree.SubElement(model, "link", name="link")
    collision = etree.SubElement(link, "collision", name="collision")
    geometry = etree.SubElement(collision, "geometry")
    box = etree.SubElement(geometry, "box")
    size = etree.SubElement(box, "size").text=str(floorEx)+" "+ str(floorEy)+" "+str(floorEz)
    visual = etree.SubElement(link, "visual", name="visual")
    geometry = etree.SubElement(visual, "geometry")
    box = etree.SubElement(geometry, "box")
    size = etree.SubElement(box, "size").text=str(floorEx)+" "+ str(floorEy)+" "+str(floorEz)

    #-- Create Walls
    for iX in range(nX):
        for iY in range(nY):
            #-- Skip box if map indicates a 0
            if inFile[iX][iY] == 0:
                continue

            x = Ex/2.0 + iX*metro_por_píxel
            y = Ey/2.0 + iY*metro_por_píxel
            z = Ez/2.0

            model = etree.SubElement(world, "model", name="box_"+str(iX)+"_"+str(iY))
            pose = etree.SubElement(model, "pose").text=str(x)+" "+str(y)+" "+str(z)+" 0 0 0"
            static = etree.SubElement(model, "static").text="true"
            link = etree.SubElement(model, "link", name="link")
            collision = etree.SubElement(link, "collision", name="collision")
            geometry = etree.SubElement(collision, "geometry")
            box = etree.SubElement(geometry, "box")
            size = etree.SubElement(box, "size").text=str(Ex)+" "+ str(Ey)+" "+str(Ez)
            visual = etree.SubElement(link, "visual", name="visual")
            geometry = etree.SubElement(visual, "geometry")
            box = etree.SubElement(geometry, "box")
            size = etree.SubElement(box, "size").text=str(Ex)+" "+ str(Ey)+" "+str(Ez)

    myStr = etree.tostring(sdf, pretty_print=True, encoding="unicode")

    # Escribir el archivo de salida
    name = archivo_entrada.split('.')[0]
    archivo_salida = open(name + '.xml', 'w')
    archivo_salida.write(myStr)
    archivo_salida.close()



# Función principal del programa.
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=str, required=True, help="Nombre del archivo CSV del laberinto")
    parser.add_argument("--name", type=str, required=True, help="Tu nombre para determinar las dimensiones del laberinto")
    parser.add_argument("--surname", type=str, required=True, help="Tu apellido para determinar las dimensiones del laberinto")
    parser.add_argument("--obstacle-density", type=float, default=0.2, help="Densidad de obstáculos en el laberinto (valor entre 0 y 1)")
    parser.add_argument("--multiplication", type=int, default=1, help="Factor de multiplicación para el laberinto")
    args = parser.parse_args()
    if not 0 <= args.obstacle_density <= 1:
        print("Error: La densidad de obstáculos debe estar entre 0 y 1.")
        return
    maze = generate_maze(args.name, args.surname, args.obstacle_density, args.multiplication)
    save_maze_to_csv(maze, args.map)

    print(f"Laberinto generado con dimensiones: {len(maze)}x{len(maze[0])}")
    generar_mapa_gazebo(args.map)

if __name__ == "__main__":
    main()


