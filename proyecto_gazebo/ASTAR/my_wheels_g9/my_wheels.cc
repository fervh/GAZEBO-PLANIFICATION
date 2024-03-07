/*
Autor: Fernando Vela Hidalgo (https://github.com/fervh)
Asignatura: Simuladores de Robots
Universidad: Universidad Carlos III de Madrid (UC3M)
Fecha: Marzo 2024

Algoritmo A* para el robot en GAZEBO. El robot se mueve a la meta con una ruta óptima.

PROS:
- Garantiza la optimización de la ruta.
- Garantiza que el robot alcance la meta.

CONS:
- Requiere un mapa del entorno.
*/

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <filesystem>

// Librerias para ASTAR
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <numeric>
#include <cmath>

// Función para convertir radianes a grados
double RadianesAGrados(double radianes) {
    return radianes * 180.0 / M_PI;
}

// Definición de acciones posibles (moverse hacia arriba, abajo, izquierda, derecha)
const int actions[][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};

// Definición de la función heurística (distancia de Manhattan)
int heuristic(std::pair<int, int> a, std::pair<int, int> b)
{
    return abs(a.first - b.first) + abs(a.second - b.second);
}

// Definición de la función A* para encontrar el camino óptimo
std::vector<std::pair<int, int>> astar(std::vector<std::vector<int>> &grid, std::pair<int, int> start, std::pair<int, int> goal)
{
    // Definición de la lista abierta y cerrada
    std::vector<std::vector<int>> closed(grid.size(), std::vector<int>(grid[0].size(), 0));
    std::vector<std::vector<int>> open(grid.size(), std::vector<int>(grid[0].size(), 0));

    // Definición de la lista de nodos padre
    std::vector<std::vector<std::pair<int, int>>> came_from(grid.size(), std::vector<std::pair<int, int>>(grid[0].size()));

    // Definición de la lista de costes
    std::vector<std::vector<int>> g(grid.size(), std::vector<int>(grid[0].size(), INT_MAX));

    // Definición de la lista de costes acumulados
    g[start.first][start.second] = 0;

    // Definición de la lista de costes acumulados + heurística
    std::vector<std::vector<int>> f(grid.size(), std::vector<int>(grid[0].size(), INT_MAX));

    // Definición de la lista de nodos abiertos
    std::vector<std::pair<int, int>> open_list;

    // Añadir el nodo inicial a la lista abierta
    open_list.push_back(start);
    open[start.first][start.second] = 1;

    // Mientras la lista abierta no esté vacía
    while (!open_list.empty())
    {
        // Obtener el nodo con el menor coste acumulado + heurística
        std::pair<int, int> current = open_list[0];
        int index = 0;
        for (int i = 0; i < open_list.size(); i++)
        {
            if (f[open_list[i].first][open_list[i].second] < f[current.first][current.second])
            {
                current = open_list[i];
                index = i;
            }
        }

        // Si el nodo actual es el nodo objetivo, reconstruir el camino y devolverlo
        if (current == goal)
        {
            std::vector<std::pair<int,
                                    int>> path;
            while (current != start)
            {
                path.push_back(current);
                current = came_from[current.first][current.second];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Eliminar el nodo actual de la lista abierta y añadirlo a la lista cerrada
        open_list.erase(open_list.begin() + index);
        open[current.first][current.second] = 0;
        closed[current.first][current.second] = 1;

        // Para cada vecino del nodo actual
        for (int i = 0; i < 4; i++)
        {
            int x = current.first + actions[i][0];
            int y = current.second + actions[i][1];

            // Si el vecino está fuera del mapa, continuar
            if (x < 0 || x >= grid.size() || y < 0 || y >= grid[0].size())
            {
                continue;
            }

            // Si el vecino es un obstáculo, continuar
            if (grid[x][y] == 1)
            {
                continue;
            }

            // Si el vecino está en la lista cerrada, continuar
            if (closed[x][y] == 1)
            {
                continue;
            }

            // Calcular el coste acumulado del vecino
            int tentative_g = g[current.first][current.second] + 1;

            // Si el vecino no está en la lista abierta o el coste acumulado es menor que el anterior
            if (tentative_g < g[x][y])
            {
                // Actualizar el coste acumulado del vecino
                g[x][y] = tentative_g;

                // Actualizar el coste acumulado + heurística del vecino
                f[x][y] = g[x][y] + heuristic(std::make_pair(x, y), goal);

                // Añadir el vecino a la lista abierta si no está
                if (open[x][y] == 0)
                {
                    open_list.push_back(std::make_pair(x, y));
                    open[x][y] = 1;
                }

                // Actualizar el nodo padre del vecino
                came_from[x][y] = current;
            }
        }
    }

    // Si no se ha encontrado un camino, devolver un vector vacío
    return std::vector<std::pair<int, int>>();
}


namespace gazebo
{

    class MySensorsModel : public ModelPlugin
    {
    public:
        // Usa el camino para mover el robot a lo largo del camino hacia la meta usando solo la posición y la rotación (yaw)
        void MoveRobotAlongPath(const ignition::math::Pose3d& pose)
        {
            // Comprueba si quedan puntos de referencia en el camino
            if (!path.empty())
            {
                // Obtiene el punto de referencia actual del camino
                std::pair<int, int> nextWaypoint = path.front();

                // Calcula la dirección hacia el siguiente punto de referencia
                double dx = nextWaypoint.first + 0.5 - pose.Pos().X();
                double dy = nextWaypoint.second + 0.5 - pose.Pos().Y();

                // Comprueba si la posición actual está dentro de los límites de la celda del siguiente punto de referencia
                if (pose.Pos().X() >= nextWaypoint.first + 0.2 && pose.Pos().X() < nextWaypoint.first + 1 &&
                    pose.Pos().Y() >= nextWaypoint.second + 0.2 && pose.Pos().Y() < nextWaypoint.second + 1)
                {
                    // Alcanzó el punto de referencia, elimínelo del camino
                    std::cout << "Reached the waypoint (" << nextWaypoint.first << ", " << nextWaypoint.second << ")" << std::endl;
                    path.erase(path.begin());
                    // Detiene la velocidad lineal y angular del robot
                    model->SetLinearVel(ignition::math::Vector3d::Zero);
                    model->SetAngularVel(ignition::math::Vector3d::Zero);
                    return;
                }

                // Calcula el ángulo a girar (yaw) en radianes
                double targetYawRad = atan2(dy, dx);
                double currentYawRad = pose.Rot().Yaw();

                // Convierte radianes a grados
                double targetYawDeg = RadianesAGrados(targetYawRad);
                double currentYawDeg = RadianesAGrados(currentYawRad);

                // Calcula la diferencia entre el yaw actual y el objetivo en grados
                double deltaAngleDeg = targetYawDeg - currentYawDeg;

                // Ajusta el ángulo para que esté dentro del rango [-180°, 180°]
                while (deltaAngleDeg > 180.0)
                    deltaAngleDeg -= 360.0;
                while (deltaAngleDeg < -180.0)
                    deltaAngleDeg += 360.0;

                // Determina el múltiplo más cercano de 90 grados
                double targetAngleDeg = round(currentYawDeg / 90.0) * 90.0;

                // Gira el robot hacia el múltiplo más cercano de 90 grados
                if (fabs(deltaAngleDeg) > 1.0)
                {
                    std::cout << "Turning towards the nearest multiple of 90 degrees..." << std::endl;
                    std::cout << "Current pose: x=" << pose.Pos().X() << ", y=" << pose.Pos().Y() << ", yaw=" << currentYawDeg << "°" << std::endl;
                    std::cout << "Desired pose: x=" << nextWaypoint.second + 0.5 << ", y=" << nextWaypoint.first + 0.5 << ", yaw=" << targetYawDeg << "°" << std::endl;
                    ignition::math::Vector3d angularVelocity(0, 0, deltaAngleDeg > 0 ? 0.5 : -0.5);
                    model->SetAngularVel(angularVelocity);
                }
                else
                {
                    std::cout << "Moving forward towards the next waypoint..." << std::endl;
                    std::cout << "Current pose: x=" << pose.Pos().X() << ", y=" << pose.Pos().Y() << ", yaw=" << currentYawDeg << "°" << std::endl;
                    std::cout << "Desired pose: x=" << nextWaypoint.second + 0.5 << ", y=" << nextWaypoint.first + 0.5 << ", yaw=" << targetYawDeg << "°" << std::endl;
                    ignition::math::Vector3d velocity(0.5, 0, 0);
                    velocity = pose.Rot().RotateVector(velocity);
                    model->SetLinearVel(velocity);
                }
            }
        }



        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            int sensor_num = _model->GetSensorCount();
            
            sensor = sensors::get_sensor("my_sensor");

            if (!sensor)
            {
                gzerr << "the plugin requires a LaserSensor.\n";
                return;
            }
            
            sensor->SetActive(true);

            gzdbg << "Opened " << sensor->ScopedName() << "\n";

            raySensor = std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
            if (!raySensor)
            {
                gzerr << "dynamic_pointer_cast to RaySensor failed!\n";
                return;
            }

            // Estos parámetros se obtienen de Gazebo
            gzdbg << "AngleMax [deg] " << raySensor->AngleMax().Degree() << "\n";
            gzdbg << "AngleMin [deg] " << raySensor->AngleMin().Degree() << "\n";
            gzdbg << "RangeMax [m] " << raySensor->RangeMax() << "\n";
            gzdbg << "RangeMin [m] " << raySensor->RangeMin() << "\n";
            gzdbg << "AngleResolution [deg] " << raySensor->AngleResolution() * 180.0 / M_PI << "\n";
            gzdbg << "RangeCount " << raySensor->RangeCount() << "\n";
            gzdbg << "UpdateRate " << raySensor->UpdateRate() << "\n";

            // Obtiene el puntero al modelo
            model = _model;

            // Escucha el evento de actualización. Este evento se transmite en cada iteración de la simulación.
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MySensorsModel::OnUpdate, this));

            // Construye la ruta absoluta a map.csv
            std::string currentDirectory = std::filesystem::current_path().string();
            std::string mapFilePath = currentDirectory + "/map/map.csv";

            // Imprime la ruta para depurar
            gzdbg << "Path to map.csv: " << mapFilePath << std::endl;

            // Abre el archivo map.csv
            std::ifstream mapFile(mapFilePath);
            if (!mapFile.is_open())
            {
                gzerr << "Failed to open map.csv file\n";
                return;
            }

            // Lee el archivo map.csv y guárdalo en un vector 2d e imprímelo
            std::vector<std::vector<int>> map;
            std::string line;
            while (std::getline(mapFile, line))
            {
                std::vector<int> row;
                std::stringstream ss(line);
                std::string cell;
                while (std::getline(ss, cell, ','))
                {
                    row.push_back(std::stoi(cell));
                }
                map.push_back(row);
            }
            for (int i = 0; i < map.size(); i++)
            {
                for (int j = 0; j < map[i].size(); j++)
                {
                    std::cout << map[i][j] << " ";
                }
                std::cout << std::endl;
            }

            // Define las posiciones de inicio y objetivo (usa las esquinas del mapa) (definir como variables de clase)
            std::pair<int, int> start = std::make_pair(1, 1);
            std::pair<int, int> goal = std::make_pair(map.size() - 2, map[0].size() - 2);

            
            // Imprime las posiciones de inicio y objetivo en el mapa (usa una copia del mapa) e imprímelo usando # para 1, espacio para 0 y S y G para inicio y objetivo
            std::vector<std::vector<int>> mapCopy = map;
            mapCopy[start.first][start.second] = 2;
            mapCopy[goal.first][goal.second] = 3;
            for (int i = 0; i < mapCopy.size(); i++)
            {
                for (int j = 0; j < mapCopy[i].size(); j++)
                {
                    if (mapCopy[i][j] == 0)
                    {
                        std::cout << "   ";
                    }
                    else if (mapCopy[i][j] == 1)
                    {
                        std::cout << " # ";
                    }
                    else if (mapCopy[i][j] == 2)
                    {
                        std::cout << " S ";
                    }
                    else if (mapCopy[i][j] == 3)
                    {
                        std::cout << " G ";
                    }
                }
                std::cout << std::endl;
            }

            std::cout << "Start: (" << start.first << ", " << start.second << ")" << std::endl;
            std::cout << "Goal: (" << goal.first << ", " << goal.second << ")" << std::endl;



            // Encuentra el camino óptimo usando A*
            path = astar(map, start, goal);

            // Imprime el camino usando X para el camino y el mapa anterior impreso 
            for (int i = 0; i < path.size(); i++)
            {
                mapCopy[path[i].first][path[i].second] = 4;
            }

            for (int i = 0; i < mapCopy.size(); i++)
            {
                for (int j = 0; j < mapCopy[i].size(); j++)
                {
                    if (mapCopy[i][j] == 0)
                    {
                        std::cout << "   ";
                    }
                    else if (mapCopy[i][j] == 1)
                    {
                        std::cout << " # ";
                    }
                    else if (mapCopy[i][j] == 2)
                    {
                        std::cout << " S ";
                    }
                    else if (mapCopy[i][j] == 3)
                    {
                        std::cout << " G ";
                    }
                    else if (mapCopy[i][j] == 4)
                    {
                        std::cout << " X ";
                    }
                }
                std::cout << std::endl;
            }

            // Imprime el camino
            std::cout << "Path: ";
            for (int i = 0; i < path.size(); i++)
            {
                std::cout << "(" << path[i].first << ", " << path[i].second << ")";
                if (i < path.size() - 1)
                {
                    std::cout << " -> ";
                }
            }
            std::cout << std::endl;

        }


        void OnUpdate()
        {
            std::vector<double> ranges;
            raySensor->Ranges(ranges);
            //gzdbg << "ranges.size() " << ranges.size() << "\n"; // RangeCount
            //gzdbg << std::accumulate(ranges.begin(), ranges.end(), 0.0) / ranges.size() << "\n";

            // Comprueba si el vector de rangos está vacío, devuelve si lo está
            if (ranges.empty()) {
                return;
            }

            // Filtra los NaN y valores inusuales
            ranges.erase(std::remove_if(ranges.begin(), ranges.end(), [](double d) {
                return std::isnan(d) || d < 0.1 || d > 30.0;
            }), ranges.end());


            // Print the averages for specific angles
            /*
            printf("Right: %.2f\n", ranges[ranges.size()/8]);
            printf("Front Right: %.2f\n", ranges[ranges.size()/4]);
            printf("Front: %.2f\n", ranges[ranges.size()/2]);
            printf("Front Left: %.2f\n", ranges[ranges.size()*3/4]);
            printf("Left: %.2f\n", ranges[ranges.size()*7/8]);
            */
            
            

            // Obtiene la pose del modelo
            ignition::math::Pose3d pose = model->WorldPose();

            // Usa el camino para mover el robot a lo largo del camino hacia la meta usando solo la posición y la rotación (yaw)
            MoveRobotAlongPath(pose);


        }

    private:
        sensors::RaySensorPtr raySensor;
        sensors::SensorPtr sensor;             // Pointer to the sensor
        physics::ModelPtr model;               // Pointer to the model
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
        std::vector<std::pair<int, int>> path;  // Camino óptimo
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MySensorsModel)
}

