/*
Autor: Fernando Vela Hidalgo (https://github.com/fervh)
Asignatura: Simuladores de Robots
Universidad: Universidad Carlos III de Madrid (UC3M)
Fecha: Marzo 2024

Algoritmo Wall Follower para el robot MyWheelsG9


PROS:
- Simple
- No requiere conocimiento previo del entorno

CONS:
- No es el camino más corto (no es óptimo)

*/

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <filesystem>


namespace gazebo
{

    class MySensorsModel : public ModelPlugin
    {
    public:
        
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

            // Obtiene los punteros a las articulaciones
            if (!_sdf->HasElement("left_joint"))
                gzerr << "MyWheels plugin missing <left_joint> element\n";

            if (!_sdf->HasElement("right_joint"))
                gzerr << "MyWheels plugin missing <right_joint> element\n";

            leftJoint = _model->GetJoint(_sdf->GetElement("left_joint")->Get<std::string>());
            rightJoint = _model->GetJoint(_sdf->GetElement("right_joint")->Get<std::string>());

            if (!leftJoint)
                gzerr << "Unable to find left joint[" << _sdf->GetElement("left_joint")->Get<std::string>() << "]\n";
            if (!rightJoint)
                gzerr << "Unable to find right joint[" << _sdf->GetElement("right_joint")->Get<std::string>() << "]\n";


            // Escucha el evento de actualización. Este evento se transmite en cada iteración de la simulación.
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MySensorsModel::OnUpdate, this));

           

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
            std::cout << "Right: " << ranges[ranges.size()/8] << std::endl;
            std::cout << "Front Right: " << ranges[ranges.size()/4] << std::endl;
            std::cout << "Front: " << ranges[ranges.size()/2] << std::endl;
            std::cout << "Front Left: " << ranges[ranges.size()*3/4] << std::endl;
            std::cout << "Left: " << ranges[ranges.size()*7/8] << std::endl;

            

            // Obtiene la pose del modelo
            ignition::math::Pose3d pose = model->WorldPose();

            // Imprime la posición del modelo
            std::cout << "At: " << pose.Pos().X() << " " << pose.Pos().Y() << " " << pose.Pos().Z() << std::endl;

            // Imprime el ángulo de rotación del modelo pose.Rot().Yaw();
            std::cout << "Angle: " << pose.Rot().Yaw() << std::endl;

            double velocity = 10.0;
            // Si el sensor frontal detecta un objeto a menos de 0.5 metro, gira a la izquierda
            if (ranges[ranges.size()/2] < 0.5 || ranges[ranges.size()/8] < 0.3) {
                    leftJoint->SetVelocity(0, -velocity);
                    rightJoint->SetVelocity(0, velocity);
                }
            // si no
            else{
                // SI la pared a la derecha no está muy lejos sigue hacia delante
                if (ranges[ranges.size()/8] < 0.5) {
                        leftJoint->SetVelocity(0, velocity);
                        rightJoint->SetVelocity(0, velocity);  
                }
                else{
                    // Si la pared a la derecha está lejos, gira a la derecha
                        leftJoint->SetVelocity(0, velocity);
                        rightJoint->SetVelocity(0, velocity/3);  
                }

            }

            // Si su pose coincide con el punto final, se detiene
            if (pose.Pos().X() > 14 && pose.Pos().Y() > 6) {
                model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
                model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));

                std::cout << "Llegó al final" << std::endl;
                exit(0);
            }


        }

    private:
        sensors::RaySensorPtr raySensor;
        sensors::SensorPtr sensor;             // Pointer to the sensor
        physics::ModelPtr model;               // Pointer to the model
        physics::JointPtr leftJoint, rightJoint; // Pointer to the joints
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
        std::vector<std::pair<int, int>> path;  // Camino óptimo
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MySensorsModel)
}

