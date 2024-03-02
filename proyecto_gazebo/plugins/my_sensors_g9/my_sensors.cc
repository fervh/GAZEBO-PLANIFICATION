#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/sensors/RaySensor.hh>

namespace gazebo
{

    class MySensors : public SensorPlugin
    {
    public:
        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
        {
            if (!_sensor)
            {
                gzerr << "the plugin requires a LaserSensor.\n";
                return;
            }
            // Store the pointer to the model
            sensor = _sensor;

            sensor->SetActive(true);

            gzdbg << "Opened " << sensor->ScopedName() << "\n";

            raySensor = std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
            if (!raySensor)
            {
                gzerr << "dynamic_pointer_cast to RaySensor failed!\n";
                return;
            }
            // this block of parameters is obtained from gazebo
            gzdbg << "AngleMax [deg] " << raySensor->AngleMax().Degree() << "\n";
            gzdbg << "AngleMin [deg] " << raySensor->AngleMin().Degree() << "\n";
            gzdbg << "RangeMax [m] " << raySensor->RangeMax() << "\n";
            gzdbg << "RangeMin [m] " << raySensor->RangeMin() << "\n";
            gzdbg << "AngleResolution [deg] " << raySensor->AngleResolution() * 180.0 / M_PI << "\n";
            gzdbg << "RangeCount " << raySensor->RangeCount() << "\n";
            gzdbg << "UpdateRate " << raySensor->UpdateRate() << "\n";

            // Listen to the update event. This event is broadcast every simulation iteration.
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MySensors::OnUpdate, this));
        }

        // Called by the world update start event
        void OnUpdate()
        {
            std::vector<double> ranges;
            raySensor->Ranges(ranges);
            //gzdbg << "ranges.size() " << ranges.size() << "\n"; // RangeCount
            gzdbg << std::accumulate(ranges.begin(), ranges.end(), 0.0) / ranges.size() << "\n";
        }

    private:
        sensors::RaySensorPtr raySensor;
        sensors::SensorPtr sensor;             // Pointer to the sensor
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
    };

    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(MySensors)
}
