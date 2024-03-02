#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
    class MyWheels : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            model = _model;

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

            // Listen to the update event for the wheels
            wheelUpdateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MyWheels::OnWheelUpdate, this));

            // Get the ray sensor
            sensor = model->GetSensor("my_sensor");
            if (!sensor)
            {
                gzerr << "The plugin requires a LaserSensor.\n";
                return;
            }
            sensor->SetActive(true);

            gzdbg << "Opened " << sensor->ScopedName() << "\n";

            raySensor = std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
            if (!raySensor)
            {
                gzerr << "Dynamic_pointer_cast to RaySensor failed!\n";
                return;
            }

            // Listen to the update event for the sensor
            sensorUpdateConnection = raySensor->ConnectUpdated(std::bind(&MyWheels::OnSensorUpdate, this));
        }

        // Called by the world update start event for the wheels
        void OnWheelUpdate()
        {
            leftJoint->SetVelocity(0, 1.0);
            rightJoint->SetVelocity(0, 1.0);
            ignition::math::Pose3d pose = model->WorldPose();
            printf("At: %f %f %f\n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
        }

        // Called when the sensor is updated
        void OnSensorUpdate()
        {
            std::vector<double> ranges;
            raySensor->Ranges(ranges);
            if (ranges.size() > 0)
            {
                double averageRange = std::accumulate(ranges.begin(), ranges.end(), 0.0) / ranges.size();
                gzdbg << "Average Range: " << averageRange << "\n";
            }
        }

    private:
        physics::ModelPtr model; // Pointer to the model
        physics::JointPtr leftJoint, rightJoint;
        sensors::RaySensorPtr raySensor;
        sensors::SensorPtr sensor;             // Pointer to the sensor
        event::ConnectionPtr wheelUpdateConnection; // Connection for wheel update event
        event::ConnectionPtr sensorUpdateConnection; // Connection for sensor update event
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MyWheels)
}
