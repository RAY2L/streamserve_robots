#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class RandomMeasurementPublisher : public ModelPlugin
  {
    private: transport::NodePtr node;
    private: transport::PublisherPtr pub;
    private: event::ConnectionPtr updateConnection;

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      this->pub = this->node->Advertise<gazebo::msgs::Float32>("~/streamserve/robot_measurement");

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RandomMeasurementPublisher::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      float measurement = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
      gazebo::msgs::Float32 msg;
      msg.set_data(measurement);
      this->pub->Publish(msg);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(RandomMeasurementPublisher)
}
