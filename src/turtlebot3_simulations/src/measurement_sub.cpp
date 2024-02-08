#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class MeasurementSubscriber : public ModelPlugin
  {
    private: transport::NodePtr node;
    private: transport::SubscriberPtr sub;

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      this->sub = this->node->Subscribe("~/streamserve/robot_measurement",
          &MeasurementSubscriber::OnMsgReceived, this);
    }

    private: void OnMsgReceived(ConstFloatPtr &_msg)
    {
      std::cout << "Received measurement: " << _msg->data() << std::endl;
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(MeasurementSubscriber)
}
