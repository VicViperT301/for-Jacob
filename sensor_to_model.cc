#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <iostream>

namespace gazebo
{
  int count = 0;
  double fx = 0;
  double fy = 0;
  double fz = 0;
  double vx = 0;
  double vy = 0;
  double vz = 0;
  double theta[3] = {0, 0, 0};
  double point1[3] = {0, 0, 0};
  double point2[3] = {0, 0, 0};
  double point3[3] = {0, 0, 0};
  double point4[3] = {0, 0, 0};
  gazebo::msgs::Contacts contacts;
  gazebo::math::Pose pose;
  gazebo::math::Vector3 position;
  gazebo::math::Vector3 velocity;
  gazebo::math::Quaternion rotation;
  class SensorRead : public ModelPlugin
  {
    transport::SubscriberPtr mySubscriber;
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      std::cout << "Plugin loaded" << std::endl;
      std::string topicName = "~/box/link/my_contact/contacts";

        transport::NodePtr node(new transport::Node());
        node->Init("default");
      
      this->mySubscriber = node->Subscribe(topicName, &SensorRead::OnUpdate, this);
    }

    // Called by the world update start event
    public: void OnUpdate(ConstContactsPtr &_contacts)
    {
       std::cout << "An update!" << std::endl;
       std::cout << "" << _contacts->DebugString();
       pose = this->model->GetWorldPose();
       velocity = this->model->GetWorldLinearVel();
       vx = velocity.x;
       position.z = pose.pos.z;
       rotation = pose.rot;
       theta[0] = rotation.GetRoll();
       theta[1] = rotation.GetPitch();
       theta[2] = rotation.GetYaw();
       
       if(position.z > 0.9)
          count = 1;
       if(count == 1)
       {
       if(!(vx > 0 && vx < 0.05) || (vx < 0 && vx > -0.05)) 
       {
       if(position.z - 0.25 < 0.00009)
       {
         fx = _contacts->contact(0).wrench(0).body_1_wrench().force().x(); 
         fy = _contacts->contact(0).wrench(0).body_1_wrench().force().y();
         fz = _contacts->contact(0).wrench(0).body_1_wrench().force().z();
       this->model->GetLink("link")->AddRelativeForce(gazebo::math::Vector3(0, -0.3*fz, 0));
       }
       
       printf("force:%lf, %lf, %lf", fx, fy, fz);
       }
       }
    }

    // Pointer to the model
    private: physics::ModelPtr model;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SensorRead)
}
