#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

namespace gazebo
{
  int n;
  double a = 0;
  double c = 0;
  double m = 0.0001;
  double x = 0.1;
  double y = 0;
  double x1 = 0;
  double x2 = 0;
  double x3 = 0;
  double theta = 0;
  double sigma = 10;
  double b = 8/3;
  double r = 28;
  double fx = 0;
  double fy = 0;
  double fz = 0;
  double vx = 0;
  double vy = 0;
  double vz = 0;
  double delta_t = 0;
  int flag = 0;
  int sw = 0;
  int input = 's';
  int i = 0;
  gazebo::math::Pose pose;
  math::Vector3 v(0,0,0);
  gazebo::math::Vector3 vel;

  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));
    }

    int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

    int getch()
    {
        static struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);           // save old settings
        newt = oldt;
        newt.c_lflag &= ~(ICANON);                 // disable buffering
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings
 
        int c = getchar();  // read character (non-blocking)
 
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
        return c;
    } 

    //public: void dBodySetTorque(this, 1000, 0, 0)
    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      if(kbhit())
        {
          input = getch();
          flag = 0;
        }   
//if (kbhit()) c = getch();
      pose = this->model->GetWorldPose();
      v = pose.pos;
      vel = this->model->GetRelativeLinearVel();
      //t = this->model->GetRelativeLinearVel();
      x = v.x;
      y = v.y;
      theta = atan2(y,x);
      //fx = m * sigma * (r*x-y-x*z-sigma*y+sigma*x);
      //fy = m*r*sigma*(y-x)-m*r*x+m*y+m*x*z-m*sigma*z*(y-x)-m*x*(-b*z+x*y);
      //fz = -m*b*(-b*z+x*y)+m*sigma*(y-x)*y+m*x*(r*x-y-x*z);
      fx = -sin(x3)*(c*sin(x2)+b*cos(x1));
      fy = cos(x3)*(c*sin(x2)+b*cos(x1));
      //vx = sigma*(y-x);
      //vy = r*x-y-x*z;
      //vz = -b*z+x*y;
      vx = vel.x;
      vy = vel.y;
      vz = vel.z;
 
      //fx = m*sigma*(r*x-y-sigma*y+sigma*x);
      //fy = m*r*sigma*(y-x)-m*r*x+m*y-m*x*x*y;
      //this->model->SetLinearVel(math::Vector3(vx, vy, vz));
      //this->model->GetLink("link")->SetForce(math::Vector3(fx, fy, fz));
      	if(input == 'f')
          {
             if(flag <= 2)
               {
        		   // Apply a small linear velocity to the model.
        		   this->model->GetLink("link")->SetTorque(math::Vector3(50, 0, 0));
                           //this->model->GetLink("link")->SetTorque(math::Vector3(fx, fy, 0));
           		   flag = flag + 1;
                           x1 = x1 + (a*sin(x3)+c*cos(x2))*delta_t;
      			   x2 = x2 + (b*sin(x1)+a*cos(x3))*delta_t;
      			   x3 = x3 + (c*sin(x2)+b*cos(x1))*delta_t;
                           delta_t += 0.001;
        		}
            else
        		{
        		   this->model->GetLink("link")->SetTorque(math::Vector3(0, 0, 0));
                           x1 = x1 + (a*sin(x3)+c*cos(x2))*delta_t;
      			   x2 = x2 + (b*sin(x1)+a*cos(x3))*delta_t;
      			   x3 = x3 + (c*sin(x2)+b*cos(x1))*delta_t;
                           delta_t += 0.001;
        		}
      
          }

      if(input == 'f' && vx == 0 && vy == 0 && vz == 0)
          {
              x1 = x1;
              x2 = x2;
              x3 = x3;
              delta_t = delta_t;
          }
              
       

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
