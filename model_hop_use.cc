#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>


namespace gazebo
{
  int n;
  double a = 0;
  double c = 0;
  double e = 0.0342239091;
  double f = 0.1665277727;
  double g = 0.0488621273;
  double h = 0.2359305727;
  double m = 0.0001;
  double x = 0.1;
  double y = 0;
  double z = 0;
  double x1 = 0;
  double x2 = 0;
  double x3 = 0;
  double theta = 0;
  double theta_x[3] = {0, 0, 0};
  double theta_y[3] = {0, 0, 0};
  double theta_z[3] = {0, 0, 0};
  double theta_threshold = 0;
  double sigma = 10;
  double b = 8/3;
  double r = 28;
  double fx = 0;
  double fy = 0;
  double fz = 0;
  double fx1 = 0;
  double fy1 = 0;
  double fz1 = 0;
  double vx = 0;
  double vy = 0;
  double vz = 0;
  double ang_velx = 0;
  double ang_vely = 0;
  double ang_velz = 0;
  double delta_t = 0;
  double step_size = 0;
  double destination = 15;
  double difference  = 0;
  double judge  = 0;
  double gravity = 0.0098;
  double inertiax = 0.016125;
  //double force = 0;
  double torque = 0;
  double reference_torque[5] = {0, 0, 0, 0, 0};
  double integral[5] = {0, 0, 0, 0, 0};
  double kp = 0.9;
  double ki = 0.00003;
  double kd = 5000;
  int flag = 0;
  int reference_flag = 0;
  int count = 0;
  int contact_count = 0;
  long int sw = 0;
  int input = 's';
  int i = 0;
  FILE *fp1;
  FILE *fp2;
  FILE *fp3;
  char filename1[] = "linear_velocity.dat";
  char filename2[] = "position.dat";
  char filename3[] = "theta_x.dat";
  gazebo::math::Pose pose;
  gazebo::math::Pose controlled_pose;
  math::Vector3 v(0,0,0);
  gazebo::math::Vector3 vel;
  gazebo::math::Vector3 force;
  gazebo::math::Vector3 ang_vel;
  gazebo::math::Vector3 applied_torque;
  gazebo::math::Vector3 phi;
  gazebo::math::Quaternion quat;
  gazebo::physics::Contact *contacts;
  gazebo::physics::ContactManager contact_m;
  //typedef boost::shared_ptr<gazebo::physics::World> WorldPtr
  gazebo::physics::WorldPtr _world;
  
  //contact_m.Init(WorldPtr _world);
  //v0x = v0y * atan(0.3);
  class ModelPush : public ModelPlugin
  {
    transport::SubscriberPtr mySubscriber;
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      
      //std::string topicName = "~/box/link/my_contact/contacts";
      //transport::NodePtr node(new transport::Node());
      //node->Init("default");

      //this->mySubscriber = node->Subscribe(topicName, &ModelPush::affectcontacts, this);
 
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

    /*public: void affectcontacts(ConstContacsPtr &_contacts)
    {
       gazebo::math::Pose pose1;
       gazebo::math::Vector3 velocity;
       gazebo::math::Vector3 position;
       gazebo::math::Quaternion rotation;
       double theta1[3] = {0, 0, 0};
       int count1 = 0;

       std::cout << "An update!" << std::endl;
       std::cout << "" << _contacts->DebugString();
       pose1 = this->model->GetWorldPose();
       velocity = this->model->GetWorldLinearVel();
       position.z = pose1.pos.z;
       rotation = pose1.rot;
       theta1[0] = rotation.GetRoll();
       theta1[1] = rotation.GetPitch();
       theta1[2] = rotation.GetYaw();

       if(position.z > 0.9)
          count1 = 1;
       if(count1 == 1)
       {
       if(!(velocity.x > 0 && velocity.x < 0.05) || (velocity.x < 0 && velocity.x > -0.05))
       {
       if(position.z - 0.25 < 0.00009)
       {
         fx1 = _contacts->contact(0).wrench(0).body_1_wrench().force().x();
         fy1 = _contacts->contact(0).wrench(0).body_1_wrench().force().y();
         fz1 = _contacts->contact(0).wrench(0).body_1_wrench().force().z();
       this->model->GetLink("link")->AddRelativeForce(gazebo::math::Vector3(0, -0.3*fz1, 0));
       }

       printf("force:%lf, %lf, %lf", fx1, fy1, fz1);
       }
       }
}*/
    //public: void dBodySetTorque(this, 1000, 0, 0)
    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      //contacts.FillMsg(msgs::Contact &_msg);
      //fx = contacts.normals[0].x;
      //fy = contacts.normals[0].y;
      //fz = contacts.normals[0].z;

      //printf("%lf\n", fz);
      //ModelPush::affectcontacts();  
      if(kbhit())
        {
          input = getch();
          flag = 0; 
        }  

      if(sw == 0)
      {
        fp1 = fopen(filename1, "w"); 
        fp2 = fopen(filename2, "w");
        fp3 = fopen(filename3, "w");
        sw = 1;
      }

        
//if (kbhit()) c = getch();
      pose = this->model->GetLink("link")->GetWorldPose();
      v = pose.pos;
      quat = pose.rot;
      phi = quat.GetAsEuler();
      printf("Euler: %lf, %lf, %lf\n", phi.x, phi.y, phi.z);
   
      //contact_m.Init(_world);
      contact_count = contact_m.GetContactCount();
      printf("%d\n", contact_count);
      //if(contact_m.GetContact(0) != NULL) 
      //{
       //contacts = contact_m.GetContact(0);
      
      //}
      //pose.rot.x = 0;
      //pose.rot.y = 0;
      //pose.rot.z = 0;
      //pose.rot.w = 1;
      vel = this->model->GetWorldLinearVel();
      //fx = contacts.normals(0).x();
      //force = this->model->GetJoint("base_joint")->GetForce(0);
      //t = this->model->GetRelativeLinearVel();
      x = v.x;
      y = v.y;
      z = v.z;
      printf("%lf, %lf, %lf\n", x, y, z);
      printf("%lf, %lf, %lf, %lf\n", pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
      theta = atan2(y,x);
      //fx = m * sigma * (r*x-y-x*z-sigma*y+sigma*x);
      printf("Contact Force: %lf, %lf, %lf\n", fx, fy, fz);
      //fy = m*r*sigma*(y-x)-m*r*x+m*y+m*x*z-m*sigma*z*(y-x)-m*x*(-b*z+x*y);
      //fz = -m*b*(-b*z+x*y)+m*sigma*(y-x)*y+m*x*(r*x-y-x*z);
      //fx = -sin(x3)*(c*sin(x2)+b*cos(x1));
      //fy = cos(x3)*(c*sin(x2)+b*cos(x1));
      //vx = sigma*(y-x);
      //vy = r*x-y-x*z;
      //vz = -b*z+x*y;
      vx = vel.x;
      vy = vel.y;
      vz = vel.z;
      //fx = force.x;
      //fy = force.y;
      //fz = force.z;
      judge = destination + y;
      difference = abs(destination - y);
      torque = (-(e*h+f*g)+sqrt(pow(e*h+f*g, 2.0)-4*e*f*g*h+2*e*g*gravity*difference))/(2*e*g); 
       
      this->model->GetLink("link")->SetForce(math::Vector3(0, 0, 0));
      //fx = m*sigma*(r*x-y-sigma*y+sigma*x);
      //fy = m*r*sigma*(y-x)-m*r*x+m*y-m*x*x*y;
      //this->model->SetLinearVel(math::Vector3(vx, vy, vz));
      //this->model->GetLink("link")->SetForce(math::Vector3(fx, fy, fz));
      	if(input == 'f')
         {
//             if((vx> -1e-13 && vx<1e-13) && (vy>-1e-13 && vy<1e-13) && (vz>-1e-13 && vz<1e-13) && (theta_x[0] < 1e-13 && theta_x[0] > -1e-13) && (theta_y[0] < 1e-13 && theta_y[0] > -1e-13) && (theta_z[0] < 1e-13 && theta_z[0] > -1e-13) && (z < 0.0751 && z > 0.0748))
  //            {
//		flag = 0;
  //              printf("%lf, %lf, %lf\n", vx, vy, vz);
    //          }

             if(flag <= 3000)
               {
                  if(judge > 0)
                   {
        		   // Apply a small linear velocity to the model.
                           printf("Add Torque!!!!!!!!!!!\n");
        		   //this->model->GetLink("link")->AddRelativeTorque(math::Vector3(torque, 0, 0));
        		   this->model->GetLink("link")->AddRelativeTorque(math::Vector3(2, 0, 0));
        		   //this->model->GetLink("link")->SetForce(math::Vector3(0, 0, 0));
                           //this->model->GetLink("link")->SetTorque(math::Vector3(fx, fy, 0));
                           
        		   //this->model->GetLink("link")->AddForce(math::Vector3(0, -0.3, 0));
                           printf("Velocity: %f %f %f\n",vel.x, vel.y, vel.z);
                           fprintf(fp1, "%lf, %lf, %lf\n",vel.x, vel.y, vel.z);
                           fprintf(fp2, "%lf, %lf, %lf\n",v.x, v.y, v.z);
                           
                           //printf("%f %f %f\n",fx, fy, fz);
           		   flag = flag + 1;
                           x1 = x1 + (a*sin(x3)+c*cos(x2))*delta_t;
      			   x2 = x2 + (b*sin(x1)+a*cos(x3))*delta_t;
      			   x3 = x3 + (c*sin(x2)+b*cos(x1))*delta_t;
                           delta_t += 0.001;
                          if( z > 0.15 && z < 0.5)
       			  {	
        		    integral[3] += ki*((-M_PI/90)-theta_y[0]);       
         		    this->model->GetLink("link")->AddRelativeTorque(math::Vector3(reference_torque[3], 0, 0));
       		          }

       //fprintf(fp3, "%lf, %lf, %lf, %lf\n",pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
                    }
                   
                  if(judge < 0)
                   {
			   // Apply a small linear velocity to the model.
                           printf("Add Torque!!!!!!!!!!!\n");
        		   this->model->GetLink("link")->AddRelativeTorque(math::Vector3(-torque, 0, 0));
                           //this->model->GetLink("link")->SetTorque(math::Vector3(fx, fy, 0));
                           
                           printf("Velocity: %f %f %f\n",vel.x, vel.y, vel.z);
                           fprintf(fp1, "%lf, %lf, %lf\n",vel.x, vel.y, vel.z);
                           fprintf(fp2, "%lf, %lf, %lf\n",v.x, v.y, v.z);
                           
                           //printf("%f %f %f\n",fx, fy, fz);
           		   flag = flag + 1;
                           x1 = x1 + (a*sin(x3)+c*cos(x2))*delta_t;
      			   x2 = x2 + (b*sin(x1)+a*cos(x3))*delta_t;
      			   x3 = x3 + (c*sin(x2)+b*cos(x1))*delta_t;
                           delta_t += 0.001;
			  if( z > 0.15 && z < 0.5)
       			  {	
        		    integral[4] += ki*((M_PI/90)-theta_y[0]);       
         		    this->model->GetLink("link")->AddRelativeTorque(math::Vector3(reference_torque[4], 0, 0));
       		          }

      /// fprintf(fp3, "%lf, %lf, %lf, %lf\n",pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
                   }
        		}
            else
        		{
        		   this->model->GetLink("link")->AddRelativeTorque(math::Vector3(0, 0, 0));
        		   //this->model->GetLink("link")->SetForce(math::Vector3(0, 0, 0));
        		   //this->model->SetWorldPose(math::Pose(x, y, z, 0, 0, 0, 0));
                           flag = flag + 1; 
                           printf("Velocity: %f %f %f\n",vel.x, vel.y, vel.z);
                           fprintf(fp1, "%lf, %lf, %lf\n",vel.x, vel.y, vel.z);
                           fprintf(fp2, "%lf, %lf, %lf\n",v.x, v.y, v.z);
                           //printf("%f %f %f\n",fx, fy, fz);
                           x1 = x1 + (a*sin(x3)+c*cos(x2))*delta_t;
      			   x2 = x2 + (b*sin(x1)+a*cos(x3))*delta_t;
      			   x3 = x3 + (c*sin(x2)+b*cos(x1))*delta_t;
                           delta_t += 0.001;
       //fprintf(fp3, "%lf, %lf, %lf, %lf\n",pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
        		}
     
       }
//if(flag > 100000)
  //          {
                   
    // 		   this->model->GetLink("link")->SetWorldPose(pose);
//}
       ang_vel = this->model->GetLink("link")->GetRelativeAngularVel();

       ang_velx = ang_vel.x;
       //ang_vely = ang_vel.y;
       ang_velz = ang_vel.z;
       
       //printf("%lf, %lf\n", ang_vely, ang_velz);

       theta_x[2] = theta_x[1];
       theta_x[1] = theta_x[0];
       //theta_x[0] = asin(2.0*pose.rot.y*pose.rot.z-2.0*pose.rot.w*pose.rot.x);
       theta_x[0] = pose.rot.GetPitch();
       applied_torque = this->model->GetLink("link")->GetWorldTorque();
       ang_vely += applied_torque.x/inertiax;
       printf("%lf, %lf\n", ang_vely, ang_velz);
       
       theta_y[2] = theta_y[1];
       theta_y[1] = theta_y[0];
         //theta_y[0] = atan2(-2.0*pose.rot.x*pose.rot.z-2.0*pose.rot.w*pose.rot.y, 1.0-2.0*pow(pose.rot.x, 2.0)-2.0*pow(pose.rot.y, 2.0));
       theta_y[0] = pose.rot.GetRoll();
       theta_z[2] = theta_z[1];
       theta_z[1] = theta_z[0];
       //theta_z[0] = atan2(-2.0*pose.rot.x*pose.rot.y-2.0*pose.rot.w*pose.rot.z, 1.0-2.0*pow(pose.rot.x, 2.0)-2.0*pow(pose.rot.z, 2.0));
       theta_z[0] = pose.rot.GetYaw();;
       //}
       //printf("%lf\n", theta_x[0]);
       //fprintf(fp3, "%lf\n",theta_y[0]);
       //if(ang_velx > 0)
       //{
       //if(theta_y[0] > 0)
       //  reference_torque += kp*(-theta_y[0]+theta_y[1]) - ki*theta_y[0] +kd * ((-theta_y[0]+theta_y[1])-(-theta_y[1]+theta_y[2]));
      // }
       
       //if(a)      
       //else if(theta_y[0] < 0)
       //  reference_torque -= kp*(-theta_y[0]+theta_y[1]) - ki*theta_y[0] +kd * ((-theta_y[0]+theta_y[1])-(-theta_y[1]+theta_y[2]));

 
       //if(reference_flag == 100000)
       //{
       printf("theta_y :%lf\n", theta_y[0]*1000000);
       printf(" yaw pitch :%lf %lf\n", pose.rot.GetYaw(), pose.rot.GetPitch());
       integral[0] += ki*(-theta_y[0]);       
       integral[1] += ki*-theta_x[0];       
       integral[2] += ki*-theta_z[0];       
       reference_torque[0] = kp*(-theta_y[0]) + integral[0] +kd * (-theta_y[0] + theta_y[1] );
       reference_torque[1] = kp*(-theta_x[0]) + integral[1] +kd * (-theta_x[0] + theta_x[1] );
       reference_torque[2] = kp*(-theta_z[0]) + integral[2] +kd * (-theta_z[0] + theta_z[1] );
       reference_torque[3] = kp*((-M_PI/90)-theta_y[0]) + integral[3] +kd * ((-M_PI/90)-theta_y[0]-(-M_PI/90) + theta_y[1] );
       reference_torque[4] = kp*((M_PI/90)-theta_y[0]) + integral[4] +kd * ((M_PI/90)-theta_y[0]-(M_PI/90) + theta_y[1] );
       if( z > 0.5)
      {
         this->model->GetLink("link")->AddRelativeTorque(math::Vector3(reference_torque[0], 0, 0));
         this->model->GetLink("link")->AddRelativeTorque(math::Vector3(0, reference_torque[1], 0));
         this->model->GetLink("link")->AddRelativeTorque(math::Vector3(0, 0, reference_torque[2]));
         //if(ang_vely == 0 && ang_velz == 0 && ang_velx == 0)
          //this->model->GetLink("link")->AddRelativeTorque(math::Vector3(0, 0, 0));
      }

           
       if(vx > -1e-13 && vx < 1e-13 && vy > -1e-13 && vy < 1e-13 && vz > -1e-13 && vz < 1e-13)
       {
         this->model->GetLink("link")->AddRelativeTorque(math::Vector3(reference_torque[0], 0, 0));
         this->model->GetLink("link")->AddRelativeTorque(math::Vector3(0, reference_torque[1], 0));
         this->model->GetLink("link")->AddRelativeTorque(math::Vector3(0, 0, reference_torque[2]));
       }  

      // if(z > 0.15 && z < 0.45)
      // {
      //    this->model->SetWorldPose(pose);
      //    this->model->SetAngularVel(math::Vector3(0, 0, 0));
      //    this->model->SetAngularAccel(math::Vector3(0, 0, 0));
      // }
       //if(ang_vel.x == 0 && ang_vel.y == 0 && ang_vel.z == 0)
       //    reference_flag = 1;

      // if(reference_flag == 1)
       //{
        //   pose.rot.x = 0;
         //  pose.rot.y = 0;
          // pose.rot.z = 0;
          // pose.rot.w = 0;
          // this->model->SetWorldPose(pose);
           //reference_flag = 0;
       //}

       //}
       //else if((theta_x[2] > 0) && (z > 0.15))
       // {
       //	   this->model->GetLink("link")->AddRelativeTorque(math::Vector3(-reference_torque, 0, 0));
       // }
     // if(input == 'f' && vx == 0 && vy == 0 && vz == 0)
      //    {
       //       x1 = x1;
        //      x2 = x2;
         //     x3 = x3;
          //    delta_t = delta_t;
         // }
              

      if(sw == 10000000000000000)
      {
        fclose(fp1);
        fclose(fp2);
        fclose(fp3);
      }

      if(i == 3)
      {
        i = 0;
      }
    
      else
      {
        i = i + 1;
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
