#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <ignition/math/Vector3.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "std_msgs/Float32.h"
#include <thread>
#include <delta3_lib/Angles.h>

namespace gazebo
{
class Delta3ModelPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    this->model = _parent;
    this->jointController = this->model->GetJointController();
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      ROS_FATAL_STREAM("Ros is not initialized."
                       << "Load the .. in gazebo_ros");
    }
    else
    {
      ROS_INFO("Starting plugin");
    }

    std::cout << "\n\n"
              << this->model->GetName() << "\n\n";
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
    ros::SubscribeOptions so = ros::SubscribeOptions::create<delta3_lib::Angles>(
        "/" + this->model->GetName() + "/pos_cmd",
        1,
        boost::bind(&Delta3ModelPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    this->rosQueueThread = std::thread(std::bind(&Delta3ModelPlugin::QueueThread, this));
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Delta3ModelPlugin::OnUpdate, this));
    ROS_INFO("Finished setting up");
  }

public:
  void OnUpdate()
  {
    if (update_num == 0)
    {
      
      this->SetAngle("upper_arm_a1_s1", angle[0]);
      this->SetAngle("upper_arm_a2_s2", angle[1]);
      this->SetAngle("upper_arm_a3_s3", angle[2]);
      angle[0] = 0;
      angle[1] = 0;
      angle[2] = 0;
    }
    else if (update_num < 200)
    {
      this->jointController->Update();
    }
    else if (update_num == 200)
    {
      this->model->GetJoint("upper_arm_a1_s1")->SetParam("fmax", 0, 0);
      this->model->GetJoint("upper_arm_a2_s2")->SetParam("fmax", 0, 0);
    }
    update_num++;
  }

private:
  void SetAngle(std::string joint_name, float degree)
  {
    if (degree >= -90 && degree <= 90)
    {
      float rad = 3.14 * degree / 180;
      std::string name = this->model->GetJoint(joint_name)->GetScopedName();
      this->jointController->SetPositionPID(name, common::PID(2, 0, 0));
      this->jointController->SetPositionTarget(name, rad);
    }
  }

public:
  void OnRosMsg(const delta3_lib::Angles::ConstPtr &msg)
  {
    // std::cout << "Angle " << this->angle << "\n"; // << " " << msg->upper_arm2 << " " << msg->upper_arm3 << std::endl;
    // physics::JointPtr upper_arm1 = this->model->GetJoint("upper_arm_a1_s1");
    // physics::JointPtr upper_arm2 = this->model->GetJoint("upper_arm_a2_s2");
    // physics::JointPtr upper_arm3 = this->model->GetJoint("upper_arm_a3_s3");
    // upper_arm1->SetForce(0, msg->upper_arm1);
    // this->angle = msg->upper_arm1;
    // this->jointController->SetJointPosition(upper_arm1, (3.14 * msg->upper_arm1 / 180));
    update_num = 0;
    angle[0] = msg->upper_arm1;
    angle[1] = msg->upper_arm2;
    angle[2] = msg->upper_arm3;
    // jc->SetJointPosition(upper_arm2, (3.14 * msg->upper_arm2 / 180));
    // jc->SetJointPosition(upper_arm3, (3.14 * msg->upper_arm3 / 180), 1);
    // float angle = (3.14 * msg->upper_arm1 / 180);
    // jc->SetPositionPID("upper_arm_a1_s1", common::PID(100, 0, 0));
    // jc->SetPositionPID("upper_arm_a2_s2", common::PID(100, 0, 0));
    // jc->SetPositionTarget("upper_arm_a1_s1", angle);
    // angle = (3.14 * msg->upper_arm2 / 180);
    // jc->SetPositionTarget("upper_arm_a2_s2", angle);
    // auto name = "upper_arm_a1_s1";
    // this->SetJointAngle(name, msg->upper_arm1, msg->upper_arm2, msg->upper_arm3);
  }

private:
  void SetJointAngle(const std::string &name, float degree)
  {
    float angle = (3.14 * degree / 180);
    this->jointController->SetPositionPID("upper_arm_a1_s1", common::PID(10, 0, 0));
    this->jointController->SetPositionTarget("upper_arm_a1_s1", angle);
    this->jointController->Update();
  }

public:
  void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

private:
  float angle[3] = {0, 0, 0};

private:
  int update_num = 0;

private:
  physics::ModelPtr model;

private:
  physics::JointControllerPtr jointController;

private:
  std::unique_ptr<ros::NodeHandle> rosNode;

private:
  ros::Subscriber rosSub;

private:
  std::thread rosQueueThread;

private:
  ros ::CallbackQueue rosQueue;

private:
  event::ConnectionPtr updateConnection;
};
GZ_REGISTER_MODEL_PLUGIN(Delta3ModelPlugin);
} // namespace gazebo