#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <thread>
#include <delta3_lib/Angles.h>
#include <cstdlib>
#include <math.h>

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
    this->pid = common::PID(5, 1, 0.5);
    std::cout << "\n\n"
              << this->model->GetName() << "\n\n";
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
    ros::SubscribeOptions so = ros::SubscribeOptions::create<delta3_lib::Angles>(
        "/" + this->model->GetName() + "/pos_cmd",
        1,
        boost::bind(&Delta3ModelPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->data_pub = this->rosNode->advertise<std_msgs::String>("/data", 1000);
    this->rosSub = this->rosNode->subscribe(so);

    this->rosQueueThread = std::thread(std::bind(&Delta3ModelPlugin::QueueThread, this));
    this->rosDataPublishThread = std::thread(std::bind(&Delta3ModelPlugin::Publish, this));
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Delta3ModelPlugin::OnUpdate, this));
    ROS_INFO("Finished setting up");
  }

private:
  void Publish()
  {
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
      if (this->update_num == -1)
      {
        std_msgs::String msg;
        std::stringstream ss;

        physics::LinkPtr gripper = this->model->GetLink("lower_triangle_holder");
        physics::LinkState state = physics::LinkState(gripper);
        ignition::math::Vector3d pos = state.Pose().Pos();
        double degrees[3] = {0, 0, 0};
        this->GetJointPoses(degrees);
        // std::cout << "Actual Arm Joint Pose: " << degrees[0] << " " << degrees[1] << " " << degrees[2] << " \n";
        // std::cout << "Expected Arm Joint Pose: " << angle[0] << " " << angle[1] << " " << angle[2] << "\n";
        // std::cout << "Gripper Pose: " << pos.X() << " " << pos.Y() << " " << pos.Z() << "\n\n";
        physics::ModelState modelState = physics::ModelState(this->model);
        ignition::math::Vector3d modelPose = modelState.Pose().Pos();

        ss << this->choose << " ";
        ss << modelPose.X() << " " << modelPose.Y() << " " << modelPose.Z() << " ";
        // ss << angle[0] << " " << angle[1] << " " << angle[2] << " ";
        // ss << degrees[0] << " " << degrees[1] << " " << degrees[2] << " ";
        // ss << pos.X() << " " << pos.Y() << " " << pos.Z();
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        this->data_pub.publish(msg);
        GenerateAngle();
        this->GenerateChoose();
        update_num = 0;
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

public:
  float Random(float a, float b)
  {
    float random = ((float)rand()) / (float)RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
  }

public:
  void OnUpdate()
  {
    if (update_num == 0)
    {

      if (choose != 1)
      {
        this->SetAngle("upper_arm_a1_s1", angle[0]);
      }
      if (choose != 2)
      {
        this->SetAngle("upper_arm_a2_s2", angle[1]);
      }
      if (choose != 3)
      {
        this->SetAngle("upper_arm_a3_s3", angle[2]);
      }
    }
    else if (update_num < 4000)
    {
      this->jointController->Update();
    }
    else if (update_num == 4000)
    {
      this->model->GetJoint("upper_arm_a1_s1")->SetParam("fmax", 0, 0);
      this->model->GetJoint("upper_arm_a2_s2")->SetParam("fmax", 0, 0);
      this->model->GetJoint("upper_arm_a3_s3")->SetParam("fmax", 0, 0);
      update_num = -1;
    }
    if (update_num >= 0)
    {
      update_num++;
    }
  }

private:
  void GenerateChoose()
  {
    switch (choose)
    {
    case 1:
      choose = 2;
      break;
    case 2:
      choose = 3;
      break;
    case 3:
      choose = 1;
      break;
    }
  }

private:
  void GenerateAngle()
  {
    angle[0] = this->Random(-60, 60);
    angle[1] = this->Random(-60, 60);
    angle[2] = this->Random(-60, 60);
  }

private:
  void GetJointPoses(double degrees[])
  {
    double p1 = physics::JointState(this->model->GetJoint("upper_arm_a1_s1")).Position(0);
    double p2 = physics::JointState(this->model->GetJoint("upper_arm_a2_s2")).Position(0);
    double p3 = physics::JointState(this->model->GetJoint("upper_arm_a3_s3")).Position(0);
    degrees[0] = p1 * 180 / M_PI;
    degrees[1] = p2 * 180 / M_PI;
    degrees[2] = p3 * 180 / M_PI;
  }

private:
  void SetAngle(std::string joint_name, float degree)
  {
    if (degree >= -60 && degree <= 60)
    {
      // std::cout << joint_name << std::endl;
      float rad = M_PI * degree / 180;
      std::string name = this->model->GetJoint(joint_name)->GetScopedName();
      this->jointController->SetPositionPID(name, pid);
      this->jointController->SetPositionTarget(name, rad);
    }
  }

public:
  void OnRosMsg(const delta3_lib::Angles::ConstPtr &msg)
  {
    angle[0] = msg->upper_arm1;
    angle[1] = msg->upper_arm2;
    angle[2] = msg->upper_arm3;
    update_num = 0;
  }

private:
  void SetJointAngle(const std::string &name, float degree)
  {
    float angle = (M_PI * degree / 180);
    this->jointController->SetPositionPID(name, this->pid);
    this->jointController->SetPositionTarget(name, angle);
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
  float gripper_pos[3] = {0, 0, 0};

private:
  int update_num = 0;

private:
  int choose = 1;

private:
  physics::ModelPtr model;

private:
  physics::JointControllerPtr jointController;

private:
  std::unique_ptr<ros::NodeHandle> rosNode;

private:
  ros::Subscriber rosSub;

private:
  ros::Subscriber linkStateSub;

private:
  std::thread rosQueueThread, rosDataPublishThread;

private:
  ros ::CallbackQueue rosQueue;

private:
  ros::Publisher data_pub;

private:
  event::ConnectionPtr updateConnection;

private:
  common::PID pid;
};
GZ_REGISTER_MODEL_PLUGIN(Delta3ModelPlugin);
} // namespace gazebo