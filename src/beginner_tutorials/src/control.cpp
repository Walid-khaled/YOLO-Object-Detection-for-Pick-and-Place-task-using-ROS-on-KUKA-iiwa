#include <cmath>
#include <iostream>
#include <csignal>

// Ros and other requirments ---------------------------
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
// -----------------------------------------------------

// States ----------------------------------------------
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/joint_position.hpp>
// -----------------------------------------------------

// Commands --------------------------------------------
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/joint_position.hpp>
#include <iiwa_ros/command/joint_velocity.hpp>
// -----------------------------------------------------

// Services --------------------------------------------
#include <iiwa_ros/service/path_parameters.hpp>
#include <iiwa_ros/service/path_parameters_lin.hpp>
#include <iiwa_ros/service/time_to_destination.hpp>
#include <iiwa_ros/service/control_mode.hpp>
#include "beginner_tutorials/commService.h"
// -----------------------------------------------------

using namespace std;

// Public Declerations ---------------------------------
iiwa_ros::state::JointPosition jp_state;
iiwa_ros::state::CartesianPose cp_state;

iiwa_ros::command::JointPosition jp_command;
iiwa_ros::command::CartesianPose cp_command;

iiwa_ros::service::PathParametersService jv_service;
iiwa_ros::service::TimeToDestinationService ttd_service;
iiwa_ros::service::ControlModeService control_mode;

ros::Publisher gripper_command;
std_msgs::Bool gripper_var;
geometry_msgs::Twist cartesian_velocity;

ros::ServiceClient client; 
beginner_tutorials::commService srv;
// -----------------------------------------------------

#define initial_speed 0.5
#define reach_speed 0.2

static bool quit{false};

void signalHandler(int /*unused*/) { quit = true; }


void readPosition(float &x, float &y, float &z, 
                  float &a, float &b, float &c, 
                  float &d,
                  bool disp = 0)
{
    auto joint_position_1 = cp_state.getPose();
    // cout << joint_position_.poseStamped << endl;
    x = joint_position_1.poseStamped.pose.position.x;
    y = joint_position_1.poseStamped.pose.position.y;
    z = joint_position_1.poseStamped.pose.position.z;

    a = joint_position_1.poseStamped.pose.orientation.x;
    b = joint_position_1.poseStamped.pose.orientation.y;
    c = joint_position_1.poseStamped.pose.orientation.z;
    d = joint_position_1.poseStamped.pose.orientation.w;

    if(disp)
    {
      cout << "x = " << x << endl;
      cout << "y = " << y << endl;
      cout << "z = " << z << endl;
      cout << "a = " << a << endl;
      cout << "b = " << b << endl;
      cout << "c = " << c << endl;
      cout << "d = " << d << endl;
      cout << "--------------------------" << endl;
    }

    return;
}

void writePosition(bool delta_pos = 0,
                   float x = 0, float y = 0, float z = 0, 
                   float a = 0, float b = 0, float c = 0, 
                   float d = 0)
{
    auto joint_position_ = cp_state.getPose();
    if(delta_pos)
    {
      joint_position_.poseStamped.pose.position.x += x;
      joint_position_.poseStamped.pose.position.y += y;
      joint_position_.poseStamped.pose.position.z += z;
    
      joint_position_.poseStamped.pose.orientation.x += a;
      joint_position_.poseStamped.pose.orientation.y += b;
      joint_position_.poseStamped.pose.orientation.z += c;
      joint_position_.poseStamped.pose.orientation.w += d;
    }
    else
    {
      geometry_msgs::PoseStamped new_pose;
      new_pose = joint_position_.poseStamped;
      // move to initial position
      // Position
      new_pose.pose.position.x = x;
      new_pose.pose.position.y = y;
      new_pose.pose.position.z = z;
      cp_command.setPose(new_pose);
      return;
    }
    
    // cout << joint_position_.poseStamped << endl;

    cp_command.setPose(joint_position_.poseStamped);

    return;
}

void waitToFinish(double timeout)
{
    cout << "[INFO] Start" << endl;
    double ttd = ttd_service.getTimeToDestination();
    
    ros::Time start_wait = ros::Time::now();
  
    while (ttd <= 0.0 && (ros::Time::now() - start_wait) < ros::Duration(timeout)) 
    {
      cout << "[INFO] Waiting for ttd." << endl;
      ros::Duration(0.5).sleep();
      ttd = ttd_service.getTimeToDestination();
      cout << "[INFO] ttd value is " << ttd << " seconds." << endl;
    }

    if (ttd > 0.0) 
    {
      cout << "[INFO] Excution time is " << ttd << " seconds." << endl;
      ros::Duration(ttd).sleep();
      cout << "[INFO] Done." << endl << endl;
      ros::Duration(1.5).sleep();
      return;
    }  
    
    cout << "[WARNING] TIMEOUT." << endl << endl;
    ros::Duration(1.5).sleep();
    return; 
}

// float place_pose[] = {0.135309, -0.530633, 0.643638, 0.512745922629, 0.858094930649, 0.000837295395136, 0.0276414488071};
float place_pose[] = {0.495470558961, -0.0307183783682, 0.648702145525, 0.512799055092, 0.858067154884, 0.00102119154473, 0.0275111259108};
float pick_pose[] = {0.150848839584, -0.541409595446, 0.526834957526, 0.512763064851, 0.858089268208, 0.000986631077723, 0.0274925261635};
float ideal_pose[] = {0.230289606536, -0.302054542641, 0.510265685153, 0.512659302303, 0.858152866364, 0.00109189842816, 0.0274397194815};

#define place_delta_z (0.350 - 0.275)
#define place_z (0.648702145525 - 0.350) 

#define pick_delta_z (0.350 - 0.275)
#define pick_z (0.526834957526 - 0.345) 

#define wait 2

void gotoPosistion(string pose)
{
    if(pose == "place")
    {
      cout << "[INFO] Moving to placing initial position." << endl;
      auto joint_position_ = cp_state.getPose();
      // Position
      joint_position_.poseStamped.pose.position.x = place_pose[0];
      joint_position_.poseStamped.pose.position.y = place_pose[1];
      joint_position_.poseStamped.pose.position.z = place_pose[2];
      // Orientation
      joint_position_.poseStamped.pose.orientation.x = place_pose[3];
      joint_position_.poseStamped.pose.orientation.y = place_pose[4];
      joint_position_.poseStamped.pose.orientation.z = place_pose[5];
      joint_position_.poseStamped.pose.orientation.w = place_pose[6];
      cp_command.setPose(joint_position_.poseStamped);
      return;
    }

    else if(pose == "pick")
    {
      cout << "[INFO] Moving to picking initial position." << endl;
      auto joint_position_ = cp_state.getPose();
      // Position
      joint_position_.poseStamped.pose.position.x = pick_pose[0];
      joint_position_.poseStamped.pose.position.y = pick_pose[1];
      joint_position_.poseStamped.pose.position.z = pick_pose[2];
      // Orientation
      joint_position_.poseStamped.pose.orientation.x = pick_pose[3];
      joint_position_.poseStamped.pose.orientation.y = pick_pose[4];
      joint_position_.poseStamped.pose.orientation.z = pick_pose[5];
      joint_position_.poseStamped.pose.orientation.w = pick_pose[6];
      cp_command.setPose(joint_position_.poseStamped);
      return;
    }

    else if(pose == "ideal")
    {
      cout << "[INFO] Moving to picking initial position." << endl;
      auto joint_position_ = cp_state.getPose();
      // Position
      joint_position_.poseStamped.pose.position.x = ideal_pose[0];
      joint_position_.poseStamped.pose.position.y = ideal_pose[1];
      joint_position_.poseStamped.pose.position.z = ideal_pose[2];
      // Orientation
      joint_position_.poseStamped.pose.orientation.x = ideal_pose[3];
      joint_position_.poseStamped.pose.orientation.y = ideal_pose[4];
      joint_position_.poseStamped.pose.orientation.z = ideal_pose[5];
      joint_position_.poseStamped.pose.orientation.w = ideal_pose[6];
      cp_command.setPose(joint_position_.poseStamped);
      return;
    }
}

void gripper(string command, float angle = 0)
{ 
  if (command == "R")
  {
    auto joint_position = jp_state.getPosition();
    joint_position.position.a7 += angle;

    if( joint_position.position.a7 > 3.0543)
      joint_position.position.a7 = -(180 - joint_position.position.a7);
    else if( joint_position.position.a7 < -3.0543)
      joint_position.position.a7 = +(180 + joint_position.position.a7);

    jp_command.setPosition(joint_position);
    return;
  }

  else if(command == "O")
  {
    gripper_var.data = true;
    gripper_command.publish(gripper_var);
    return;
  }

  else if(command == "C")
  {
    gripper_var.data = false;
    gripper_command.publish(gripper_var);
    return;
  }

  return;
}

bool message_communication(string msg, int idx)
{
  srv.request.f = msg;
  srv.request.x = idx;

  if (client.call(srv))
  {
    return 1;
  }
  else
  {
    ROS_ERROR("Failed to call service comm");
    return 0;
  }
}

float dx = 0;
float dy = 0;
float dz = 0;
float angle = 0;

bool go_shape_position(int idx = 0)
{
  // Closing Gripper
  gripper("O");
  waitToFinish(0.1);

  jv_service.setSmartServoJointSpeedLimits(initial_speed, initial_speed);
  ros::Duration(0.5).sleep();

  // move to pick initial position
  gotoPosistion("pick");
  waitToFinish(wait);

  // wait unitil manipulator stable
  ros::Duration(wait).sleep();

  // Put YOLO in Prediction Mode
  message_communication("YP", idx);

  // wait to get stable readings
  ros::Duration(10).sleep();

  // Get values of center and angle of shape  
  if (message_communication("YS", idx))
  {
    dx = srv.response.dx;
    dy = srv.response.dy;
    angle = srv.response.angle;

    cout << dx << " " << dy << " " << angle << endl;
  }
  else
  {
    ROS_ERROR("Failed to call service comm");
    return -1;
  }

  cout << "Ready to move to shape position" << endl;
  writePosition(true, dx/1000, dy/1000);
  waitToFinish(wait);

  // Retake the center for fine tuning
  // wait unitil manipulator stable
  ros::Duration(wait).sleep();

  // Put YOLO in Prediction Mode
  message_communication("YP", idx);

  // wait to get stable readings
  ros::Duration(10).sleep();

  // Get values of center and angle of box  
  if (message_communication("YS", idx))
  {
    dx = srv.response.dx;
    dy = srv.response.dy;
    angle = srv.response.angle;

    cout << dx << " " << dy << " " << angle << endl;
  }
  else
  {
    ROS_ERROR("Failed to call service comm");
    return -1;
  }

  cout << "Approaching the shape" << endl;
  // Approch the shape
  writePosition(true, (dx + 30)/1000, (dy - 17.5)/1000, -pick_z);
  waitToFinish(wait);


  // Rotate the gripper
  if(angle != 0)
  {
    gripper("R", angle);
    waitToFinish(0.2);
  }

  jv_service.setSmartServoJointSpeedLimits(reach_speed, reach_speed);
  ros::Duration(0.5).sleep();

  // Reach the shape
  writePosition(true, 0, 0, -pick_delta_z);
  waitToFinish(wait);
  
  // Close the Gripper
  gripper("C");
  waitToFinish(0.2);

  cout << "Retracting from the shape" << endl;
  // Move a way from the shape
  writePosition(true, 0, 0, pick_delta_z);
  waitToFinish(wait);


  // // Re-rotate the gripper
  // if(angle != 0)
  // {
  //   gripper("R", -angle);
  //   waitToFinish(0.2);
  // }

  cout << "picking operation finished" << endl;
  return 1;
}

bool go_box_position(int idx = 0)
{ 
  // Closing Gripper
  gripper("C");
  waitToFinish(0.2);

  jv_service.setSmartServoJointSpeedLimits(initial_speed, initial_speed);
  ros::Duration(0.5).sleep();

  // Put ColorSegmentation in Ready Mode
  message_communication("CSR", idx);

  // move to place initial position
  gotoPosistion("place");
  waitToFinish(wait);

  // Put ColorSegmentation in Prediction Mode
  message_communication("CSP", idx);

  // wait to get stable readings
  ros::Duration(0.5).sleep();
  //ros::Duration(wait).sleep();

  // Get values of center and angle of box  
  if (message_communication("CSS", idx))
  {
    dx = srv.response.dx;
    dy = srv.response.dy;
    angle = srv.response.angle;

    cout << dx << " " << dy << " " << angle << endl;
  }
  else
  {
    ROS_ERROR("Failed to call service comm");
    return -1;
  }

  cout << "Ready to move to box position" << endl;
  writePosition(true, dx/1000, dy/1000);
  waitToFinish(wait);

  // Retake the center for fine tuning
  // wait unitil manipulator stable
  //ros::Duration(wait).sleep();
  ros::Duration(0.5).sleep();

  // Put ColorSegmentation in Prediction Mode
  message_communication("CSP", idx);

  // wait to get stable readings
  //ros::Duration(wait).sleep();
  ros::Duration(0.5).sleep();

  // Get values of center and angle of box  
  if (message_communication("CSS", idx))
  {
    dx = srv.response.dx;
    dy = srv.response.dy;
    angle = srv.response.angle;

    cout << dx << " " << dy << " " << angle << endl;
  }
  else
  {
    ROS_ERROR("Failed to call service comm");
    return -1;
  }

  cout << "Approaching the Box" << endl;
  // Approch the Box
  writePosition(true, (dx + 19)/1000, (dy - 23)/1000, -place_z);
  waitToFinish(wait);


  // Rotate the gripper
  if(angle != 0)
  {
    gripper("R", angle);
    waitToFinish(0.2);
  }

  // Reach the box
  writePosition(true, 0, 0, -place_delta_z);
  waitToFinish(wait);

  //jv_service.setSmartServoJointSpeedLimits(reach_speed, reach_speed);
  //ros::Duration(0.5).sleep();
  
  // Open the Gripper
  gripper("O");
  waitToFinish(0.2);

  cout << "Retracting from the Box" << endl;
  // Move a way from the box
  writePosition(true, 0, 0, place_delta_z);
  waitToFinish(wait);

  // // Rerotate the gripper
  // if(angle != 0)
  // {
  //   gripper("R", -angle);
  //   waitToFinish(0.2);
  // }

  cout << "Placing operation finished" << endl;
  return 1;
}

float x, y, z, a, b, c, d;

string shapes[] = {"cir", "hex", "rec", "rng", "slt", "sqr"};

bool picking(string shape_name = "cir")
{ 
  int idx = -1;
  
  // activate picking shape detection script

    for (int j = 0; j < 6; j++)
    {
      if(shape_name == shapes[j])
      {
        idx = j;
        break;
      }
    }

    if (idx == -1)
    {
      cout << "Shape name is not correct" << endl;
      return 0;
    }

    else if(go_shape_position(idx) == 0) 
      return 0;

  return 1;
}

string boxes[] = {"R", "B", "Y"};

bool placing(string box_color = "R")
{
  int idx = -1;
  // activate placing box detection script

    for (int j = 0; j < 6; j++)
    {
      if(box_color == boxes[j])
      {
        idx = j;
        break;
      }
    }

    if (idx == -1)
    {
      cout << "Box color is not correct" << endl;
      return 0;
    }

    else if(go_box_position(idx) == 0) 
      return 0;

  return 1;
}


int main() {
  // init
  cp_state.init("iiwa");
  jp_state.init("iiwa");
  
  cp_command.init("iiwa");
  jp_command.init("iiwa");
  
  // cv_service.init("iiwa");
  jv_service.init("iiwa");
  ttd_service.init("iiwa");
  control_mode.init("iiwa");
  // ------------------------------------------------------------


  // Prepare computer vision Service
  ros::NodeHandle n;
  client = n.serviceClient<beginner_tutorials::commService>("comm_Service");

  gripper_command = n.advertise<std_msgs::Bool>("/iiwa/command/GripperCommand", 1000);

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // ------------------------------------------------------------

  // Signal handlers.
  signal(SIGTERM, signalHandler);
  signal(SIGINT, signalHandler);
  signal(SIGHUP, signalHandler);
  // ------------------------------------------------------------

  // Wait a bit, so that you can be sure the subscribers are connected.
  //jv_service.setSmartServoJointSpeedLimits(initial_speed, initial_speed);
  //ros::Duration(0.5).sleep();
  // ------------------------------------------------------------


  string input;
  for (int i = 0; i < 3; i++)
  {
    cout << "Enter shape : ";
    cin >> input;
    if(!picking(input))
      continue;

    cout << "Enter box color : ";
    cin >> input;
    if(!placing(input))
      continue;
  }

  cout << "Go to Ideal position" << endl;
  gotoPosistion("ideal");
  waitToFinish(2);

  std::cerr << "Stopping spinner..." << std::endl;
  spinner.stop();

  std::cerr << "Bye!" << std::endl;

  return 0;
}
