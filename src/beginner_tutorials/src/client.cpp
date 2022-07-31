#include "ros/ros.h"
#include "beginner_tutorials/commService.h"
#include <cstdlib>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "comm_Service");
  if (argc != 3)
  {
    ROS_INFO("usage: comm_Server F I");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::commService>("comm_Service");
  beginner_tutorials::commService srv;
  
  srv.request.f = argv[1];
  srv.request.x = atoll(argv[2]);

  if (client.call(srv))
  {
    cout << srv.response.index << endl;
    cout << srv.response.dx << endl;
    cout << srv.response.dy << endl;
    cout << srv.response.angle << endl;
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}