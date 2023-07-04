
#include "ros/ros.h"
#include <franka_effort_controller/franka_effort_controller.h>

namespace franka_effort_controller
{
  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to an \b effort interface.
   */
  typedef joint_trajectory_controller::FrankaJointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>>
          FrankaJointTrajectoryController;
}



int main(int argc, char **argv)
{

    ros::init(argc,argv,"franka_effort_controller_node");
    ros::NodeHandle node;
    franka_effort_controller::FrankaJointTrajectoryController main_controller(&node,&altitude_controller);

    main_controller.initialize();

    ros::spin();




}