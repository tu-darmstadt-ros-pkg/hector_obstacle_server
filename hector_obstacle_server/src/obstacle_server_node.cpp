//
// Created by Karim Barth on 02.07.19.
//

#include <ros/ros.h>
#include "hector_obstacle_server/obstacle_server.h"

int main( int argc, char **argv )
{
  ros::init( argc, argv, "hector_obstacle_server" );
  ros::NodeHandle n;

  auto obstacle_server = std::make_unique<hector_obstacle_server::ObstacleServer>();

  ros::Rate rate( 40 );

  while ( !ros::isShuttingDown())
  {
    ros::spinOnce();
    rate.sleep();
  }

  obstacle_server = nullptr;

  return 0;
}