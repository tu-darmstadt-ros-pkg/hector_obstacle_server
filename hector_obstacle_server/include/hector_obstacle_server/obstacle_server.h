//
// Created by Karim Barth on 02.07.19.
//

#ifndef HECTOR_OBSTACLE_SERVER_OBSTACLE_SERVER_H
#define HECTOR_OBSTACLE_SERVER_OBSTACLE_SERVER_H

#include <ros/service.h>
#include <ros/node_handle.h>
#include <std_srvs/Empty.h>
#include <hector_std_msgs/GetPoseArrayService.h>
#include <hector_std_msgs/PoseService.h>

#include <hector_obstacle_msgs/ObstacleModel.h>
#include <hector_obstacle_msgs/AddObstacle.h>
#include <hector_obstacle_msgs/GetObstacleModel.h>
#include <hector_obstacle_msgs/RemoveObstacles.h>
#include <hector_std_msgs/StringService.h>
#include <boost/thread.hpp>

namespace hector_obstacle_server
{
class ObstacleServer
{
public:
  ObstacleServer();

protected:
  void publishObstacleModel();
  bool
  addObstacle( hector_obstacle_msgs::AddObstacleRequest &request, hector_obstacle_msgs::AddObstacleResponse &response );

  bool getObstacleModel( hector_obstacle_msgs::GetObstacleModelRequest &request,
                         hector_obstacle_msgs::GetObstacleModelResponse &response );

  bool
  removeObstacle( hector_std_msgs::StringServiceRequest &request, hector_std_msgs::StringServiceResponse &response );

  bool
  removeObstacles( hector_obstacle_msgs::RemoveObstaclesRequest &request,
                   hector_obstacle_msgs::RemoveObstaclesResponse &response );

  void convertMapToArray( std::map<std::string, hector_obstacle_msgs::Obstacle> &map,
                          std::vector<hector_obstacle_msgs::Obstacle> &vec );

  ros::NodeHandle nh_;

  ros::ServiceServer add_obstacle_server_;
  ros::ServiceServer get_obstacle_model_server_;
  ros::ServiceServer remove_obstacle_server;
  ros::ServiceServer remove_obstacles_server;
  ros::Publisher obstacle_model_pub_;

  //hector_obstacle_msgs::ObstacleModel model_;
  std::map<std::string, hector_obstacle_msgs::Obstacle> model_;
  boost::shared_mutex model_access_;
};
}


#endif //HECTOR_OBSTACLE_SERVER_OBSTACLE_SERVER_H
