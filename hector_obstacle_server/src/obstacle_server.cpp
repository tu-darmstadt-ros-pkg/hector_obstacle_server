//
// Created by Karim Barth on 02.07.19.
//

#include "hector_obstacle_server/obstacle_server.h"
#include <iostream>
#include <algorithm>
#include <iterator>

namespace hector_obstacle_server
{

ObstacleServer::ObstacleServer()
{
  remove_obstacle_server = nh_.advertiseService( "hector_obstacle_server/remove_obstacle",
                                                 &ObstacleServer::removeObstacle, this );
  remove_obstacles_server = nh_.advertiseService( "hector_obstacle_server/remove_obstacles",
                                                &ObstacleServer::removeObstacles, this );
  add_obstacle_server_ = nh_.advertiseService( "hector_obstacle_server/add_obstacle", &ObstacleServer::addObstacle,
                                               this );
  get_obstacle_model_server_ = nh_.advertiseService( "hector_obstacle_server/get_obstacle_model",
                                                     &ObstacleServer::getObstacleModel, this );

  obstacle_model_pub_ = nh_.advertise<hector_obstacle_msgs::ObstacleModel>( "hector_obstacle_server/obstacle_model", 1,
                                                                            true );
}

void ObstacleServer::publishObstacleModel()
{
  std::vector<hector_obstacle_msgs::Obstacle> model_vev;
  convertMapToArray(model_, model_vev);
  hector_obstacle_msgs::ObstacleModel obstacle_model;
  obstacle_model.model = model_vev;
  obstacle_model_pub_.publish(obstacle_model);

}

bool ObstacleServer::addObstacle( hector_obstacle_msgs::AddObstacleRequest &request,
                                  hector_obstacle_msgs::AddObstacleResponse &response )
{

  boost::unique_lock<boost::shared_mutex> lock( model_access_ );
  static uint64_t id_counter = 0;
  std::stringstream id_stream;
  id_stream << "obstacle_" << id_counter;
  std::string id = id_stream.str();
  response.id =id;

  auto obstacle = request.obstacle;
  obstacle.id = id_stream.str();
  model_[id] = obstacle;

  ROS_INFO_STREAM( "ObstacleServer add obstacle with id: " << id_stream.str());
  id_counter++;

  publishObstacleModel();
  return true;
}


bool ObstacleServer::getObstacleModel( hector_obstacle_msgs::GetObstacleModelRequest &request,
                                       hector_obstacle_msgs::GetObstacleModelResponse &response )
{
  boost::shared_lock<boost::shared_mutex> lock( model_access_ );
  std::vector<hector_obstacle_msgs::Obstacle> model_vev;
  convertMapToArray(model_, model_vev);
  response.model.model = model_vev;
  return true;
}

bool ObstacleServer::removeObstacle( hector_std_msgs::StringServiceRequest &request,
                                     hector_std_msgs::StringServiceResponse &response )
{
  boost::unique_lock<boost::shared_mutex> lock( model_access_ );

  auto search = model_.find(request.param);
  if(search != model_.end())
  {
    ROS_INFO_STREAM( "ObstacleServer remove obstacle with id: " << request.param);
    model_.erase(request.param);
    publishObstacleModel();
    return true;
  }
  else{
    ROS_WARN_STREAM("ObstacleServer Obstacle with id " << request.param << " not found!");
    return false;
  }

}

bool ObstacleServer::removeObstacles( hector_obstacle_msgs::RemoveObstaclesRequest &request,
                                      hector_obstacle_msgs::RemoveObstaclesResponse &response )
{
  boost::unique_lock<boost::shared_mutex> lock( model_access_ );
  for(auto id : request.ids)
  {
    auto search = model_.find(id);
    if(search != model_.end())
    {
      ROS_INFO_STREAM( "ObstacleServer remove obstacle with id: " << id);
      model_.erase(id);
    }
    else{
      ROS_WARN_STREAM("ObstacleServer Obstacle with id " << id << " not found!");
      return false;
    }
  }
  publishObstacleModel();
  return true;
}

void ObstacleServer::convertMapToArray(std::map<std::string, hector_obstacle_msgs::Obstacle>& map,
  std::vector<hector_obstacle_msgs::Obstacle>& vec)
{
  std::transform(
    map.begin(),
    map.end(),
    std::back_inserter(vec),
    [](auto &kv){ return kv.second;}
  );
}

}