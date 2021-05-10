/*!*********************************************************************************
 *  \file       swarm_belief_updater_process.h
 *  \brief      SwarmBeliefUpdaterProcess definition file.
 *  \details    This file contains the BeliefUpdaterProcess declaration. To obtain more information about
 *              it's definition consult the belief_updater_process.cpp file.
 *  \authors    Guillermo De Fermin
 *  \copyright  Copyright 2016 Universidad Politecnica de Madrid (UPM)
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program. If not, see http://www.gnu.org/licenses/.
 ********************************************************************************/
#ifndef SWARM_BELIEF_UPDATER_PROCESS_H
#define SWARM_BELIEF_UPDATER_PROCESS_H

#include <string.h>
#include <ros/ros.h>
#include "droneMsgsROS/Observation3D.h"
#include "belief_manager_msgs/QueryBelief.h"
#include "belief_manager_msgs/AddBelief.h"
#include "belief_manager_msgs/GenerateID.h"
#include "belief_manager_msgs/RemoveBelief.h"
#include "droneMsgsROS/obsVector.h"
#include "droneMsgsROS/dronePose.h"
#include <sensor_msgs/BatteryState.h>
#include <aerostack_msgs/SocialCommunicationStatement.h>
#include <aerostack_msgs/SharedRobotPosition.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include <cmath>
#include <stdlib.h> 



class SwarmBeliefUpdaterProcess{
public:
  void ownSetUp();
  void ownStart();
  void ownStop();
  void ownRun();

private:
  struct Point {
    double x, y, z;
    Point(double x, double y, double z);
    Point();
    double maxDifference(Point p);
    void roundTo(double value);
  };



  ros::NodeHandle n;

  ros::Subscriber pose_subscriber;
  ros::Subscriber battery_subscriber;
  ros::Subscriber message_from_robot_sub; //?¿?¿
  ros::Subscriber shared_robot_positions_channel_sub;


  ros::ServiceClient add_client;
  ros::ServiceClient remove_client;
  ros::ServiceClient query_client;
  ros::ServiceClient generate_id_client;

  std::string drone_id_namespace;
  std::string pose_topic;
  std::string battery_topic;
  std::string message_from_robot;
  std::string shared_robot_positions_channel_str;


  void poseCallback(const geometry_msgs::PoseStamped& pose);
  void batteryCallback(const sensor_msgs::BatteryState& battery);
  void message_from_robotCallback(const aerostack_msgs::SocialCommunicationStatement &message);
  void sharedRobotPositionCallback(const aerostack_msgs::SharedRobotPosition &message);

  std::map<int,std::pair<geometry_msgs::Point, int32_t>> last_positions;
  geometry_msgs::Point vel;

  Point current_pose;
  std::string current_flight_state;
  std::string current_battery_level;


  bool sendFlightState(std::string flight_state);
  bool sendPose(Point pose);
  bool sendBatteryLevel(std::string level);
  std::vector<std::string> getPairs(std::string subs);
  std::vector<std::string> getSubstitutions(std::vector<std::string> pairs);
  bool collision_detected(geometry_msgs::Point shared_position,
  geometry_msgs::Point shared_vel, geometry_msgs::Point own_position, geometry_msgs::Point own_vel);
  double get_angle (geometry_msgs::Point shared_vel, geometry_msgs::Point own_vel);

  const int REQUIRED_MESSAGES = 5;
  const double POSE_MIN_DISTANCE = 0.1;
  const double BATTERY_LOW_THRESHOLD = 25;
  const double BATTERY_MEDIUM_THRESHOLD = 75;
  const double TIME_STEP=0.1;
  const double TEMPORAL_HORIZON=5.0;
  const double COLLISION_DISTANCE=1;

  int my_id;
};


#endif
