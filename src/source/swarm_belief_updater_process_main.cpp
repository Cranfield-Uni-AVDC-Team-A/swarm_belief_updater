#include "swarm_belief_updater_process.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, ros::this_node::getName());

  SwarmBeliefUpdaterProcess swarm_belief_updater;
  swarm_belief_updater.ownSetUp();
  swarm_belief_updater.ownStart();

  ros::Rate r(100); // 100 hz
  while (ros::ok())
  {
   ros::spinOnce();
   r.sleep();
  }
}
