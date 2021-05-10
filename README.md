# Brief
This process is responsible for updating the belief memory with values obtained from several perception topics.

# Subscribed topics
- **estimated_pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html))  
Estimated current pose of the drone.

- **battery** ([sensor_msgs/BatteryState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/BatteryState.html))  
Current battery percentage.

- **message_from_robot** ([aerostack_msgs/SocialCommunicationStatement](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/8d8dda3bb1547b445a2a5f4ca43665f8fcd53a58/msg/SocialCommunicationStatement.msg))  
Receive messages from other robots.

- **shared_robot_positions_channel** ([aerostack_msgs/SocialCommunicationStatement](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/8d8dda3bb1547b445a2a5f4ca43665f8fcd53a58/msg/SharedRobotPosition.msg))  
Receive positions of other robots.

# Service clients
- **add_client**([aerostack_msgs/AddBelief](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/8d8dda3bb1547b445a2a5f4ca43665f8fcd53a58/srv/AddBelief.srv))  
Add predicates to belief memory.

- **remove_client**([aerostack_msgs/RemoveBelief](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/8d8dda3bb1547b445a2a5f4ca43665f8fcd53a58/srv/RemoveBelief.srv))  
Remove predicates to belief memory.

- **query_client**([aerostack_msgs/QueryBelief](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/8d8dda3bb1547b445a2a5f4ca43665f8fcd53a58/srv/QueryBelief.srv))  
Query to the memory of beliefs

- **generate_id_client**([droneMsgsROS/GenerateID](https://bitbucket.org/joselusl/dronemsgsros/src/master/srv/GenerateID.srv))  
Generate new id.
---

