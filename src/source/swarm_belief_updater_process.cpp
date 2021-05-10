#include "swarm_belief_updater_process.h"

void SwarmBeliefUpdaterProcess::ownSetUp() {
  ros::NodeHandle private_nh("~");
  //private_nh.param<std::string>("pose_topic", pose_topic, "self_localization/pose");
  //private_nh.param<std::string>("battery_topic", battery_topic, "sensor_measurement/battery_state");
  private_nh.param<std::string>("message_from_robot", message_from_robot,"message_from_robot");
  private_nh.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");
  private_nh.param<std::string>("shared_robot_positions_channel_topic", shared_robot_positions_channel_str,
                                 "shared_robot_positions_channel");
}

void SwarmBeliefUpdaterProcess::ownStart() {
  //pose_subscriber = n.subscribe(pose_topic, 1, &SwarmBeliefUpdaterProcess::poseCallback, this);
  //battery_subscriber = n.subscribe(battery_topic, 1, &SwarmBeliefUpdaterProcess::batteryCallback, this);
  message_from_robot_sub =n.subscribe('/' + message_from_robot, 100,
                            &SwarmBeliefUpdaterProcess::message_from_robotCallback, this);
  shared_robot_positions_channel_sub =
      n.subscribe('/' + shared_robot_positions_channel_str, 1000, &SwarmBeliefUpdaterProcess::sharedRobotPositionCallback, this);
  add_client = n.serviceClient<belief_manager_msgs::AddBelief>("add_belief");
  remove_client = n.serviceClient<belief_manager_msgs::RemoveBelief>("remove_belief");
  query_client = n.serviceClient<belief_manager_msgs::QueryBelief>("query_belief");
  generate_id_client = n.serviceClient<belief_manager_msgs::GenerateID>("belief_manager_process/generate_id");

//It's needed to wait at least 2 seconds because if you don't wait it is possible that the clients are not connected.
  ros::Duration(2).sleep();
}

void SwarmBeliefUpdaterProcess::ownStop() {
  message_from_robot_sub.shutdown();
  shared_robot_positions_channel_sub.shutdown();
}

void SwarmBeliefUpdaterProcess::ownRun() {}


std::vector<std::string> SwarmBeliefUpdaterProcess::getPairs(std::string subs){
  std::vector<std::string> string_portions;
  int ini=0;
  int pos=0;
  while((pos=subs.find("\n",pos))!=std::string::npos){
    string_portions.push_back(subs.substr(ini,pos-ini));
    pos=pos+1;
    ini=pos;
  }
  //now it is going to delete spaces
  std::vector<std::string> res;
  for(int j=0;j<string_portions.size();j++){
    std::string aux="";
    for(int  i = 0; string_portions[j][i] != 0;i++){
      if(string_portions[j][i] != 32){
        aux=aux+string_portions[j][i];
      }
    }
    res.push_back(aux);
  }
return res;
}

std::vector<std::string> SwarmBeliefUpdaterProcess::getSubstitutions(std::vector<std::string> pairs){
  std::vector<std::string>res;
  for (int i=0; i<pairs.size();i++){
     res.push_back(pairs[i].substr(pairs[i].find(":")+1,pairs[i].size()-1));
  }
  return res;
}

void SwarmBeliefUpdaterProcess::message_from_robotCallback(const aerostack_msgs::SocialCommunicationStatement &message) {

  if(message.sender != drone_id_namespace && message.receiver ==drone_id_namespace){
    belief_manager_msgs::QueryBelief srv;

    srv.request.query = "object(?x,drone), name(?x,"+message.sender+")"; 
    query_client.call(srv);
    belief_manager_msgs::QueryBelief::Response response= srv.response;
    int id;
    if(response.success==false){
      belief_manager_msgs::GenerateID::Request req;
      belief_manager_msgs::GenerateID::Response res;
      generate_id_client.call(req, res);
      if (res.ack)
      {
        id = res.id;
      }   
      belief_manager_msgs::AddBelief srv2;
      std::stringstream s;
      s << "object(" << id << ", drone), name(" << id << ","<< message.sender <<")";
      srv2.request.belief_expression = s.str();
      srv2.request.multivalued = true;
      add_client.call(srv2);
      belief_manager_msgs::AddBelief::Response response= srv2.response;
      int id;
   }else{
    std::string sub = response.substitutions;
    std::vector<std::string> pairs = getPairs(sub);
    std::vector<std::string> subs = getSubstitutions(pairs);
    id = std::stoi(subs[0]);
   }
  YAML::Node content = YAML::Load(message.content);
  if(content["TEXT"]){
    std::string text=content["TEXT"].as<std::string>();
    belief_manager_msgs::GenerateID::Request req2;
    belief_manager_msgs::GenerateID::Response res2;
    generate_id_client.call(req2, res2);
    int id_message;
    if (res2.ack)
    {
      id_message = res2.id;
    }
    belief_manager_msgs::AddBelief::Request req;
    belief_manager_msgs::AddBelief::Response res;
    std::stringstream ss;
    ss << "object(" << id_message << ", message) , sender(" << id_message<<", " << id << "), text(" << id_message<<", "<< text <<")";
    req.belief_expression = ss.str();
    req.multivalued = false;
    add_client.call(req, res);
  }
}
}
double SwarmBeliefUpdaterProcess::get_angle(geometry_msgs::Point shared_vel, geometry_msgs::Point own_vel){
  double x1 = shared_vel.x;
  double y1 = shared_vel.y;
  double z1 = shared_vel.z;

  double x2 = own_vel.x;
  double y2 = own_vel.y;
  double z2 = own_vel.z;

  double dot = x1*x2 + y1*y2;
  double lenSq1 = x1*x1 + y1*y1;
  double lenSq2 = x2*x2 + y2*y2;
  double angle = std::acos(dot/std::sqrt(lenSq1 * lenSq2))  * 180.0 / M_PI ;
  return angle;
}

bool SwarmBeliefUpdaterProcess::collision_detected(geometry_msgs::Point shared_position , geometry_msgs::Point shared_vel 
  , geometry_msgs::Point own_position , geometry_msgs::Point own_vel){
  int times= (int)TEMPORAL_HORIZON/TIME_STEP;
  for (int i = TIME_STEP; i<=times;i=i+1){
    // shared position until TEMPORAL_HORIZON 
    geometry_msgs::Point next_shared_position;
    next_shared_position.x = shared_position.x+shared_vel.x*(TIME_STEP*i);
    next_shared_position.y = shared_position.y+shared_vel.y*(TIME_STEP*i);
    next_shared_position.z = shared_position.z+shared_vel.z*(TIME_STEP*i);
    // own position until TEMPORAL_HORIZON 
    geometry_msgs::Point next_own_position;
    next_own_position.x = own_position.x+own_vel.x*(TIME_STEP*i);
    next_own_position.y = own_position.y+own_vel.y*(TIME_STEP*i);
    next_own_position.z = own_position.z+own_vel.z*(TIME_STEP*i);

    double dist = sqrt( pow(next_shared_position.x - next_own_position.x,2.) + 
      pow(next_shared_position.y - next_own_position.y,2.) + 
      pow(next_shared_position.z - next_own_position.z,2.));
  if(dist<=COLLISION_DISTANCE){
      return true; 
    }
  }
  return false;
}

void SwarmBeliefUpdaterProcess::sharedRobotPositionCallback(
    const aerostack_msgs::SharedRobotPosition &message)
{
  if(message.sender != drone_id_namespace){
    belief_manager_msgs::QueryBelief srv;
    srv.request.query = "object(?x,drone), name(?x,"+message.sender+")"; 
    query_client.call(srv);
    belief_manager_msgs::QueryBelief::Response response= srv.response;
    int id;
    if(response.success==false){
      belief_manager_msgs::GenerateID::Request req;
      belief_manager_msgs::GenerateID::Response res;
      generate_id_client.call(req, res);
      if (res.ack)
      {
        id = res.id;
      }
      belief_manager_msgs::AddBelief srv2;
      std::stringstream s;
      s << "object(" << id << ", drone), name(" << id << ","<< message.sender <<")";
      srv2.request.belief_expression = s.str();
      srv2.request.multivalued = true;
      add_client.call(srv2);
      belief_manager_msgs::AddBelief::Response response= srv2.response;
   }else{
    std::string sub = response.substitutions;
    std::vector<std::string> pairs = getPairs(sub);
    std::vector<std::string> subs = getSubstitutions(pairs);
    id = std::stoi(subs[0]);
  }
  belief_manager_msgs::AddBelief::Request req;
  belief_manager_msgs::AddBelief::Response res;
  double val_x=message.position.x;
  val_x=val_x*100;
  val_x=std::round(val_x);
  val_x=val_x/100;  
  double val_y=message.position.y;
  val_y=val_y*100;
  val_y=std::round(val_y);
  val_y=val_y/100;  
  double val_z=message.position.z;
  val_z=val_z*100;
  val_z=std::round(val_z);
  val_z=val_z/100;  
  if(val_x>=-0.01 && val_x<=0.01){
    val_x=0;
  }
  if(val_y>=-0.01 && val_y<=0.01){
    val_y=0;
  }
  if(val_z>=-0.01 && val_z<=0.01){
    val_z=0;
  }
  std::stringstream ss;
  ss << "position(" << id << ", (" <<val_x<< ", "<< val_y<< ", " << val_z << "))";
  req.belief_expression = ss.str();
  req.multivalued = false;
  add_client.call(req, res);
  if( !(last_positions.find(id) == last_positions.end())){
    //calculate velocity
    int32_t d_time=abs(int(message.time-last_positions[id].second));
    if(d_time>0.2){
    double x_now = val_x;
    double x_previous = last_positions[id].first.x;
    double d_x= x_now-x_previous; 
    double vel_x = d_x/(double)d_time;
    double y_now = val_y;
    double y_previous = last_positions[id].first.y;
    double d_y= y_now-y_previous; 
    double vel_y = d_y/(double)d_time;
    double z_now = val_z;
    double z_previous = last_positions[id].first.z;
    double d_z= z_now-z_previous; 
    double vel_z = d_z/(double)d_time;
    geometry_msgs::Point shared_vel;
    shared_vel.x=vel_x;
    shared_vel.y=vel_y;
    shared_vel.z=vel_z;
    geometry_msgs::Point shared_position;
    shared_position.x=x_now;
    shared_position.y=y_now;
    shared_position.z=z_now;
    geometry_msgs::Point own_position;
    own_position.x=last_positions[my_id].first.x;
    own_position.y=last_positions[my_id].first.y;
    own_position.z=last_positions[my_id].first.z;
    geometry_msgs::Point own_vel=vel;
    //it is verified that there is collision between both vectors
    if(collision_detected(shared_position , shared_vel , own_position , own_vel)){
      double angle = get_angle(shared_vel, own_vel);
      if (angle>=160){
      belief_manager_msgs::QueryBelief srv;
      std::stringstream s;
      s << "frontal_collision_course(" << my_id << ", "<< id <<")";
      srv.request.query = s.str();
      query_client.call(srv);
      belief_manager_msgs::QueryBelief::Response response= srv.response;
      if(!response.success){
        std::cout<<"collision detected";
        belief_manager_msgs::AddBelief srv_collision;
        s.str(std::string());
        s << "frontal_collision_course(" << my_id << ", "<< id <<")";
        srv_collision.request.belief_expression = s.str();
        srv_collision.request.multivalued = false;
        add_client.call(srv_collision);
      }
      belief_manager_msgs::RemoveBelief::Request req;
      belief_manager_msgs::RemoveBelief::Response res;
      s.str(std::string());
      s << "collision_course(" << my_id << ", "<< id <<")";
      req.belief_expression = s.str();
      remove_client.call(req, res);
      }else{
      belief_manager_msgs::QueryBelief srv;
      std::stringstream s;
      s << "collision_course(" << my_id << ", "<< id <<")";
      srv.request.query = s.str();
      query_client.call(srv);
      belief_manager_msgs::QueryBelief::Response response= srv.response;
      if(!response.success){
        std::cout<<"collision detected";
        belief_manager_msgs::AddBelief srv_collision;
        s.str(std::string());
        s << "collision_course(" << my_id << ", "<< id <<")";
        srv_collision.request.belief_expression = s.str();
        srv_collision.request.multivalued = false;
        add_client.call(srv_collision);
      }
      belief_manager_msgs::RemoveBelief::Request req;
      belief_manager_msgs::RemoveBelief::Response res;
      s.str(std::string());
      s << "frontal_collision_course(" << my_id << ", "<< id <<")";
      req.belief_expression = s.str();
      remove_client.call(req, res);
      }
    }else{
      belief_manager_msgs::QueryBelief srv;
      std::stringstream s;
      s << "frontal_collision_course(" << my_id << ", "<< id <<")";
      srv.request.query = s.str();
      query_client.call(srv);
      belief_manager_msgs::QueryBelief::Response response1= srv.response;
      if(response1.success){
      belief_manager_msgs::RemoveBelief::Request req;
      belief_manager_msgs::RemoveBelief::Response res;
      s.str(std::string());
      s << "frontal_collision_course(" << my_id << ", "<< id <<")";
      req.belief_expression = s.str();
      remove_client.call(req, res);
      }
      s.str(std::string());
      s << "collision_course(" << my_id << ", "<< id <<")";
      srv.request.query = s.str();
      query_client.call(srv);
      belief_manager_msgs::QueryBelief::Response response2= srv.response;
      if(response2.success){
      belief_manager_msgs::RemoveBelief::Request req;
      belief_manager_msgs::RemoveBelief::Response res;
      s.str(std::string());
      s << "collision_course(" << my_id << ", "<< id <<")";
      req.belief_expression = s.str();
      remove_client.call(req, res);
      }
    }
    geometry_msgs::Point p;
    p.x=val_x;
    p.y=val_y;
    p.z=val_z;

    std::pair <geometry_msgs::Point, int32_t> pair (p, message.time);
    last_positions[id]=pair;
  }
}else{  
  //update last shared position 
  geometry_msgs::Point p;
  p.x=val_x;
  p.y=val_y;
  p.z=val_z;

  std::pair <geometry_msgs::Point, int32_t> pair (p, message.time);
  last_positions[id]=pair;
  }
}
}
SwarmBeliefUpdaterProcess::Point::Point(double x_coord, double y_coord, double z_coord) {
  x = x_coord;
  y = y_coord;
  z = z_coord;
}

SwarmBeliefUpdaterProcess::Point::Point() {
  x = 0;
  y = 0;
  z = 0;
}

double SwarmBeliefUpdaterProcess::Point::maxDifference(SwarmBeliefUpdaterProcess::Point p) {
  double x_diff = (x - p.x) > 0? x - p.x: p.x - x;
  double y_diff = (y - p.y) > 0? y - p.y: p.y - y;
  double z_diff = (z - p.z) > 0? z - p.z: p.z - z;
  double max = 0;
  for(double d: {x_diff, y_diff, z_diff}) {
    if(d > max) {
      max = d;
    }
  }
  return max;
}

void SwarmBeliefUpdaterProcess::Point::roundTo(double value) {
  x = std::round(x/value)*value;
  y = std::round(y/value)*value;
  z = std::round(z/value)*value;
}
