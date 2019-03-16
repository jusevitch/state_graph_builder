#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <state_graph_builder/graph.h>
class Builder
{
  public:
  Builder();
  ~Builder();
  private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_;
  ros::Publisher pub;
  std_msgs::Bool switch_signal;
  ros::Subscriber switch_sub;

  int n;
  int id;// 1 for ugv 0 for uav

  ros::Subscriber uav_msgs;
  std::vector<ros::Subscriber> uav_subs;
  std::vector<geometry_msgs::Point> uav_list;

  std::vector<nav_msgs::Odometry> ugv_list1;
  std::vector<geometry_msgs::Twist> ugv_list2;
  std::vector<ros::Subscriber> ugv_sub1;
  std::vector<ros::Subscriber> ugv_sub2;

  void ugv1_subCallback(const nav_msgs::Odometry::ConstPtr& msgs, const int list_idx);
  void ugv2_subCallback(const geometry_msgs::Twist::ConstPtr& msgs, const int list_idx);
  void uav_subCallback(const geometry_msgs::PointStamped::ConstPtr& msgs, const int list_idx);
  void switch_subCallback(const std_msgs::Bool::ConstPtr& msg);
  state_graph_builder::graph msg;

};
void Builder::ugv1_subCallback(const nav_msgs::Odometry::ConstPtr& msgs, const int list_idx){
  
  
}
void Builder::uav_subCallback(const geometry_msgs::PointStamped::ConstPtr& msgs, const int list_idx){
  uav_list[list_idx].x = msgs->point.x;
  uav_list[list_idx].y = msgs->point.y;
  uav_list[list_idx].z = msgs->point.z;  
}
void Builder::switch_subCallback(const std_msgs::Bool::ConstPtr& msg){
  switch_signal.data = msg->data;
}

Builder::Builder()
  :nh_private_("~")
{
  
     nh_private_.param<int>("n", n, 15);
     uav_list.resize(n);
     int id = 0;

     
     pub=nh.advertise<geometry_msgs::Point>("graph", 100);
     switch_sub = nh.subscribe("/switch", 10, &Builder::switch_subCallback, this);

     while(switch_signal.data==false){
       ros::Duration(0.1).sleep();
     }
     for (int i=1; i < n+1; i++){
        std::string sub_topic = "/uav" + std::to_string(i) + "/ground_truth/position";
        uav_subs.push_back( nh.subscribe<geometry_msgs::PointStamped>(sub_topic, 10, boost::bind(&Builder::uav_subCallback,this,_1, i)) );
     }

     ros::Rate rate(10);
     uint time_count=0;

       while(ros::ok()) {
	 msg.points.clear();
	 msg.time=time_count;
	 for (int i=0; i<n; i++){
	   msg.points.push_back(uav_list[i]);
	   msg.id.push_back(id);
	 }

	 pub.publish(msg);
          //Delays untill it is time to send another message
	  time_count+=1;
          rate.sleep();
       }
    
}

Builder::~Builder()
{
  ros::shutdown();
}

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "builder");

     Builder builder;

     ros::spin();
     return 0;
     
}
