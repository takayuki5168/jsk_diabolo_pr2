#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
//#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>

class PublishDiaboloModelNode
{
public:
  PublishDiaboloModelNode() : nh_(""), pnh_("~"), r_(30), pitch_(0), yaw_(0)
  {
    // Subscriber
    sub_diabolo_state_ = pnh_.subscribe("/diabolo_system/diabolo_state", 1, &PublishDiaboloModelNode::messageCallbackForDiaboloState, this);
    //sub_diabolo_arm_ = pnh_.subscribe("/diabolo_system/diabolo_state", 1, &PublishDiaboloModelNode::messageCallback, this);        

    // Publisher
    pub_marker_diabolo_model_ = pnh_.advertise<visualization_msgs::Marker>("diabolo_model", 1);
    pub_marker_string_right_ = pnh_.advertise<visualization_msgs::Marker>("diabolo_string_right", 1);
    pub_marker_string_left_ = pnh_.advertise<visualization_msgs::Marker>("diabolo_string_left", 1);

    // Marker
    initMarker();
  }

  void execute() {
    while (true) {
      try {
	tfl_right_.lookupTransform("/base_footprint", "/r_gripper_r_finger_tip_link", ros::Time(0), tf_right_);
	tfl_left_.lookupTransform("/base_footprint", "/l_gripper_l_finger_tip_link", ros::Time(0), tf_left_);
      }
      catch (tf::TransformException ex){
	//ROS_ERROR("%s",ex.what());
      }
      std::cout << tf_right_.getOrigin().x() << " " << tf_right_.getOrigin().y() << " " << tf_right_.getOrigin().z() << std::endl;
      std::cout << tf_left_.getOrigin().x() << " " << tf_left_.getOrigin().y() << " " << tf_left_.getOrigin().z() << std::endl;      
      
      ros::spinOnce();
    }
  }
  
private:
  void initMarker() {
    /*
     * marker diabolo model
     */
    marker_diabolo_model_.header.frame_id = "/base_footprint";
    marker_diabolo_model_.header.stamp = ros::Time::now();
    marker_diabolo_model_.ns = "diabolo_model_marker";
    marker_diabolo_model_.id = 0;
    marker_diabolo_model_.type = visualization_msgs::Marker::MESH_RESOURCE;    
    marker_diabolo_model_.action = visualization_msgs::Marker::ADD;
    marker_diabolo_model_.mesh_resource = "package://jsk_diabolo_pr2/meshes/nil_link_mesh.dae";    
    marker_diabolo_model_.scale.x = 1;
    marker_diabolo_model_.scale.y = 1;
    marker_diabolo_model_.scale.z = 1;
    marker_diabolo_model_.color.r = 0.0f;
    marker_diabolo_model_.color.g = 0.8f;
    marker_diabolo_model_.color.b = 0.7f;
    marker_diabolo_model_.color.a = 0.8;
    marker_diabolo_model_.lifetime = ros::Duration();

    /*
     * marker string right
     */
    marker_string_right_.header.frame_id = "/base_footprint";
    marker_string_right_.header.stamp = ros::Time::now();
    marker_string_right_.ns = "diabolo_string_right";
    marker_string_right_.id = 0;
    marker_string_right_.type = visualization_msgs::Marker::LINE_STRIP;    
    marker_string_right_.action = visualization_msgs::Marker::ADD;
    marker_string_right_.scale.x = 10;
    marker_string_right_.scale.y = 10;
    marker_string_right_.scale.z = 10;    
    marker_string_right_.pose.orientation.w = 1.0;
    marker_string_right_.color.r = 1.0f;
    marker_string_right_.color.g = 1.0f;
    marker_string_right_.color.b = 1.0f;
    marker_string_right_.lifetime = ros::Duration();

    /*
     * marker string left
     */
    marker_string_left_.header.frame_id = "/base_footprint";
    marker_string_left_.header.stamp = ros::Time::now();
    marker_string_left_.ns = "diabolo_string_left";
    marker_string_left_.id = 0;
    marker_string_left_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_string_left_.action = visualization_msgs::Marker::ADD;
    marker_string_left_.scale.x = 10;
    marker_string_left_.scale.y = 10;
    marker_string_left_.scale.z = 10;    
    marker_string_left_.pose.orientation.w = 1.0;    
    marker_string_left_.color.r = 0.0f;
    marker_string_left_.color.g = 1.0f;
    marker_string_left_.color.b = 1.0f;
    marker_string_left_.lifetime = ros::Duration();
  }

  void publishMarker()
  {
    /*
     * marker diabolo model
     */
    marker_diabolo_model_.header.frame_id = "/base_footprint";
    marker_diabolo_model_.header.stamp = ros::Time::now();
    marker_diabolo_model_.pose.position.x = 0.7;
    marker_diabolo_model_.pose.position.y = 0;
    marker_diabolo_model_.pose.position.z = 0.21;
    tf::Quaternion q_diabolo = tf::createQuaternionFromRPY(0, -pitch_ * 3.14 / 180 + 1.57, yaw_ * 3.14 / 180);
    quaternionTFToMsg(q_diabolo, marker_diabolo_model_.pose.orientation);
    pub_marker_diabolo_model_.publish(marker_diabolo_model_);

    geometry_msgs::Point p;
    
    /*
     * marker string right
     */
    marker_string_right_.header.frame_id = "/base_footprint";
    marker_string_right_.header.stamp = ros::Time::now();
    marker_string_right_.points.clear();
    marker_string_right_.pose.orientation.x = 0.0;
    marker_string_right_.pose.orientation.y = 0.0;
    marker_string_right_.pose.orientation.z = 0.0;
    marker_string_right_.pose.orientation.w = 1.0;
    marker_string_right_.pose.position.x = 0.0;
    marker_string_right_.pose.position.y = 0.0;
    marker_string_right_.pose.position.z = 0.0;
    p.x = 100;//tf_right_.getOrigin().x();
    p.y = 100;//tf_right_.getOrigin().y();
    p.z = 100;//tf_right_.getOrigin().z();     
    marker_string_right_.points.push_back(p);
    p.x = 0.7;
    p.y = 0;
    p.z = 0.21;
    marker_string_right_.points.push_back(p);
    p.x = 100;
    p.y = 0;
    p.z = 0;
    marker_string_right_.points.push_back(p);    
    pub_marker_string_right_.publish(marker_string_right_);

    /*
     * marker string left
     */
    marker_string_left_.header.frame_id = "/base_footprint";
    marker_string_left_.header.stamp = ros::Time::now();
    marker_string_left_.points.clear();    
    p.x = tf_left_.getOrigin().x();
    p.y = tf_left_.getOrigin().y();
    p.z = tf_left_.getOrigin().z();     
    marker_string_left_.points.push_back(p);
    p.x = 0.7;
    p.y = 0;
    p.z = 0.21;
    marker_string_left_.points.push_back(p);
    p.x = 100;
    p.y = 0;
    p.z = 0;
    marker_string_left_.points.push_back(p);    
    pub_marker_string_left_.publish(marker_string_left_);
  }
  
  void messageCallbackForDiaboloState(const std_msgs::Float64MultiArray diabolo_state)
  {
    pitch_ = diabolo_state.data[0];
    yaw_ = diabolo_state.data[1];
    
    publishMarker();
  }

  // ros params
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_diabolo_state_;
  ros::Publisher pub_marker_diabolo_model_, pub_marker_string_right_, pub_marker_string_left_;   // Marker
  visualization_msgs::Marker marker_diabolo_model_, marker_string_right_, marker_string_left_;
  ros::Rate r_;
  tf::TransformListener tfl_right_, tfl_left_;
  tf::StampedTransform tf_right_, tf_left_;  

  // diabolo state
  double pitch_, yaw_;  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_diabolo_model");

  PublishDiaboloModelNode n;
  n.execute();

  return 0;
}
