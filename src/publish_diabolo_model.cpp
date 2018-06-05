#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include "tf/transform_broadcaster.h"

class PublishDiaboloModelNode
{
public:
  PublishDiaboloModelNode() : nh_(""), pnh_("~"), r_(30), pitch_(0), yaw_(0)
  {
    // Subscriber
    sub_diabolo_state_ = pnh_.subscribe("/calc_idle_diabolo_state/diabolo_state", 1, &PublishDiaboloModelNode::messageCallback, this);

    // Publisher
    pub_marker_diabolo_model_ = pnh_.advertise<visualization_msgs::Marker>("diabolo_model", 1);

    // Marker
    initMarker();
  }

  void execute()
  {
    while (true) {
      ros::spinOnce();
      publishMarker();
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
  }

  void publishMarker()
  {
    marker_diabolo_model_.header.frame_id = "/base_footprint";
    marker_diabolo_model_.header.stamp = ros::Time::now();
    marker_diabolo_model_.pose.position.x = 0.7;
    marker_diabolo_model_.pose.position.y = 0;
    marker_diabolo_model_.pose.position.z = 0.21;
    tf::Quaternion q_diabolo = tf::createQuaternionFromRPY(0, -pitch_ * 3.14 / 180 + 1.57, yaw_ * 3.14 / 180);
    quaternionTFToMsg(q_diabolo, marker_diabolo_model_.pose.orientation);
    
    pub_marker_diabolo_model_.publish(marker_diabolo_model_);
  }
  
  void messageCallback(const std_msgs::Float64MultiArray diabolo_state)
  {
    pitch_ = diabolo_state.data[0];
    yaw_ = diabolo_state.data[1];
  }

  // ros params
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_diabolo_state_;
  ros::Publisher pub_marker_diabolo_model_; // Marker
  visualization_msgs::Marker marker_diabolo_model_;
  ros::Rate r_;

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
