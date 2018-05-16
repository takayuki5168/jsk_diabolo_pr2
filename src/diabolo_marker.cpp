#include <ros/ros.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <iostream>


class DiaboloMarkerNode
{
public:
  DiaboloMarkerNode() : nh_(""), pnh_("~"), pitch_(0), yaw_(0), r(1)
  {
    sub_pitch_ = pnh_.subscribe(
				"sample_pcl/diabolo_pitch",
				1 /* queue size */,
				&DiaboloMarkerNode::messageCallbackForPitch, this);
    sub_yaw_ = pnh_.subscribe(
			      "sample_pcl/diabolo_yaw",
			      1 /* queue size */,
			      &DiaboloMarkerNode::messageCallbackForYaw, this);
    pub_ = pnh_.advertise<visualization_msgs::Marker>("diabolo_marker", 1);

    marker_.header.frame_id = "/base_footprint";
    marker_.header.stamp = ros::Time::now();

    marker_.ns = "diabolo_marker";
    marker_.id = 0;

    marker_.type = visualization_msgs::Marker::CYLINDER;
    marker_.action = visualization_msgs::Marker::ADD;

    marker_.pose.position.x = 0;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    marker_.scale.x = 1.0;
    marker_.scale.y = 1.0;
    marker_.scale.z = 1.0;

    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 1.0;

    marker_.lifetime = ros::Duration();
  }
  
  void spin()
  {
    while (ros::ok()) {
      publish();
    
      ros::spinOnce();    
      r.sleep();    
    }
  }
  
private:
  void publish() { pub_.publish(marker_); }
  
  void messageCallbackForPitch(const std_msgs::Float64 pitch) { pitch_ = pitch.data; }
  void messageCallbackForYaw(const std_msgs::Float64 yaw) { yaw_ = yaw.data; }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_pitch_, sub_yaw_;
  ros::Publisher pub_;
  ros::Rate r;    

  float pitch_;
  float yaw_;

  visualization_msgs::Marker marker_;  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diabolo_marker");
  DiaboloMarkerNode dmn;

  dmn.spin();
  return 0;
}
