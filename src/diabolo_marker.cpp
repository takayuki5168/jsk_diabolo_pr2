#include <ros/ros.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class DiaboloMarkerNode
{
public:
  DiaboloMarkerNode() : nh_(""), pnh_("~"), pitch_(0), yaw_(0), r(1)
  {
    sub_pitch_ = pnh_.subscribe(
				"/calc_diabolo_state/pitch",
				1 /* queue size */,
				&DiaboloMarkerNode::messageCallbackForPitch, this);
    sub_yaw_ = pnh_.subscribe(
			      "/calc_diabolo_state/yaw",
			      1 /* queue size */,
			      &DiaboloMarkerNode::messageCallbackForYaw, this);
    sub_points_ = pnh_.subscribe(
			      "/calc_diabolo_state/points",
			      1 /* queue size */,
			      &DiaboloMarkerNode::messageCallbackForPoints, this);
    sub_cube_ = pnh_.subscribe(
			      "/calc_diabolo_state/cube",
			      1 /* queue size */,
			      &DiaboloMarkerNode::messageCallbackForCube, this);
    
    pub_marker_ = pnh_.advertise<visualization_msgs::Marker>("diabolo_marker", 1);
    pub_cube_ = pnh_.advertise<visualization_msgs::Marker>("diabolo_cube_marker", 1);    
    

    {
      marker_.header.frame_id = "/base_footprint";
      marker_.header.stamp = ros::Time::now();

      marker_.ns = "diabolo_marker";
      marker_.id = 0;

      marker_.type = visualization_msgs::Marker::CYLINDER;
      marker_.action = visualization_msgs::Marker::ADD;

      marker_.scale.x = 0.01;
      marker_.scale.y = 0.01;
      marker_.scale.z = 0.5;

      marker_.color.r = 0.0f;
      marker_.color.g = 1.0f;
      marker_.color.b = 0.0f;
      marker_.color.a = 1.0;

      marker_.lifetime = ros::Duration();
    }
    {
      marker_cube_.header.frame_id = "/base_footprint";
      marker_cube_.header.stamp = ros::Time::now();

      marker_cube_.ns = "diabolo_cube_marker";
      marker_cube_.id = 1;

      marker_cube_.type = visualization_msgs::Marker::CUBE;
      marker_cube_.action = visualization_msgs::Marker::ADD;

      marker_cube_.scale.x = 0.01;
      marker_cube_.scale.y = 0.01;
      marker_cube_.scale.z = 0.5;

      marker_cube_.color.r = 1.0f;
      marker_cube_.color.g = 0.2f;
      marker_cube_.color.b = 0.0f;
      marker_cube_.color.a = 0.3;

      marker_.lifetime = ros::Duration();
    }
  }

  void updateState()
  {
    marker_.header.frame_id = "/base_footprint";
    marker_.header.stamp = ros::Time::now();
    marker_.pose.position.x = center_x;
    marker_.pose.position.y = center_y;
    marker_.pose.position.z = center_z;

    marker_cube_.header.frame_id = "/base_footprint";
    marker_cube_.header.stamp = ros::Time::now();
    marker_cube_.pose.position.x = (cube_sx + cube_ex) / 2.0;
    marker_cube_.pose.position.y = (cube_sy + cube_ey) / 2.0;
    marker_cube_.pose.position.z = (cube_sz + cube_ez) / 2.0;
    marker_cube_.scale.x = std::abs(cube_sx - cube_ex);
    marker_cube_.scale.y = std::abs(cube_sy - cube_ey);
    marker_cube_.scale.z = std::abs(cube_sz - cube_ez);
    
    
    tf::Quaternion q_diabolo = tf::createQuaternionFromRPY(0, pitch_ * 3.14 / 180, yaw_ * 3.14 / 180);
    quaternionTFToMsg(q_diabolo, marker_.pose.orientation);
  }
  
  void spin()
  {
    while (ros::ok()) {
      updateState();
      publish();
    
      ros::spinOnce();    
      r.sleep();    
    }
  }
  
private:
  void publish()
  {
    pub_marker_.publish(marker_);
    pub_cube_.publish(marker_cube_);    
  }
  
  void messageCallbackForPitch(const std_msgs::Float64 pitch) { pitch_ = pitch.data; }
  void messageCallbackForYaw(const std_msgs::Float64 yaw) { yaw_ = yaw.data; }
  void messageCallbackForPoints(const sensor_msgs::PointCloud2::ConstPtr& msg_points)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg_points, cloud);
    center_x = 0, center_y = 0, center_z = 0;
    int cnt = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator p = cloud.points.begin(); p != cloud.points.end(); *p++) {
	center_x += p->x;
	center_y += p->y;
	center_z += p->z;
	cnt++;
    }
    center_x /= cnt;
    center_y /= cnt;
    center_z /= cnt;
  }
  void messageCallbackForCube(const std_msgs::Float64MultiArray cube)
  {
    cube_sx = cube.data[0];
    cube_ex = cube.data[1];    
    cube_sy = cube.data[2];
    cube_ey = cube.data[3];    
    cube_sz = cube.data[4];
    cube_ez = cube.data[5];    
  }
  
private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_pitch_, sub_yaw_, sub_points_, sub_cube_;
  ros::Publisher pub_marker_, pub_cube_;
  ros::Rate r;

  float pitch_;
  float yaw_;
  
  float center_x, center_y, center_z;
  
  float cube_sx, cube_ex, cube_sy, cube_ey, cube_sz, cube_ez;

  visualization_msgs::Marker marker_;
  visualization_msgs::Marker marker_cube_;    
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diabolo_marker");
  DiaboloMarkerNode dmn;



  dmn.spin();
  return 0;
}
