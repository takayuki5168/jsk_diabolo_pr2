#include <ros/ros.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class TossDiaboloMarkerNode
{
public:
  TossDiaboloMarkerNode() : nh_(""), pnh_("~"), r(30)
  {
    sub_cube_ = pnh_.subscribe("/calc_toss_diabolo_pos/cube", 1, &TossDiaboloMarkerNode::messageCallbackForCube, this);
    pub_cube_ = pnh_.advertise<visualization_msgs::Marker>("toss_diabolo_cube_marker", 1);

    // for cutting cube in which diabolo is
    marker_cube_.header.frame_id = "/base_footprint";
    marker_cube_.header.stamp = ros::Time::now();
    marker_cube_.ns = "toss_diabolo_cube_marker";
    marker_cube_.id = 1;
    marker_cube_.type = visualization_msgs::Marker::CUBE;
    marker_cube_.action = visualization_msgs::Marker::ADD;
    marker_cube_.scale.x = 1;
    marker_cube_.scale.y = 1;
    marker_cube_.scale.z = 1;
    marker_cube_.color.r = 1.0f;
    marker_cube_.color.g = 0.2f;
    marker_cube_.color.b = 0.0f;
    marker_cube_.color.a = 0.3;
  }

  void updateState()
  {
    // for cube
    marker_cube_.header.frame_id = "/base_footprint";
    marker_cube_.header.stamp = ros::Time::now();
    marker_cube_.pose.position.x = (cube_sx + cube_ex) / 2.0;
    marker_cube_.pose.position.y = (cube_sy + cube_ey) / 2.0;
    marker_cube_.pose.position.z = (cube_sz + cube_ez) / 2.0;
    marker_cube_.scale.x = std::abs(cube_sx - cube_ex);
    marker_cube_.scale.y = std::abs(cube_sy - cube_ey);
    marker_cube_.scale.z = std::abs(cube_sz - cube_ez);
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
    pub_cube_.publish(marker_cube_);
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
  ros::Subscriber sub_cube_;
  ros::Publisher pub_cube_;
  ros::Rate r;

  float cube_sx, cube_ex, cube_sy, cube_ey, cube_sz, cube_ez;

  visualization_msgs::Marker marker_cube_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "toss_diabolo_marker");
  TossDiaboloMarkerNode tdmn;

  tdmn.spin();
  return 0;
}
