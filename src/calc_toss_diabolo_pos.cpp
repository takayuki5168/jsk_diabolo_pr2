#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>

class CalcTossDiaboloPosNode
{
public:
  CalcTossDiaboloPosNode() : nh_(""), pnh_("~"), r(30)
  {
    // subscriber
    sub_ = pnh_.subscribe("/tf_transform_cloud/output", 1, &CalcTossDiaboloPosNode::messageCallback, this);

    // publisher
    pub_points_ = pnh_.advertise<sensor_msgs::PointCloud2>("points", 1);
    pub_cube_ = pnh_.advertise<std_msgs::Float64MultiArray>("cube", 1);
    pub_pos_ = pnh_.advertise<sensor_msgs::PointCloud2>("pos", 1);
    pub_pos_float_ = pnh_.advertise<std_msgs::Float64MultiArray>("pos_float", 1);    
  }

private:  
  void messageCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_sub)
  {
    // translate ros msg to point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg_sub, cloud);

    // msgs
    std_msgs::Float64MultiArray cube, pos_float;
    pcl::PointCloud<pcl::PointXYZ> points, pos;
    sensor_msgs::PointCloud2 msg_points, msg_pos;

    // calculate some value to use calculating pitch, yaw, ...
    double min_x = 0, max_x = 1.0;
    double min_y = -0.4, max_y = 0.3;
    double min_z = 0.1, max_z = 2;
    double sum_x = 0, sum_y = 0, sum_z = 0;
    int cnt = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator p = cloud.points.begin(); p != cloud.points.end(); *p++) {
      if (p->x < max_x and p->x > min_x and p->y > min_y and p->y < max_y and p->z > min_z and p->z < max_z) {
	sum_x += p->x;
	sum_y += p->y;
	sum_z += p->z;
	cnt++;

        points.push_back(pcl::PointXYZ(p->x, p->y, p->z));
      }
    }
    
    // publish diabolo points
    pcl::toROSMsg(points, msg_points);
    msg_points.header.frame_id = "/base_footprint";
    msg_points.header.stamp = ros::Time::now();
    pub_points_.publish(msg_points);

    // calculate and publish cube
    cube.data.push_back(min_x);
    cube.data.push_back(max_x);
    cube.data.push_back(min_y);
    cube.data.push_back(max_y);
    cube.data.push_back(min_z);
    cube.data.push_back(max_z);
    pub_cube_.publish(cube);

    // calculate and publish pos
    if (cnt != 0) {
      pos.push_back(pcl::PointXYZ(sum_x / cnt, sum_y / cnt, sum_z / cnt));
      pcl::toROSMsg(pos, msg_pos);
      msg_pos.header.frame_id = "/base_footprint";
      msg_pos.header.stamp = ros::Time::now();
      pub_pos_.publish(msg_pos);
      
      pos_float.data.push_back(sum_x / cnt);
      pos_float.data.push_back(sum_y / cnt);
      pos_float.data.push_back(sum_z / cnt);
      pub_pos_float_.publish(pos_float);
      
      std::cout << "[pos] " << sum_x / cnt << " " << sum_y / cnt << " " << sum_z / cnt << std::endl;
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_pos_, pub_pos_float_, pub_points_, pub_cube_;
  ros::Rate r;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calc_toss_diabolo_pos");

  CalcTossDiaboloPosNode n;
  ros::spin();

  return 0;
}
