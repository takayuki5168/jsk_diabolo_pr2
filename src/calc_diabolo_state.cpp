#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>


class CalcDiaboloStateNode
{
public:
  CalcDiaboloStateNode(int idx) : idx_(idx), nh_(""), pnh_("~")
  {
    // subscriber
    sub_ = pnh_.subscribe("input", 1, &CalcDiaboloStateNode::messageCallback, this);

    // publisher
    pub_pitch_ = pnh_.advertise<std_msgs::Float64>("pitch", 1);
    pub_yaw_ = pnh_.advertise<std_msgs::Float64>("yaw", 1);
    pub_points_ = pnh_.advertise<sensor_msgs::PointCloud2>("points", 1);
    pub_cube_ = pnh_.advertise<std_msgs::Float64MultiArray>("cube", 1);
    pub_mid_ = pnh_.advertise<std_msgs::Float64>("mid", 1);
    pub_pitch_points_ = pnh_.advertise<sensor_msgs::PointCloud2>("pitch_points", 1);
  }

  void messageCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_sub)
  {
    // translate ros msg to point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg_sub, cloud);

    // msgs
    std_msgs::Float64 msg_pitch, msg_yaw, msg_mid;
    std_msgs::Float64MultiArray cube;
    pcl::PointCloud<pcl::PointXYZ> points, pitch_points;
    sensor_msgs::PointCloud2 msg_points, msg_pitch_points;

    // calculate some value to use calculating pitch, yaw, ...
    double max_x = -1000;  // TODO
    double min_x = 1000;   // TODO
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    int cnt = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator p = cloud.points.begin(); p != cloud.points.end(); *p++) {
      if (p->x < 1.0 and p->x > 0.3 and p->y > -0.2 and p->y < 0.2 and p->z > 0.10 and p->z < 0.6) {
	max_x = (max_x > p->x) ? max_x : p->x;
	min_x = (min_x < p->x) ? min_x : p->x;

	sum_x += p->x;
	sum_y += p->y;
	sum_xy += p->x * p->y;
	sum_x2 += p->x * p->x;
	cnt++;

	if (idx_ == 1) {
	  points.push_back(pcl::PointXYZ(p->x, p->y, p->z));
	}
      }
    }
    if (idx_ == 1) { // to publish diabolo points and cube
      pcl::toROSMsg(points, msg_points);
      msg_points.header.frame_id = "/base_footprint";
      msg_points.header.stamp = ros::Time::now();
      pub_points_.publish(msg_points);

      cube.data.push_back(0.3);
      cube.data.push_back(1.0);
      cube.data.push_back(-0.2);
      cube.data.push_back(0.2);
      cube.data.push_back(0.10);
      cube.data.push_back(0.6);
      pub_cube_.publish(cube);
    }
    
    // calculate yaw and publish
    double yaw = std::atan2((cnt * sum_xy - sum_x * sum_y), (cnt * sum_x2 - sum_x * sum_x)) / 3.14 * 180;
    if (not std::isnan(yaw)) {
      msg_yaw.data = yaw;
      pub_yaw_.publish(msg_yaw);
    }

    // calculate middle x and publish
    double mid_x = (max_x + min_x) / 2.;
    msg_mid.data = mid_x;
    pub_mid_.publish(msg_mid);

    // calculate pitch points and publish
    double max_z_temae = 0;  // TODO
    double max_x_temae;      // TODO
    double max_z_oku = 0;    // TODO
    double max_x_oku;        // TODO
    for (pcl::PointCloud<pcl::PointXYZ>::iterator p = cloud.points.begin(); p != cloud.points.end(); *p++) {
      if (p->x < 1.0 and p->x > 0.3 and p->y > -0.2 and p->y < 0.2 and p->z > 0.10 and p->z < 0.6) {
	if (p->x > mid_x) {
	  if (max_z_oku < p->z) {
	    max_z_oku = p->z;
	    max_x_oku = p->x;
	  }
	} else {
	  if (max_z_temae < p->z) {
	    max_z_temae = p->z;
	    max_x_temae = p->x;
	  }
	}
      }
    }
    pitch_points.push_back(pcl::PointXYZ(max_x_oku, 0, max_z_oku));
    pitch_points.push_back(pcl::PointXYZ(max_x_temae, 0, max_z_temae));        
    pcl::toROSMsg(pitch_points, msg_pitch_points);
    msg_pitch_points.header.frame_id = "/base_footprint";
    msg_pitch_points.header.stamp = ros::Time::now();
    pub_pitch_points_.publish(msg_pitch_points);
    

    // calculate pitch and publish
    double pitch = std::atan2(max_z_oku - max_z_temae, max_x_oku - max_x_temae) / 3.14 * 180;
    if (not std::isnan(pitch)) {
      msg_pitch.data = pitch;
      pub_pitch_.publish(msg_pitch);
    }

    std::cout << "[yaw] " << yaw << " [pitch] " << pitch << std::endl;
  }

  int idx_;

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_pitch_, pub_yaw_, pub_points_, pub_cube_, pub_mid_, pub_pitch_points_;  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "calc_diabolo_state");

  // 1 is to publish diabolo points and cube
  // 0 is not to do above
  int idx = 1;

  CalcDiaboloStateNode n(idx);
  ros::spin();

  return 0;
}
