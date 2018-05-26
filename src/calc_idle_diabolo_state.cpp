#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>


class CalcIdleDiaboloStateNode
{
public:
  CalcIdleDiaboloStateNode() : nh_(""), pnh_("~"), r(30)
  {
    // subscriber
    sub_ = pnh_.subscribe("/tf_transform_cloud/output", 1, &CalcIdleDiaboloStateNode::messageCallback, this);

    // publisher
    pub_diabolo_state_ = pnh_.advertise<std_msgs::Float64MultiArray>("diabolo_state", 1);
    pub_points_ = pnh_.advertise<sensor_msgs::PointCloud2>("points", 1);
    pub_cube_ = pnh_.advertise<std_msgs::Float64MultiArray>("cube", 1);
    pub_mid_ = pnh_.advertise<std_msgs::Float64>("mid", 1);
    pub_pitch_points_ = pnh_.advertise<sensor_msgs::PointCloud2>("pitch_points", 1);
  }

  void messageCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_sub)
  {
    std::cout << "PO" << std::endl;
    // translate ros msg to point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg_sub, cloud);

    // msgs
    std_msgs::Float64MultiArray msg_diabolo_state;
    std_msgs::Float64 msg_mid;
    std_msgs::Float64MultiArray cube;
    pcl::PointCloud<pcl::PointXYZ> points, pitch_points;
    sensor_msgs::PointCloud2 msg_points, msg_pitch_points;

    // calculate some value to use calculating pitch, yaw, ...
    double min_x = 0.3, max_x = 1.0;
    double min_y = -0.2, max_y = 0.2;
    double min_z = 0.1, max_z = 0.6;
    double max_diabolo_x = -1000;  // TODO
    double min_diabolo_x = 1000;   // TODO
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    int cnt = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator p = cloud.points.begin(); p != cloud.points.end(); *p++) {
      if (p->x < max_x and p->x > min_x and p->y > min_y and p->y < max_y and p->z > min_z and p->z < max_z) {	
	max_diabolo_x = (max_diabolo_x > p->x) ? max_diabolo_x : p->x;
	min_diabolo_x = (min_diabolo_x < p->x) ? min_diabolo_x : p->x;

	sum_x += p->x;
	sum_y += p->y;
	sum_xy += p->x * p->y;
	sum_x2 += p->x * p->x;
	cnt++;

	points.push_back(pcl::PointXYZ(p->x, p->y, p->z));
      }
    }
    
    // publish diabolo points
    pcl::toROSMsg(points, msg_points);
    msg_points.header.frame_id = "/base_footprint";
    msg_points.header.stamp = ros::Time::now();
    pub_points_.publish(msg_points);
    

    // calculate cube
    cube.data.push_back(min_x);
    cube.data.push_back(max_x);
    cube.data.push_back(min_y);
    cube.data.push_back(max_y);
    cube.data.push_back(min_z);
    cube.data.push_back(max_z);
    pub_cube_.publish(cube);
    
    // calculate yaw and publish
    double tmp_yaw = std::atan2((cnt * sum_xy - sum_x * sum_y), (cnt * sum_x2 - sum_x * sum_x)) / 3.14 * 180;
    if (not std::isnan(tmp_yaw)) {
      yaw = tmp_yaw;
    }
    msg_diabolo_state.data.push_back(yaw);

    
    // calculate middle x and publish
    double mid_diabolo_x = (max_diabolo_x + min_diabolo_x) / 2.;
    msg_mid.data = mid_diabolo_x;
    pub_mid_.publish(msg_mid);

    // calculate pitch points and publish
    double max_z_temae = 0;  // TODO
    double max_x_temae;      // TODO
    double max_z_oku = 0;    // TODO
    double max_x_oku;        // TODO
    for (pcl::PointCloud<pcl::PointXYZ>::iterator p = cloud.points.begin(); p != cloud.points.end(); *p++) {
      if (p->x < 1.0 and p->x > 0.3 and p->y > -0.2 and p->y < 0.2 and p->z > 0.10 and p->z < 0.6) {
	if (p->x > mid_diabolo_x) {
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
    double tmp_pitch = std::atan2(max_z_oku - max_z_temae, max_x_oku - max_x_temae) / 3.14 * 180;
    if (not std::isnan(tmp_pitch)) {
      pitch = tmp_pitch;
    }
    msg_diabolo_state.data.push_back(pitch);
    pub_diabolo_state_.publish(msg_diabolo_state);    

    std::cout << "[yaw] " << yaw << " [pitch] " << pitch << std::endl;
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_diabolo_state_, pub_points_, pub_cube_, pub_mid_, pub_pitch_points_;
  ros::Rate r;

  double pitch = 0, yaw = 0;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "calc_idle_diabolo_state");

  CalcIdleDiaboloStateNode n;
  ros::spin();

  return 0;
}
