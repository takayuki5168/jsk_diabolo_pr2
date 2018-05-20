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
    // initialization
    sub_ = pnh_.subscribe(
			  "input" /* this subscribes '~input' topic */,
			  1 /* queue size */,
			  &CalcDiaboloStateNode::messageCallback, this);
    pub_pitch_ = pnh_.advertise<std_msgs::Float64>("pitch", 1000);           //TODO
    pub_yaw_ = pnh_.advertise<std_msgs::Float64>("yaw", 1000);               //TODO
    pub_points_ = pnh_.advertise<sensor_msgs::PointCloud2>("points", 1000);  //TODO
    pub_cube_ = pnh_.advertise<std_msgs::Float64MultiArray>("cube", 1000);   //TODO
    pub_mid_ = pnh_.advertise<std_msgs::Float64>("mid", 1000);               //TODO
    pub_pitch_points_ = pnh_.advertise<std_msgs::Float64MultiArray>("pitch_points", 1000);               //TODO	
  }

  void messageCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_sub)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg_sub, cloud);

    std_msgs::Float64 msg_pitch;
    std_msgs::Float64 msg_yaw;
    std_msgs::Float64 msg_mid;

    pcl::PointCloud<pcl::PointXYZ> points;
    std_msgs::Float64MultiArray msg_pitch_points;

    std_msgs::Float64MultiArray cube;
    sensor_msgs::PointCloud2 msg_points;

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
    if (idx_ == 1) {
      std::cout << points.size() << std::endl;
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
    // yaw傾き計算
    //double a = (cnt * sum_xy - sum_x * sum_y) / (cnt * sum_x2 - sum_x * sum_x);
    //double b = (sum_y - a * sum_x) / cnt;
    double yaw = std::atan2((cnt * sum_xy - sum_x * sum_y), (cnt * sum_x2 - sum_x * sum_x)) / 3.14 * 180;
    if (not std::isnan(yaw)) {
      msg_yaw.data = yaw;
      pub_yaw_.publish(msg_yaw);
    }

    double mid_x = (max_x + min_x) / 2.;
    msg_mid.data = mid_x;
    pub_mid_.publish(msg_mid);

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

    double pitch = std::atan2(max_z_oku - max_z_temae, max_x_oku - max_x_temae) / 3.14 * 180;
    msg_pitch_points.data.push_back(max_x_oku);
    msg_pitch_points.data.push_back(max_z_oku);
    msg_pitch_points.data.push_back(max_x_temae);
    msg_pitch_points.data.push_back(max_z_temae);
    pub_pitch_points_.publish(msg_pitch_points);
    if (not std::isnan(pitch)) {
      msg_pitch.data = pitch;
      pub_pitch_.publish(msg_pitch);
    }

    std::cout << "[yaw] " << yaw << " [pitch] " << pitch << std::endl;
  }

  int idx_;

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_pitch_;
  ros::Publisher pub_yaw_;
  ros::Publisher pub_points_;
  ros::Publisher pub_cube_;
  ros::Publisher pub_mid_;
  ros::Publisher pub_pitch_points_;  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "calc_diabolo_state");

  int idx = 0;
  if (argc > 1) {
    idx = atoi(argv[1]);
  }
  idx = 1;

  CalcDiaboloStateNode n(idx);

  ros::spin();

  return 0;
}
