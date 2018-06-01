#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

class CalcTossDiaboloPosNode
{
public:
  CalcTossDiaboloPosNode() : nh_(""), pnh_("~"), r_(30),
			     now_pos_x_(0), now_pos_y_(0), now_pos_z_(0),
			     past_pos_x_(0), past_pos_y_(0), past_pos_z_(0),
			     min_cube_x_(0.3), max_cube_x_(1.0),
			     min_cube_y_(-0.23), max_cube_y_(0.23),
			     min_cube_z_(0.1), max_cube_z_(2),
			     down_flag_(false)
  {
    // Subscriber
    sub_pointcloud_ = pnh_.subscribe("/tf_transform_cloud/output", 1, &CalcTossDiaboloPosNode::messageCallback, this);

    // Publisher
    pub_diabolo_points_ = pnh_.advertise<sensor_msgs::PointCloud2>("diabolo_points", 1);
    pub_diabolo_pos_ = pnh_.advertise<sensor_msgs::PointCloud2>("diabolo_pos", 1);
    pub_diabolo_pos_x_ = pnh_.advertise<std_msgs::Float64>("diabolo_pos_x", 1);
    pub_marker_cube_ = pnh_.advertise<visualization_msgs::Marker>("marker_cube", 1);    

    // Marker
    initMarker();
  }

private:
  void initMarker()
  {
    /*
     * marker cube
     */
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
  
  void messageCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_pointcloud)
  {
    // translate ros msg to pointcloud
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    pcl::fromROSMsg(*msg_pointcloud, pointcloud);

    // msgs
    std_msgs::Float64 msg_diabolo_pos_x;
    pcl::PointCloud<pcl::PointXYZ> diabolo_points, diabolo_pos;    
    sensor_msgs::PointCloud2 msg_diabolo_points, msg_diabolo_pos;

    // calculate some value to use calculating pitch, yaw, ...
    double sum_diabolo_x = 0, sum_diabolo_y = 0, sum_diabolo_z = 0;
    int diabolo_cnt = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator p = pointcloud.points.begin(); p != pointcloud.points.end(); *p++) {
      if (p->x < max_cube_x_ and p->x > min_cube_x_ and p->y > min_cube_y_ and p->y < max_cube_y_ and p->z > min_cube_z_ and p->z < max_cube_z_) {		
	sum_diabolo_x += p->x;
	sum_diabolo_y += p->y;
	sum_diabolo_z += p->z;
	diabolo_cnt++;

        diabolo_points.push_back(pcl::PointXYZ(p->x, p->y, p->z));
      }
    }
    
    /*
     * publish diabolo points
     */
    pcl::toROSMsg(diabolo_points, msg_diabolo_points);
    msg_diabolo_points.header.frame_id = "/base_footprint";
    msg_diabolo_points.header.stamp = ros::Time::now();
    pub_diabolo_points_.publish(msg_diabolo_points);


    /*
     * calculate diabolo pos, pos_x and publish
     */
    if (diabolo_cnt != 0) {
      msg_diabolo_pos_x.data = sum_diabolo_x / diabolo_cnt;
      pub_diabolo_pos_x_.publish(msg_diabolo_pos_x);
    
      diabolo_pos.clear();
      diabolo_pos.push_back(pcl::PointXYZ(sum_diabolo_x / diabolo_cnt, sum_diabolo_y / diabolo_cnt, sum_diabolo_z / diabolo_cnt));
      pcl::toROSMsg(diabolo_pos, msg_diabolo_pos);
      msg_diabolo_pos.header.frame_id = "/base_footprint";
      msg_diabolo_pos.header.stamp = ros::Time::now();
      pub_diabolo_pos_.publish(msg_diabolo_pos);
      
      //std::cout << "[pos] " << sum_diabolo_x / diabolo_cnt << " " << sum_diabolo_y / diabolo_cnt << " " << sum_diabolo_z / diabolo_cnt << std::endl;

      /*
       * predict falling diabolo pos
       */
      // calc now diabolo pos
      now_pos_x_ = sum_diabolo_x / diabolo_cnt;
      now_pos_y_ = sum_diabolo_y / diabolo_cnt;
      now_pos_z_ = sum_diabolo_z / diabolo_cnt;
      // judge whether diabolo is falling
      if (now_pos_z_ < past_pos_z_) {
	down_flag_ = true;
      } else {
	down_flag_ = false;
      }
      // if falling, predict when diabolo pos z is 1000 and publish
      if (down_flag_) {
	poses_x_.push_back(now_pos_x_);
	poses_y_.push_back(now_pos_y_);
	poses_z_.push_back(now_pos_z_);	
	if (poses_x_.size() >= 2) {
	  int s = poses_x_.size();
	  double p_x = (poses_x_.at(s - 2) - poses_x_.at(s - 1)) / (poses_y_.at(s - 2) - poses_y_.at(s - 1)) * (poses_z_.at(s - 1) - 1000 / 1000.) + poses_x_.at(s - 1);
  	  //msg_diabolo_pos_x.data = p_x;
          //pub_diabolo_pos_x_.publish(msg_diabolo_pos_x);
	  //std::cerr << "[predict] " << p_x << std::endl;
	  std::cout << "[pos] " << sum_diabolo_x / diabolo_cnt << " " << sum_diabolo_y / diabolo_cnt << " " << sum_diabolo_z / diabolo_cnt << std::endl;	  
	}
      }
      past_pos_x_ = now_pos_x_;
      past_pos_y_ = now_pos_y_;
      past_pos_z_ = now_pos_z_;            
    }

    /*
     * calculate marker cube and publish
     */
    marker_cube_.header.frame_id = "/base_footprint";
    marker_cube_.header.stamp = ros::Time::now();
    marker_cube_.pose.position.x = (max_cube_x_ + min_cube_x_) / 2.0;
    marker_cube_.pose.position.y = (max_cube_y_ + min_cube_y_) / 2.0;
    marker_cube_.pose.position.z = (max_cube_z_ + min_cube_z_) / 2.0;
    marker_cube_.scale.x = std::abs(max_cube_x_ - min_cube_x_);
    marker_cube_.scale.y = std::abs(max_cube_y_ - min_cube_y_);
    marker_cube_.scale.z = std::abs(max_cube_z_ - min_cube_z_);
    pub_marker_cube_.publish(marker_cube_);    
  }

  // ros params
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_pointcloud_;
  ros::Publisher pub_diabolo_points_, pub_diabolo_pos_, pub_diabolo_pos_x_;
  ros::Publisher pub_marker_cube_;
  visualization_msgs::Marker marker_cube_;  
  ros::Rate r_;

  // cube params
  double min_cube_x_, max_cube_x_;
  double min_cube_y_, max_cube_y_;
  double min_cube_z_, max_cube_z_;

  std::vector<double> poses_x_, poses_y_, poses_z_;
  double now_pos_x_, now_pos_y_, now_pos_z_;  
  double past_pos_x_, past_pos_y_, past_pos_z_;

  bool down_flag_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calc_toss_diabolo_pos");

  CalcTossDiaboloPosNode n;
  ros::spin();

  return 0;
}
