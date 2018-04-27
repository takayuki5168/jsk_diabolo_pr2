// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met: *
 *   * Redistributions of source code must retain the above copyright *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
 * sample_pcl.cpp
 * Author: lfurushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "std_msgs/Float64.h"


class SamplePCLNode
{
public:
    SamplePCLNode() : nh_(""), pnh_("~")
    {
        // initialization
        sub_ = pnh_.subscribe(
            "input" /* this subscribes '~input' topic */,
            1 /* queue size */,
            &SamplePCLNode::messageCallback, this);
        pub_pitch_ = pnh_.advertise<std_msgs::Float64>("diabolo/pitch", 1000);  //TODO
        pub_yaw_ = pnh_.advertise<std_msgs::Float64>("diabolo/yaw", 1000);      //TODO
    }

    void messageCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_sub)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg_sub, cloud);

        std_msgs::Float64 msg_pitch;
        std_msgs::Float64 msg_yaw;


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

                /*
                std::cout << "PO" << std::endl;
                for (pcl::PointCloud<pcl::PointXYZ>::iterator p = cloud.points.begin(); p != cloud.points.end(); *p++) {
                    if (p->x < 1.0 and p->x > 0.3 and p->y > -0.2 and p->y < 0.2 and p->z > 0.10 and p->z < 0.6) {
                        std::cout << p->x << " " << p->y << " " << p->z << std::endl;
                    }
                }
                std::cout << "PO" << std::endl;
            */
            }
        }

        // yaw傾き計算
        //double a = (cnt * sum_xy - sum_x * sum_y) / (cnt * sum_x2 - sum_x * sum_x);
        //double b = (sum_y - a * sum_x) / cnt;
        double yaw = std::atan2((cnt * sum_xy - sum_x * sum_y), (cnt * sum_x2 - sum_x * sum_x)) / 3.14 * 180;
        msg_yaw.data = yaw;
        pub_yaw_.publish(msg_yaw);

        double mid_x = (max_x + min_x) / 2.;

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
        msg_pitch.data = pitch;
        pub_pitch_.publish(msg_pitch);

        std::cout << "[yaw] " << yaw << " [pitch] " << pitch << std::endl;
        //std::cout << max_x_temae << " " << max_z_temae << " " << max_x_oku << " " << max_z_oku << " " << pitch << std::endl;
    }

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_;
    ros::Publisher pub_pitch_;
    ros::Publisher pub_yaw_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sample_pcl");

    SamplePCLNode n;

    ros::spin();

    return 0;
}
