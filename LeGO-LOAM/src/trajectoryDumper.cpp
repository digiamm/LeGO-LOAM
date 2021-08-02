#include "Eigen/Geometry"
#include "lib/tf_helpers.h"
#include "srrg_geometry/geometry2d.h"
#include "srrg_geometry/geometry3d.h"
#include "srrg_pcl/point_types.h"
#include "tf/transform_listener.h"
#include "tf2_msgs/TFMessage.h"
#include <geometry_msgs/Twist.h>
#include <iomanip>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace srrg2_core;

std::shared_ptr<tf::TransformListener> _listener;

std::string _map_frame_id = "map";
std::string _base_link_frame_id = "base_link";
std::string _point_cloud_topic = "/os1_cloud_node/points";
std::string _odom_topic = "/camera/odom/sample";

double _time_lag = 1.;
std::map<double, sensor_msgs::PointCloud2> _laser_msgs;
std::map<double, nav_msgs::Odometry> _odom_msgs;

double _last_laser_update_time = 0.;
double _last_odom_update_time = 0.;

std::string _odom_filename = "odom_dumped.txt";
std::string _laser_filename = "legoloam_dumped.txt";

void laserCallback(const sensor_msgs::PointCloud2 &scan_) {
  _laser_msgs.insert(std::make_pair(scan_.header.stamp.toSec(), scan_));
  _last_laser_update_time =
      std::max(_last_laser_update_time, scan_.header.stamp.toSec());
}

void odomCallback(const nav_msgs::Odometry &odom_) {
  _odom_msgs.insert(std::make_pair(odom_.header.stamp.toSec(), odom_));
  _last_odom_update_time =
      std::max(_last_odom_update_time, odom_.header.stamp.toSec());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_dumper");
  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");
  _nh.getParam("map_frame_id", _map_frame_id);
  _nh.getParam("base_link_frame_id", _base_link_frame_id);
  _nh.getParam("laser_topic", _point_cloud_topic);
  _nh.getParam("time_lag", _time_lag);
  _nh.getParam("laser_filename", _laser_filename);
  _nh.getParam("odom_filename", _odom_filename);

  std::cerr << argv[0] << ": running with these parameters" << std::endl;
  std::cerr << "_map_frame_id:=" << _map_frame_id << std::endl;
  std::cerr << "_base_link_frame_id:=" << _base_link_frame_id << std::endl;
  std::cerr << "_point_cloud_topic:=" << _point_cloud_topic << std::endl;
  std::cerr << "_odom_topic:=" << _odom_topic << std::endl;
  std::cerr << "_laser_filename:=" << _laser_filename << std::endl;
  std::cerr << "_odom_filename:=" << _odom_filename << std::endl;
  std::cerr << "_time_lag:=" << _time_lag << std::endl;

  ros::Subscriber odom_subscriber = nh.subscribe(_odom_topic, 10, odomCallback);
  ros::Subscriber laser_subscriber =
      nh.subscribe(_point_cloud_topic, 10, laserCallback);
  ros::Rate loop_rate(50);
  ros::Duration tfCacheDuration;
  tfCacheDuration = tfCacheDuration.fromSec(600);
  _listener.reset(new tf::TransformListener(tfCacheDuration));

  std::ofstream laser_stream(_laser_filename);
  laser_stream << "# timestamp tx ty tz qx qy qz qw" << std::endl;
  laser_stream << std::setprecision(9);
  laser_stream << std::fixed;

  std::ofstream odom_stream(_odom_filename);
  odom_stream << "# timestamp tx ty tz qx qy qz qw" << std::endl;
  odom_stream << std::setprecision(9);
  odom_stream << std::fixed;

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();

    while (!_laser_msgs.empty()) {
      const auto &scan = _laser_msgs.begin()->second;
      if (_last_laser_update_time - scan.header.stamp.toSec() < _time_lag) {
        std::cerr << "l";
        break;
      }
      if (_listener->canTransform(_map_frame_id, scan.header.frame_id,
                                  scan.header.stamp)) {
        Eigen::Isometry3f laser_pose;
        getTfTransform(laser_pose, *_listener, _map_frame_id,
                       scan.header.frame_id, scan.header.stamp);
        std::cerr << "L";
        // ldg q rep as <qw qx qy qz>
        const srrg2_core::Vector7f laser_pose_vec =
            geometry3d::t2tnqw(laser_pose);
        const float &tx = laser_pose_vec[0];
        const float &ty = laser_pose_vec[1];
        const float &tz = laser_pose_vec[2];
        const float &qw = laser_pose_vec[3];
        const float &qx = laser_pose_vec[4];
        const float &qy = laser_pose_vec[5];
        const float &qz = laser_pose_vec[6];
        laser_stream << scan.header.stamp.toSec() << " " << tx << " " << ty
                     << " " << tz << " " << qx << " " << qy << " " << qz << " "
                     << qw << std::endl;
      }
      _laser_msgs.erase(_laser_msgs.begin());
    }

    while (!_odom_msgs.empty()) {
      const nav_msgs::Odometry &odom = _odom_msgs.begin()->second;
      if (_last_odom_update_time - odom.header.stamp.toSec() < _time_lag) {
        std::cerr << "o";
        break;
      }
      std::cerr << "O";
      const float &tx = odom.pose.pose.position.x;
      const float &ty = odom.pose.pose.position.y;
      const float &tz = odom.pose.pose.position.z;
      const float &qw = odom.pose.pose.orientation.w;
      const float &qx = odom.pose.pose.orientation.x;
      const float &qy = odom.pose.pose.orientation.y;
      const float &qz = odom.pose.pose.orientation.z;
      odom_stream << odom.header.stamp.toSec() << " " << tx << " " << ty << " "
                  << tz << " " << qx << " " << qy << " " << qz << " " << qw
                  << std::endl;
      _odom_msgs.erase(_odom_msgs.begin());
    }
  }
}
