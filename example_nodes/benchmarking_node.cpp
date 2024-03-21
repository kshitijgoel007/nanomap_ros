#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>

#include <nanomap_ros/nanomap.h>
#include <nanomap_ros/nanomap_visualizer.h>
#include <nanomap_ros/nanomap_types.h>
#include <nanomap_ros/stopwatch.h>

#include <parameter_utils/ParameterUtils.h>

using namespace std;
namespace pu = parameter_utils;

class BenchmarkingNode
{
public:
  BenchmarkingNode() : nh("~")
  {
    // Get values from param server
    pu::get("max_range", max_range, (float)20.0);
    pu::get("num_depth_image_history", num_depth_image_history, 150);

    std::cout << "Got from parameter server:\n"
      << "max_range: " << max_range  << "\n"
      << "num_depth_image_history: " << num_depth_image_history << "\n"
      << std::endl;

    nanomap_visualizer.Initialize(nh);

    nanomap.SetSensorRange(max_range);
    nanomap.SetNumDepthImageHistory(num_depth_image_history);
    Matrix3 body_to_rdf;
    body_to_rdf << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    nanomap.SetBodyToRdf(body_to_rdf);

    datafile.open("nanomap_data_1000.txt");
    datafile << "point_cloud_count, sequence_number, time(ms), insertion_time(ms), sample_time(ms)" << std::endl;

    tf_buffer = new tf2_ros::Buffer();
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    // Subscribers
    pcl_sub = nh.subscribe("points", 100, &BenchmarkingNode::PointCloudCallback, this);
    pose_updates_sub = nh.subscribe("path", 100, &BenchmarkingNode::SmoothedPosesCallback, this);
    odom_sub = nh.subscribe("odometry", 100, &BenchmarkingNode::OdometryCallback, this);
    camera_info_sub = nh.subscribe("/camera/camera_info", 100, &BenchmarkingNode::CameraInfoCallback, this);

  };
  ~BenchmarkingNode() {
    datafile.close();
  }

  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Subscriber pose_updates_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber camera_info_sub;
  ros::Publisher query_points_pub;

  // For benchmarking
  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;
  std::ofstream datafile;
  Stopwatch global_time;
  int point_cloud_count = 0;

  NanoMap nanomap;
  NanoMapVisualizer nanomap_visualizer;

  float max_range;
  int num_depth_image_history;

  bool got_camera_info = false;

  void CameraInfoCallback(const sensor_msgs::CameraInfo &msg)
  {
    if (got_camera_info)
      return;
    std::cout << "Got camera info: \n"
      << "width: " << msg.width << "\n"
      << "height: " << msg.height << "\n"
      << "binning: " << msg.binning_x << "\n"
      << "fx: " << msg.K[0] << "\n"
      << "fy: " << msg.K[4] << "\n"
      << "cx: " << msg.K[2] << "\n"
      << "cy: " << msg.K[5] << "\n"
      << std::endl;

		Matrix3 K_camera_info;
		K_camera_info << msg.K[0], msg.K[1], msg.K[2], msg.K[3], msg.K[4], msg.K[5], 
                     msg.K[6], msg.K[7], msg.K[8];
    nanomap.SetCameraInfo(msg.binning_x, msg.width, msg.height, K_camera_info);
    nanomap_visualizer.SetCameraInfo(max_range, msg.width, msg.height, K_camera_info);
    got_camera_info = true;

    if (NANOMAP_DEBUG_PRINT)
      std::cout << "Initialized camera parameters" << std::endl;
  }

  void PointCloudCallback(const sensor_msgs::PointCloud2 &msg)
  {
    // if (NANOMAP_DEBUG_PRINT)
      std::cout << "In PointCloudCallback" << std::endl;
    if (!got_camera_info) {
      if (NANOMAP_DEBUG_PRINT)
        std::cout << "## Not initialized" << std::endl;
      return;
    }

    Stopwatch sw;
    sw.Start();

    geometry_msgs::TransformStamped transform_world_sensor;
    try {
      transform_world_sensor = tf_buffer->lookupTransform("world", msg.header.frame_id, msg.header.stamp);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

    pcl::PCLPointCloud2 cloud2_rdf;
    pcl_conversions::toPCL(msg, cloud2_rdf);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rdf(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud2_rdf, *cloud_rdf);
    NanoMapTime nm_time(msg.header.stamp.sec, msg.header.stamp.nsec);
    nanomap.AddPointCloud(cloud_rdf, nm_time, msg.header.seq);

    float insertion_time = sw.ElapsedMillis();
    std::cout << "## Insertion_time: " << insertion_time << std::endl;

    sw.Start();
    int num_samples = 10;
    float rad = 15.0;
    float delta = 2 * rad / (num_samples - 1) - .0001; // subtraction needed to get floating point happy in loop

    NanoMapKnnArgs args;
    args.axis_aligned_linear_covariance = Vector3(0.1, 0.1, 0.1);
    args.early_exit = false;
    args.query_point_current_body_frame = Vector3(3.0, 0.0, 0.0);
    int ni = 0;
    for (float x = -rad; x <= rad; x =x+delta) {
      for (float y = -rad; y <= rad; y =y+delta) {
        for (float z = -rad; z <= rad; z =z+delta) {
          ni++;
          args.query_point_current_body_frame = Vector3(x, y, z);
          NanoMapKnnReply reply = nanomap.KnnQuery(args);	
        }
      }
    }
    sw.Stop();
    float sample_time = sw.ElapsedMillis();
    std::cout << "## Sample_time: " << sample_time << std::endl;
    std::cout << "## Num queries: " << ni << std::endl;

    point_cloud_count++;
    datafile << point_cloud_count << "," << msg.header.seq << "," << global_time.ElapsedMillis() << "," << insertion_time << "," << sample_time << std::endl;
    std::cout << "## Processed point cloud: " << point_cloud_count << std::endl;
  }

  void DrawNanoMapVisualizer()
  {
    std::vector<Matrix4> edges = nanomap.GetCurrentEdges();
    nanomap_visualizer.DrawFrustums(edges);
  }

  void OdometryCallback(nav_msgs::Odometry const &odom)
  {
    if (NANOMAP_DEBUG_PRINT)
      std::cout << "In OdometryCallback" << std::endl;
    if (!got_camera_info) {
      if (NANOMAP_DEBUG_PRINT)
        std::cout << "## Not initialized" << std::endl;
      return;
    }
    geometry_msgs::Pose pose = odom.pose.pose;
    Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Vector3 pos = Vector3(pose.position.x, pose.position.y, pose.position.z);
    NanoMapTime nm_time(odom.header.stamp.sec, odom.header.stamp.nsec);
    NanoMapPose nm_pose(pos, quat, nm_time);
    nanomap.AddPose(nm_pose);

    Matrix4 transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = nm_pose.quaternion.toRotationMatrix();
    transform.block<3, 1>(0, 3) = nm_pose.position;
    nanomap_visualizer.SetLastPose(transform);
    DrawNanoMapVisualizer();
  }

  void SmoothedPosesCallback(nav_msgs::Path path)
  {
    if (!got_camera_info) 
      return;
    std::vector<NanoMapPose> smoothed_path_vector;
    size_t path_size = path.poses.size();
    for (size_t i = 0; i < path_size; i++)
    {
      geometry_msgs::PoseStamped pose = path.poses.at(i);
      Eigen::Quaterniond quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      Vector3 pos = Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      NanoMapTime nm_time(pose.header.stamp.sec, pose.header.stamp.nsec);
      NanoMapPose nm_pose(pos, quat, nm_time);
      smoothed_path_vector.push_back(nm_pose);
    }
    nanomap.AddPoseUpdates(smoothed_path_vector);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nanomap_benchmarking_node");
  BenchmarkingNode nanomap_node;
  ros::spin();
  return 0;
}
