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

#include <nanomap_ros/nanomap.h>
#include <nanomap_ros/nanomap_visualizer.h>
#include <nanomap_ros/nanomap_types.h>

#include <nanomap_ros/stopwatch.h>

using namespace std;

Stopwatch global_time;

geometry_msgs::Pose curr_pose;

visualization_msgs::Marker getQueryPtMarker(int status, int id, Vector3 point_pos)
{
  visualization_msgs::Marker marker1;
  marker1.header.frame_id = "rocky0704/base";
  marker1.header.stamp = ros::Time();
  marker1.ns = "p" + std::to_string(id);
  marker1.id = id;
  marker1.type = visualization_msgs::Marker::SPHERE;
  marker1.action = visualization_msgs::Marker::ADD;
  marker1.pose.position.x = point_pos(0);
  marker1.pose.position.y = point_pos(1);
  marker1.pose.position.z = point_pos(2);
  marker1.pose.orientation.x = 0.0;
  marker1.pose.orientation.y = 0.0;
  marker1.pose.orientation.z = 0.0;
  marker1.pose.orientation.w = 1.0;
  marker1.scale.x = 1.0;
  marker1.scale.y = 1.0;
  marker1.scale.z = 1.0;
  if (status == 6) // free space, green
  {
    marker1.color.a = 1.0; // Don't forget to set the alpha!
    marker1.color.r = 0.0;
    marker1.color.g = 1.0;
    marker1.color.b = 0.0;
  }
  else if(status == 5) {   // occluded, purple
    marker1.color.a = 1.0; // Don't forget to set the alpha!
    marker1.color.r = 1.0;
    marker1.color.g = 0.0;
    marker1.color.b = 1.0;
  }
  else if(status == 3) {   // laterally outside FOV, blue
    marker1.color.a = 1.0; // Don't forget to set the alpha!
    marker1.color.r = 0.0;
    marker1.color.g = 0.0;
    marker1.color.b = 1.0;
  }
  else if(status == 4) {   // beyond sensor horizon, yellow
    marker1.color.a = 1.0; // Don't forget to set the alpha!
    marker1.color.r = 1.0;
    marker1.color.g = 1.0;
    marker1.color.b = 0.0;
  }
  else
  {
    // cout << "Occupied Space!" << endl;
    // red
    marker1.color.a = 1.0; // Don't forget to set the alpha!
    marker1.color.r = 1.0;
    marker1.color.g = 0.0;
    marker1.color.b = 0.0;
  }
  return marker1;
}

class NanoMapNode
{
public:
  NanoMapNode() : nh("~")
  {
    nanomap_visualizer.Initialize(nh);

    nanomap.SetSensorRange(20.0);
    nanomap.SetNumDepthImageHistory(150);
    Matrix3 body_to_rdf;
    body_to_rdf << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    nanomap.SetBodyToRdf(body_to_rdf);

    pcl_sub = nh.subscribe("points", 100, &NanoMapNode::PointCloudCallback, this);
    pose_updates_sub = nh.subscribe("path", 100, &NanoMapNode::SmoothedPosesCallback, this);
    odom_sub = nh.subscribe("odometry", 100, &NanoMapNode::OdometryCallback, this);
    camera_info_sub = nh.subscribe("/camera/camera_info", 100, &NanoMapNode::CameraInfoCallback, this);
  };

  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Subscriber pose_updates_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber camera_info_sub;
  NanoMap nanomap;
  NanoMapVisualizer nanomap_visualizer;
  ros::Publisher query_points_pub = nh.advertise<visualization_msgs::MarkerArray>("query_points", 0);

  bool got_camera_info = false;

  void CameraInfoCallback(const sensor_msgs::CameraInfo &msg)
  {
    if (got_camera_info)
      return;
		Matrix3 K_camera_info;
		K_camera_info << msg.K[0], msg.K[1], msg.K[2], msg.K[3], msg.K[4], msg.K[5], 
                     msg.K[6], msg.K[7], msg.K[8];
    nanomap.SetCameraInfo(4.0, msg.width, msg.height, K_camera_info);
    got_camera_info = true;
    if (NANOMAP_DEBUG_PRINT)
      std::cout << "Initialized camera parameters" << std::endl;
  }

  void PointCloudCallback(const sensor_msgs::PointCloud2 &msg)
  {
    if (NANOMAP_DEBUG_PRINT)
      std::cout << "In PointCloudCallback" << std::endl;
    if (!got_camera_info) {
      if (NANOMAP_DEBUG_PRINT)
        std::cout << "## Not initialized" << std::endl;
      return;
    }
    Stopwatch sw;
    sw.Start();
    pcl::PCLPointCloud2 cloud2_rdf;
    pcl_conversions::toPCL(msg, cloud2_rdf);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rdf(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud2_rdf, *cloud_rdf);
    NanoMapTime nm_time(msg.header.stamp.sec, msg.header.stamp.nsec);
    nanomap.AddPointCloud(cloud_rdf, nm_time, msg.header.seq);

    std::vector<Matrix4> edges = nanomap.GetCurrentEdges();
    if (NANOMAP_DEBUG_PRINT)
      std::cout << "PointCloudCallback " << edges.size() << " edges" << std::endl;

    float insertion_time = sw.ElapsedMillis();
    // std::cout << "insertion_time: " << insertion_time << std::endl;

    sw.Start();

    sw.Stop();
    float distance_update_time = sw.ElapsedMillis();
    //std::cout << "distance_update_time: " << distance_update_time << std::endl;

    sw.Start();
    int num_samples = 10;
    float rad = 15.0;
    float delta = 2 * rad / (num_samples - 1) - .0001; // subtraction needed to get floating point happy in loop

    NanoMapKnnArgs args;
    args.axis_aligned_linear_covariance = Vector3(0.1, 0.1, 0.1);
    args.early_exit = false;

    visualization_msgs::MarkerArray query_points;
    NanoMapKnnReply reply;

    int n = 0;
    for (float m = -1.57; m <= 1.57; m += 0.5)
    {
      for (float x = -rad; x <= rad; x = x + delta)
      {
        for (float q = -2; q <= 2; q += 1) // add z sample - to be improved
        {
          n++;
          bool hit = false;
          // args.query_point_current_body_frame = Vector3(x, m * x, 0.0);
          args.query_point_current_body_frame = Vector3(x, m * x, q);
          reply = nanomap.KnnQuery(args); // pass a point to query
          // std::cout << "Query point: "
          //           << args.query_point_current_body_frame(0) << " " << args.query_point_current_body_frame(1) << " " << args.query_point_current_body_frame(2) << std::endl;
          // std::cout << "FOV status: " << reply.fov_status << std::endl;

          for (Vector3 v : reply.closest_points_in_frame_id)
          {
            double cld = (reply.query_point_in_frame_id - v).norm();

            if (cld <= 2.0)
            {
              hit = true;
              break;
            }
          }
          int status;
          // if ((int)(reply.fov_status) == 6)
          // {
          //   if (!hit)
          //     status = 6;
          //   else
          //     status = 5;
          // }
          // else
          //   status = 5;
          
          status = (int)reply.fov_status;

          visualization_msgs::Marker mp0 = getQueryPtMarker(status, n, args.query_point_current_body_frame);
          query_points.markers.push_back(mp0);
        }
      }
    }

    args.query_point_current_body_frame = Vector3(15.0 - curr_pose.position.x, 0.0 - curr_pose.position.y, 0.0 - curr_pose.position.z);
    reply = nanomap.KnnQuery(args);

    query_points_pub.publish(query_points);

    sw.Stop();
    float sample_time = sw.ElapsedMillis();
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
  ros::init(argc, argv, "nanomap_visualization_example");
  NanoMapNode nanomap_node;
  ros::spin();
  return 0;
}
