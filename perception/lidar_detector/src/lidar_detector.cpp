#include <memory>
#include <vector>
#include <algorithm>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

class LidarDetector : public rclcpp::Node {
public:
  LidarDetector() : Node("lidar_detector") {
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/carla/lidar", 10,
      std::bind(&LidarDetector::cb, this, std::placeholders::_1));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/carla/detections_markers", 10);
    det_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("/carla/detections", 10);
    RCLCPP_INFO(get_logger(), "lidar_detector up: subscribing /carla/lidar");
  }

private:
  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    CloudT::Ptr ds(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.2f, 0.2f, 0.2f);
    vg.filter(*ds);

    CloudT::Ptr cropped(new CloudT);
    pcl::PassThrough<PointT> pt;
    pt.setInputCloud(ds);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(-2.0, 3.0);
    pt.filter(*cropped);
    if (cropped->empty()) { publishDeleteAll(msg->header); return; }

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.3);
    seg.setMaxIterations(100);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
    seg.setInputCloud(cropped);
    seg.segment(*inliers, *coef);

    CloudT::Ptr objects(new CloudT);
    if (!inliers->indices.empty()) {
      pcl::ExtractIndices<PointT> ex;
      ex.setInputCloud(cropped);
      ex.setIndices(inliers);
      ex.setNegative(true);
      ex.filter(*objects);
    } else {
      *objects = *cropped;
    }
    if (objects->empty()) { publishDeleteAll(msg->header); return; }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(objects);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.6);
    ec.setMinClusterSize(15);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(objects);
    ec.extract(clusters);

    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker del;
    del.header = msg->header;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    vision_msgs::msg::Detection3DArray da;
    da.header = msg->header;

    int id = 0;
    for (const auto & cl : clusters) {
      float minx=1e9f, miny=1e9f, minz=1e9f, maxx=-1e9f, maxy=-1e9f, maxz=-1e9f;
      for (int idx : cl.indices) {
        const auto & p = objects->points[idx];
        minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
        minz = std::min(minz, p.z); maxz = std::max(maxz, p.z);
      }
      float cx=(minx+maxx)/2.0f, cy=(miny+maxy)/2.0f, cz=(minz+maxz)/2.0f;
      float sx=std::max(0.1f,maxx-minx), sy=std::max(0.1f,maxy-miny), sz=std::max(0.1f,maxz-minz);

      visualization_msgs::msg::Marker m;
      m.header = msg->header;
      m.ns = "clusters";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = cx; m.pose.position.y = cy; m.pose.position.z = cz;
      m.pose.orientation.w = 1.0;
      m.scale.x = sx; m.scale.y = sy; m.scale.z = sz;
      m.color.r = 0.1f; m.color.g = 1.0f; m.color.b = 0.2f; m.color.a = 0.5f;
      m.lifetime = rclcpp::Duration::from_seconds(5.0);
      ma.markers.push_back(m);

      vision_msgs::msg::Detection3D d;
      d.header = msg->header;
      d.bbox.center.position.x = cx; d.bbox.center.position.y = cy; d.bbox.center.position.z = cz;
      d.bbox.center.orientation.w = 1.0;
      d.bbox.size.x = sx; d.bbox.size.y = sy; d.bbox.size.z = sz;
      da.detections.push_back(d);
    }

    marker_pub_->publish(ma);
    det_pub_->publish(da);
    RCLCPP_INFO(get_logger(), "frame: %zu pts -> %zu clusters", cloud->size(), clusters.size());
  }

  void publishDeleteAll(const std_msgs::msg::Header & h) {
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker del;
    del.header = h;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);
    marker_pub_->publish(ma);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr det_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDetector>());
  rclcpp::shutdown();
  return 0;
}
