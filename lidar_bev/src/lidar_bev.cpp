#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <image_transport/image_transport.hpp>
#include <lidar_bev/cloud_filter.hpp>

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

CloudFilter filter;

class PointCloudPublisher : public rclcpp::Node
{
public:
  PointCloudPublisher()
  : Node("pointcloud_publishers"), count_(0)
  {
    pub1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_topic_1", 10);
    pub2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filetered_pointcloud", 10);

    
  }

private:
  void cloud_callback(const sensor_msgs::PointCloud2Ptr & cloud_msg)
  {
    // Change the intensity field name, so we can use it with pcl point type
    cloud_msg->fields[3].name = "intensity";
    // Convert cloud msg to pcl type 
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::moveFromROSMsg(*cloud_msg, *cloud_ptr);

     // Update the filter cloud
    filter.setInputCloud(cloud_ptr);

    // Remove points out of the camera FOV
    //    filter.filterFOV(camera_fov);

    std::shared_ptr<Mat> bird_ground = filter.birdGround(cell_size, ground_cell_span, grid_dim);

    // Remove floor points
    if(remove_floor){
        filter.removeFloor(cell_size_height_map, height_threshold, grid_dim_height_map);
    }

    std::shared_ptr<Mat> bird_view = filter.birdView(cell_size, max_height, grid_dim, false);

    int grid_cells = grid_dim / cell_size; // Number of col/rows of the birdview

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<bird_ground->rows; i++){
        float* row_ptr = bird_ground->ptr<float>(i);
        for(int j=0; j<bird_ground->cols; j++){
            float z = row_ptr[j] - filter.getBaseVeloTF().getOrigin().z();
            double x = (grid_cells/2. - i)*cell_size;
            double y = (grid_cells/2. - j)*cell_size;

            // Push the ground XYZ point
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = z;
            ground_cloud_ptr->push_back(point);
        }
    }

    // Publish the ground pointcloud
    sensor_msgs::PointCloud2 ground_ros;
    pcl::toROSMsg(*ground_cloud_ptr, ground_ros);
    ground_ros.header = cloud_msg->header;
    ground_cloud_pub.publish(ground_ros);

    auto cloud2 = sensor_msgs::msg::PointCloud2();
    cloud2.header.stamp = this->now();
    cloud2.header.frame_id = "map";
    // Aquí puedes rellenar cloud2 con datos reales

    RCLCPP_INFO(this->get_logger(), "Publicando PointClouds %zu", count_++);
    pub1_->publish(cloud1);
    pub2_->publish(cloud2);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub1_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub2_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}