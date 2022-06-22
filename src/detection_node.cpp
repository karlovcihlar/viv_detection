#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#define PCL_NO_PRECOMPILE
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/pcl_macros.h>


// #include <pcl/io/pcd_io.h>


using namespace std;

ros::Publisher pub;
ros::Publisher pub2;

struct EIGEN_ALIGN16 PointXYZIR
{
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t reflectivity;
    PCL_MAKE_ALIGNED_OPERATOR_NEW 
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (uint16_t, reflectivity, reflectivity)
)


void cloud_cb(const sensor_msgs::PointCloud2 & cloud_msg)
{
    // 1. Msg to pointcloud
    pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>);
    // pcl::fromROSMsg(*input, *cloud);
    int pointBytes = cloud_msg.point_step;
    int offset_x;
    int offset_y;
    int offset_z;
    int offset_int;
    int offset_ref;
    for (int f=0; f<cloud_msg.fields.size(); ++f)
    {
    if (cloud_msg.fields[f].name == "x")
      offset_x = cloud_msg.fields[f].offset;
    if (cloud_msg.fields[f].name == "y")
      offset_y = cloud_msg.fields[f].offset;
    if (cloud_msg.fields[f].name == "z")
      offset_z = cloud_msg.fields[f].offset;
    if (cloud_msg.fields[f].name == "intensity")
      offset_int = cloud_msg.fields[f].offset;
    if (cloud_msg.fields[f].name == "reflectivity")
      offset_ref = cloud_msg.fields[f].offset;
    
    }
    

  // populate point cloud object
    int width=0;
    for (int p=0; p<cloud_msg.width*cloud_msg.height; ++p)
    {
      PointXYZIR newPoint;

      newPoint.x = *(float*)(&cloud_msg.data[0] + (pointBytes*p) + offset_x);
      newPoint.y = *(float*)(&cloud_msg.data[0] + (pointBytes*p) + offset_y);
      newPoint.z = -1*(*(float*)(&cloud_msg.data[0] + (pointBytes*p) + offset_z));
      newPoint.intensity = *(float*)(&cloud_msg.data[0] + (pointBytes*p) + offset_int);
      newPoint.reflectivity = *(uint16_t*)(&cloud_msg.data[0] + (pointBytes*p) + offset_ref);
      if(newPoint.reflectivity < 2000 && newPoint.reflectivity > 50){
        cloud->points.push_back(newPoint);
        width++;
      }
        
    }

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<PointXYZIR>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZIR>);
    ROS_INFO("width: %d", width);
    cloud_filtered->width = width;
    cloud_filtered->height = 1;
    cloud_filtered->is_dense = false;
    cloud_filtered->points.resize (static_cast<int> (cloud->width));


    pcl::PassThrough<PointXYZIR> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits(50, 300);
    // pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);

    PointXYZIR min_pt, max_pt;
    pcl::getMinMax3D(*cloud_filtered, min_pt, max_pt);

    geometry_msgs::Pose center;
    geometry_msgs::Vector3 size;
    center.position.x = (min_pt.x + max_pt.x)/2;
    center.position.y = (min_pt.y + max_pt.y)/2;
    center.position.z = (min_pt.z + max_pt.z)/2;
    center.orientation.x=0;
    center.orientation.y=0;
    center.orientation.z=1;
    center.orientation.w=0;
    size.x=(max_pt.x-min_pt.x )/2;
    size.y=(max_pt.y-min_pt.y )/2;
    size.z=(max_pt.z-min_pt.z )/2;
    jsk_recognition_msgs::BoundingBox bbox;
    bbox.pose = center;
    bbox.dimensions = size;
    bbox.header.frame_id = cloud_msg.header.frame_id;

    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = cloud_msg.header.frame_id;

    ros::Time curr_time = ros::Time::now();
    output.header.stamp = curr_time;
    bbox.header.stamp = curr_time;

    pub.publish(output);
    pub2.publish(bbox);
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "viv_detector");

    ros::NodeHandle n;
    std::string topic = n.resolveName("/os_cloud_node/points");

    uint32_t queue_size = 3;
    pub = n.advertise<sensor_msgs::PointCloud2>("extracted_points", 1000);
    pub2 = n.advertise<jsk_recognition_msgs::BoundingBox>("extracted_bbox", 1000); 

    ros::Subscriber sub = n.subscribe(topic, queue_size, cloud_cb);

    ros::spin();

    return 0;
}