#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

void cllbck_sub_lidar_points(const sensor_msgs::PointCloud2ConstPtr &msg);

//=====Timer
ros::Timer tim_50hz;
//=====Subscriber
ros::Subscriber sub_lidar_points;
//=====Publisher
ros::Publisher pub_lidar_points;
//=====TransformListener
tf::TransformListener *transform_listener;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_interpreter");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Subscriber
    sub_lidar_points = NH.subscribe("/lidar_points", 1, cllbck_sub_lidar_points);
    //=====Publisher
    pub_lidar_points = NH.advertise<sensor_msgs::PointCloud2>("/base_points", 1);
    //=====TransformListener
    transform_listener = new tf::TransformListener(NH);

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_sub_lidar_points(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr points_lidar_link(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr points_base_link(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *points_lidar_link);

    transform_listener->waitForTransform("base_link", "lidar_link", msg->header.stamp, ros::Duration(0.1));
    pcl_ros::transformPointCloud("base_link", *points_lidar_link, *points_base_link, *transform_listener);

    sensor_msgs::PointCloud2 msg_base_points;
    pcl::toROSMsg(*points_base_link, msg_base_points);
    msg_base_points.header.frame_id = "base_link";
    pub_lidar_points.publish(msg_base_points);
}