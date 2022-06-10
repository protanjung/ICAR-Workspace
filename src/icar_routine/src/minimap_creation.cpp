#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "pcl/filters/crop_box.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#define BUFFER_LENGTH 10

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);
void cllbck_sub_odom_twist(const geometry_msgs::TwistStampedConstPtr &msg);
void cllbck_sub_odom_pose(const geometry_msgs::PoseStampedConstPtr &msg);
void cllbck_sub_base_points(const sensor_msgs::PointCloud2ConstPtr &msg);

int minimap_creation_init();
int minimap_creation_routine();

//=====Timer
ros::Timer tim_50hz;
//=====Subscriber
ros::Subscriber sub_odom_twist;
ros::Subscriber sub_odom_pose;
ros::Subscriber sub_base_points;
//=====Publisher
ros::Publisher pub_minimap_points;

//=====Odometry
double vx, vy, vth;
double x, y, th;

//=====PointCloud
pcl::PointCloud<pcl::PointXYZI>::Ptr base_points_raw;
pcl::PointCloud<pcl::PointXYZI>::Ptr base_points_filtered;
pcl::PointCloud<pcl::PointXYZI>::Ptr base_points_pool[BUFFER_LENGTH];

int main(int argc, char **argv)
{
    ros::init(argc, argv, "minimap_creation");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    //=====Subscriber
    sub_odom_twist = NH.subscribe("/odom/twist", 1, cllbck_sub_odom_twist);
    sub_odom_pose = NH.subscribe("/odom/pose", 1, cllbck_sub_odom_pose);
    sub_base_points = NH.subscribe("/base_points", 1, cllbck_sub_base_points);
    //=====Publisher
    pub_minimap_points = NH.advertise<sensor_msgs::PointCloud2>("/minimap_points", 1);

    if (minimap_creation_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_tim_50hz(const ros::TimerEvent &event)
{
    if (minimap_creation_routine() == 1)
        ros::shutdown();
}

//==============================================================================

void cllbck_sub_odom_twist(const geometry_msgs::TwistStampedConstPtr &msg)
{
    vx = msg->twist.linear.x;
    vy = msg->twist.linear.y;
    vth = msg->twist.angular.z;
}

void cllbck_sub_odom_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    th = tf::getYaw(msg->pose.orientation);
}

void cllbck_sub_base_points(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, *base_points_raw);

    pcl::CropBox<pcl::PointXYZI> crop_box;
    crop_box.setMin(Eigen::Vector4f(0.00, -7.50, -0.50, 1.00));
    crop_box.setMax(Eigen::Vector4f(10.00, 7.50, 0.50, 1.00));
    crop_box.setInputCloud(base_points_raw);
    crop_box.filter(*base_points_filtered);
}

//==============================================================================

int minimap_creation_init()
{
    // Initialize base_point_raw
    base_points_raw = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    // Initialize base_point_filtered
    base_points_filtered = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    // Initialize base_point_pool
    for (int i = 0; i < BUFFER_LENGTH; i++)
        base_points_pool[i] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    return 0;
}

int minimap_creation_routine()
{
    static double prev_x, prev_y, prev_th;

    double dx = x - prev_x;
    double dy = y - prev_y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance > 0.25)
    {
        double forward_or_backward = cos(th - atan2(dy, dx));

        //==============================

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        // Rotate on z-axis
        transform.rotate(Eigen::AngleAxisf(prev_th - th, Eigen::Vector3f::UnitZ()));

        // Translate on y-axis
        if (forward_or_backward >= 0)
            transform.translate(Eigen::Vector3f(-distance, 0, 0));
        else if (forward_or_backward < 0)
            transform.translate(Eigen::Vector3f(distance, 0, 0));

        //==============================

        pcl::PointCloud<pcl::PointXYZI>::Ptr minimap_points(new pcl::PointCloud<pcl::PointXYZI>);

        for (int i = 0; i < BUFFER_LENGTH - 1; i++)
        {
            pcl::transformPointCloud(*base_points_pool[i + 1], *base_points_pool[i], transform);
            *minimap_points += *base_points_pool[i];
        }
        pcl::copyPointCloud(*base_points_filtered, *base_points_pool[BUFFER_LENGTH - 1]);
        *minimap_points += *base_points_pool[BUFFER_LENGTH - 1];

        //==============================

        sensor_msgs::PointCloud2 msg_minimap_points;
        pcl::toROSMsg(*minimap_points, msg_minimap_points);
        msg_minimap_points.header.frame_id = "base_link";
        pub_minimap_points.publish(msg_minimap_points);

        //==============================

        prev_x = x;
        prev_y = y;
        prev_th = th;
    }

    return 0;
}
