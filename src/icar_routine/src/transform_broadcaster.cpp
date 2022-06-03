#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);
void cllbck_sub_odom_twist(const geometry_msgs::TwistStampedConstPtr &msg);
void cllbck_sub_odom_pose(const geometry_msgs::PoseStampedConstPtr &msg);

void send_transform(tfScalar x, tfScalar y, tfScalar z, tfScalar roll, tfScalar pitch, tfScalar yaw, const char *frame_id, const char *child_id);

//=====Timer
ros::Timer tim_50hz;
//=====Subscriber
ros::Subscriber sub_odom_twist;
ros::Subscriber sub_odom_pose;

//=====Odometry
double vx, vy, vth;
double x, y, th;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_broadcaster");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    //=====Subscriber
    sub_odom_twist = NH.subscribe("/odom/twist", 1, cllbck_sub_odom_twist);
    sub_odom_pose = NH.subscribe("/odom/pose", 1, cllbck_sub_odom_pose);

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_tim_50hz(const ros::TimerEvent &event)
{
    send_transform(x, y, 0.00,
                   0.00, 0.00, th,
                   "odom", "base_link");
    send_transform(2.85, 0.00, 0.65,
                   1.23, 20.00, 0.00,
                   "base_link", "lidar_link");
    send_transform(2.85, 0.00, 0.95,
                   0.00, 0.00, 0.00,
                   "base_link", "camera_link");
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

//==============================================================================

void send_transform(tfScalar x, tfScalar y, tfScalar z, tfScalar roll, tfScalar pitch, tfScalar yaw, const char *frame_id, const char *child_id)
{
    static tf::TransformBroadcaster transform_broadcaster;

    tf::Vector3 origin;
    tf::Quaternion rotation;

    origin.setValue(x, y, z);
    rotation.setRPY(roll * M_PI / 180, pitch * M_PI / 180, yaw * M_PI / 180);

    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(rotation);

    transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_id));
}
