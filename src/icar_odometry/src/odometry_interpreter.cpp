#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include "tf/tf.h"

#define M_PER_PULSE 0.0022934
#define PULSE_PER_M 436.0409399

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);
void cllbck_sub_rotary_encoder_kiri(const std_msgs::UInt16ConstPtr &msg);
void cllbck_sub_rotary_encoder_kanan(const std_msgs::UInt16ConstPtr &msg);
void cllbck_sub_gyroscope(const std_msgs::Float32ConstPtr &msg);

//=====Timer
ros::Timer tim_50hz;
//=====Subscriber
ros::Subscriber sub_rotary_encoder_kiri;
ros::Subscriber sub_rotary_encoder_kanan;
ros::Subscriber sub_gyroscope;
//=====Publisher
ros::Publisher pub_odom_twist;
ros::Publisher pub_odom_pose;

//=====Rotary Encoder
uint16_t rotary_encoder_kiri = 0;
uint16_t rotary_encoder_kanan = 0;

//=====Gyroscope
float gyroscope_deg = 0;
float gyroscope_rad = 0;

//=====Odometry
double velocity_kiri = 0;
double velocity_kanan = 0;
double vx = 0, vy = 0, vth = 0;
double x = 0, y = 0, th = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_interpreter");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    //=====Subscriber
    sub_rotary_encoder_kiri = NH.subscribe("/car/odom_left/raw", 1, cllbck_sub_rotary_encoder_kiri);
    sub_rotary_encoder_kanan = NH.subscribe("/car/odom_right/raw", 1, cllbck_sub_rotary_encoder_kanan);
    sub_gyroscope = NH.subscribe("/car/gyro", 1, cllbck_sub_gyroscope);
    //=====Publisher
    pub_odom_twist = NH.advertise<geometry_msgs::TwistStamped>("/odom/twist", 1);
    pub_odom_pose = NH.advertise<geometry_msgs::PoseStamped>("/odom/pose", 1);

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_tim_50hz(const ros::TimerEvent &event)
{
    static ros::Time start_time = ros::Time::now();

    static uint16_t prev_rotary_encoder_kiri = 0;
    static uint16_t prev_rotary_encoder_kanan = 0;
    static float prev_gyroscope_deg = 0;
    static float prev_gyroscope_rad = 0;

    // Mengabaikan data 2 detik pertama
    if (ros::Time::now() - start_time < ros::Duration(2))
    {
        prev_rotary_encoder_kiri = rotary_encoder_kiri;
        prev_rotary_encoder_kanan = rotary_encoder_kanan;
        prev_gyroscope_deg = gyroscope_deg;
        prev_gyroscope_rad = gyroscope_rad;

        return;
    }

    // Mengatasi diskontinuitas data rotary encoder kiri
    if (rotary_encoder_kiri < 16384 && prev_rotary_encoder_kiri > 49152)
        velocity_kiri = rotary_encoder_kiri + (65536 - prev_rotary_encoder_kiri);
    else if (rotary_encoder_kiri > 49152 && prev_rotary_encoder_kiri < 16384)
        velocity_kiri = -prev_rotary_encoder_kiri - (655356 - rotary_encoder_kiri);
    else
        velocity_kiri = rotary_encoder_kiri - prev_rotary_encoder_kiri;

    // Mengatasi diskontinuitas data rotary encoder kanan
    if (rotary_encoder_kanan < 16384 && prev_rotary_encoder_kanan > 49152)
        velocity_kanan = rotary_encoder_kanan + (65536 - prev_rotary_encoder_kanan);
    else if (rotary_encoder_kanan > 49152 && prev_rotary_encoder_kanan < 16384)
        velocity_kanan = -prev_rotary_encoder_kanan - (65536 - rotary_encoder_kanan);
    else
        velocity_kanan = rotary_encoder_kanan - prev_rotary_encoder_kanan;

    // Menghitung odometry menggunakan data kecepatan roda dan gyroscope
    vth = gyroscope_rad - prev_gyroscope_rad;
    th += vth;

    vx = ((velocity_kiri + velocity_kanan) * 0.5 * M_PER_PULSE) * cos(th);
    vy = ((velocity_kiri + velocity_kanan) * 0.5 * M_PER_PULSE) * sin(th);
    x += vx;
    y += vy;

    // Publish data kecepatan dan posisi
    ros::Time time = ros::Time::now();

    geometry_msgs::TwistStamped msg_odom_twist;
    msg_odom_twist.header.stamp = time;
    msg_odom_twist.header.frame_id = "odom";
    msg_odom_twist.twist.linear.x = vx;
    msg_odom_twist.twist.linear.y = vy;
    msg_odom_twist.twist.angular.z = vth;
    pub_odom_twist.publish(msg_odom_twist);

    geometry_msgs::PoseStamped msg_odom_pose;
    msg_odom_pose.header.stamp = time;
    msg_odom_pose.header.frame_id = "odom";
    msg_odom_pose.pose.position.x = x;
    msg_odom_pose.pose.position.y = y;
    msg_odom_pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
    pub_odom_pose.publish(msg_odom_pose);

    // Mencatat posisi terakhir roda kanan dan kiri
    prev_rotary_encoder_kiri = rotary_encoder_kiri;
    prev_rotary_encoder_kanan = rotary_encoder_kanan;
    prev_gyroscope_deg = gyroscope_deg;
    prev_gyroscope_rad = gyroscope_rad;
}

//==============================================================================

void cllbck_sub_rotary_encoder_kiri(const std_msgs::UInt16ConstPtr &msg)
{
    rotary_encoder_kiri = msg->data;
}

void cllbck_sub_rotary_encoder_kanan(const std_msgs::UInt16ConstPtr &msg)
{
    rotary_encoder_kanan = msg->data;
}

void cllbck_sub_gyroscope(const std_msgs::Float32ConstPtr &msg)
{
    gyroscope_deg = msg->data;
    gyroscope_rad = msg->data * M_PI / 180.0;
}