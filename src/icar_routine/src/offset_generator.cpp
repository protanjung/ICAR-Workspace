#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "icar_routine/misc.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#define OFFSET_STEP_ORIENTASI 0.75

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);
void cllbck_tim_1hz(const ros::TimerEvent &event);
void cllbck_sub_curb_parameter_kiri(const std_msgs::Float64MultiArrayConstPtr &msg);
void cllbck_sub_curb_parameter_kanan(const std_msgs::Float64MultiArrayConstPtr &msg);

void publish_offset_posisi(double x, double y);
void publish_offset_orientasi(double th);

void offset_supaya_mobil_ke_kiri();
void offset_supaya_mobil_ke_kanan();

//=====Timer
ros::Timer tim_50hz;
ros::Timer tim_1hz;
//=====Subscriber
ros::Subscriber sub_curb_parameter_kiri;
ros::Subscriber sub_curb_parameter_kanan;
//=====Publisher
ros::Publisher pub_offset_posisi;
ros::Publisher pub_offset_orientasi;

//=====CurbParameter
double m_kiri, c_kiri, r2_kiri, avg_kiri;
double m_kanan, c_kanan, r2_kanan, avg_kanan;

//=====Offset
int offset_timer = 0;
bool offset_active = false;

double offset_x;
double offset_y;
double offset_th;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offset_generator");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    tim_1hz = NH.createTimer(ros::Duration(1.0), cllbck_tim_1hz);
    //=====Subscriber
    sub_curb_parameter_kiri = NH.subscribe("/curb_parameter/kiri", 1, cllbck_sub_curb_parameter_kiri);
    sub_curb_parameter_kanan = NH.subscribe("/curb_parameter/kanan", 1, cllbck_sub_curb_parameter_kanan);
    //=====Publisher
    pub_offset_posisi = NH.advertise<geometry_msgs::Point>("/autonomous_car/gps_offset", 1);
    pub_offset_orientasi = NH.advertise<geometry_msgs::Point32>("/serial/offsetGPS", 1);

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_tim_50hz(const ros::TimerEvent &event)
{
    char ch = kbhit_and_getch();

    switch (ch)
    {
    case 'q':
        printf("Offset Orientasi +++\n");
        publish_offset_orientasi(OFFSET_STEP_ORIENTASI);
        break;
    case 'e':
        printf("Offset Orientasi ---\n");
        publish_offset_orientasi(-OFFSET_STEP_ORIENTASI);
        break;
    case '=':
        offset_active = !offset_active;
        if (offset_active)
            printf("Offset Active\n");
        else
            printf("Offset Inactive\n");
        break;
    }
}

void cllbck_tim_1hz(const ros::TimerEvent &event)
{
    static double set_point_kiri = 2.00;
    static double set_point_kanan = 2.00;

    double jarak_mobil_ke_curb_kiri = avg_kiri;
    double jarak_mobil_ke_curb_kanan = -avg_kanan;
    double jarak_curb_kiri_ke_curb_kanan = avg_kiri - avg_kanan;

    bool reliabel_kiri = false;
    bool reliabel_kanan = false;

    if (r2_kiri >= 0.5)
        reliabel_kiri = true;
    if (r2_kanan >= 0.5)
        reliabel_kanan = true;

    // if (reliabel_kiri && reliabel_kanan)
    //     set_point_kanan = jarak_curb_kiri_ke_curb_kanan - 2.00;

    printf("x: %.2f, y: %.2f, th: %.2f | kiri: %d %.2f %.2f, kanan: %d %.2f %.2f\n",
           offset_x, offset_y, offset_th,
           reliabel_kiri, jarak_mobil_ke_curb_kiri, set_point_kiri,
           reliabel_kanan, jarak_mobil_ke_curb_kanan, set_point_kanan);

    if (offset_timer++ <= 2)
        return;

    if (offset_active != true)
        return;

    if (reliabel_kiri && reliabel_kanan && offset_active)
    {
        set_point_kanan = jarak_curb_kiri_ke_curb_kanan - 2.00;

        if (jarak_mobil_ke_curb_kiri < set_point_kiri - 0.25)
            offset_supaya_mobil_ke_kanan();
        else if (jarak_mobil_ke_curb_kiri > set_point_kiri + 0.25)
            offset_supaya_mobil_ke_kiri();
    }
    else if (!reliabel_kiri && reliabel_kanan && offset_active)
    {
        if (jarak_mobil_ke_curb_kanan < set_point_kanan - 0.25)
            offset_supaya_mobil_ke_kiri();
        else if (jarak_mobil_ke_curb_kanan > set_point_kanan + 0.25)
            offset_supaya_mobil_ke_kanan();
    }
    else if (reliabel_kiri && !reliabel_kanan && offset_active)
    {
        if (jarak_mobil_ke_curb_kiri < set_point_kiri - 0.25)
            offset_supaya_mobil_ke_kanan();
        else if (jarak_mobil_ke_curb_kiri > set_point_kiri + 0.25)
            offset_supaya_mobil_ke_kiri();
    }
}

//==============================================================================

void cllbck_sub_curb_parameter_kiri(const std_msgs::Float64MultiArrayConstPtr &msg)
{
    m_kiri = msg->data[0];
    c_kiri = msg->data[1];
    r2_kiri = msg->data[2];
    avg_kiri = msg->data[3];
}

void cllbck_sub_curb_parameter_kanan(const std_msgs::Float64MultiArrayConstPtr &msg)
{
    m_kanan = msg->data[0];
    c_kanan = msg->data[1];
    r2_kanan = msg->data[2];
    avg_kanan = msg->data[3];
}

//==============================================================================

void publish_offset_posisi(double x, double y)
{
    geometry_msgs::Point msg_offset_posisi;
    msg_offset_posisi.x = x;
    msg_offset_posisi.y = y;
    pub_offset_posisi.publish(msg_offset_posisi);

    offset_x += x;
    offset_y += y;
}

void publish_offset_orientasi(double th)
{
    geometry_msgs::Point32 msg_offset_orientasi;
    msg_offset_orientasi.z = th;
    pub_offset_orientasi.publish(msg_offset_orientasi);

    offset_th += th;
}

//==============================================================================

void offset_supaya_mobil_ke_kiri()
{
    offset_timer = 0;
    printf("Offset Orientasi ---\n");
    publish_offset_orientasi(-OFFSET_STEP_ORIENTASI);
    publish_offset_posisi(0.15, 0);
}

void offset_supaya_mobil_ke_kanan()
{
    offset_timer = 0;
    printf("Offset Orientasi +++\n");
    publish_offset_orientasi(OFFSET_STEP_ORIENTASI);
    publish_offset_posisi(-0.15, 0);
}