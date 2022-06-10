#include "pcl/common/common.h"
#include "pcl/filters/crop_box.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "visualization_msgs/Marker.h"

//=====Prototype
void cllbck_sub_minimap_points(const sensor_msgs::PointCloud2ConstPtr &msg);

int linreg(const int n, const double x[], const double y[], double *m, double *b, double *r2);

//=====Subscriber
ros::Subscriber sub_minimap_points;
//=====Publisher
ros::Publisher pub_curb_parameter_kiri;
ros::Publisher pub_curb_parameter_kanan;
ros::Publisher pub_curb_points;
ros::Publisher pub_marker;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "curb_detection");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Subscriber
    sub_minimap_points = NH.subscribe("/minimap_points", 1, cllbck_sub_minimap_points);
    //=====Publisher
    pub_curb_parameter_kanan = NH.advertise<std_msgs::Float64MultiArray>("/curb_parameter/kanan", 1);
    pub_curb_parameter_kiri = NH.advertise<std_msgs::Float64MultiArray>("/curb_parameter/kiri", 1);
    pub_curb_points = NH.advertise<sensor_msgs::PointCloud2>("/curb_points", 1);
    pub_marker = NH.advertise<visualization_msgs::Marker>("/marker", 20);

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_sub_minimap_points(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr minimap_points_raw(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *minimap_points_raw);

    //==================================

    pcl::PointCloud<pcl::PointXYZI>::Ptr curb_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sample_points(new pcl::PointCloud<pcl::PointXYZI>);

    double koordinat_kiri_x[20], koordinat_kiri_y[20];
    double koordinat_kanan_x[20], koordinat_kanan_y[20];

    bool limit_kiri = false;
    bool limit_kanan = false;

    // Memindai sisi kiri mobil
    for (int x = 0; x < 15; x++)
    {
        float x_start = x * 0.20 + 2.00;
        float x_stop = x * 0.20 + 2.25;

        size_t jumlah_titik[50];

        float avg_z[50] = {0};
        float sum_z[50] = {0};

        for (int y = 0; y < 50; y++)
        {
            float y_start = y * 0.1 + 0.50;
            float y_stop = y * 0.1 + 0.70;

            //==========================

            pcl::CropBox<pcl::PointXYZI> crop_box;
            crop_box.setMin(Eigen::Vector4f(x_start, y_start, 0.0, 0.0));
            crop_box.setMax(Eigen::Vector4f(x_stop, y_stop, 0.5, 0.0));
            crop_box.setInputCloud(minimap_points_raw);
            crop_box.filter(*sample_points);

            //==========================

            // pcl::PointXYZI min_point, max_point;
            // pcl::getMinMax3D(*sample_points, min_point, max_point);

            //==========================

            koordinat_kiri_x[x] = x_start;
            koordinat_kiri_y[x] = y_start;

            //==========================

            jumlah_titik[y] = sample_points->points.size();

            for (int i = 0; i < jumlah_titik[y]; i++)
                sum_z[y] += sample_points->at(i).z;
            avg_z[y] = sum_z[y] / jumlah_titik[y];

            //==========================

            if (y >= 5 && avg_z[y] - avg_z[y - 5] > 0.025)
                break;

            if (y == 49)
                limit_kiri = true;
        }

        *curb_points += *sample_points;
    }

    // Memindai sisi kanan mobil
    for (int x = 0; x < 15; x++)
    {
        float x_start = x * 0.25 + 2.00;
        float x_stop = x * 0.25 + 2.25;

        size_t jumlah_titik[50];

        float avg_z[50] = {0};
        float sum_z[50] = {0};

        for (int y = 0; y < 50; y++)
        {
            float y_start = y * -0.1 - 0.75;
            float y_stop = y * -0.1 - 0.50;

            //==========================

            pcl::CropBox<pcl::PointXYZI> crop_box;
            crop_box.setMin(Eigen::Vector4f(x_start, y_start, -0.5, 0.0));
            crop_box.setMax(Eigen::Vector4f(x_stop, y_stop, 0.5, 0.0));
            crop_box.setInputCloud(minimap_points_raw);
            crop_box.filter(*sample_points);

            //==========================

            // pcl::PointXYZI min_point, max_point;
            // pcl::getMinMax3D(*sample_points, min_point, max_point);

            //==========================

            koordinat_kanan_x[x] = x_start;
            koordinat_kanan_y[x] = y_start;

            //==========================

            jumlah_titik[y] = sample_points->points.size();

            for (int i = 0; i < jumlah_titik[y]; i++)
                sum_z[y] += sample_points->at(i).z;
            avg_z[y] = sum_z[y] / jumlah_titik[y];

            //==========================

            if (y >= 5 && avg_z[y] - avg_z[y - 5] > 0.025)
                break;

            if (y >= 49)
                limit_kanan = true;
        }

        *curb_points += *sample_points;
    }

    //==================================

    double avg_kiri = 0.0;
    for (int i = 0; i < 15; i++)
        avg_kiri += koordinat_kiri_y[i];
    avg_kiri /= 15.0;

    double avg_kanan = 0.0;
    for (int i = 0; i < 15; i++)
        avg_kanan += koordinat_kanan_y[i];
    avg_kanan /= 15.0;

    //==================================

    double m_kiri, b_kiri, r2_kiri;
    double m_kanan, b_kanan, r2_kanan;

    linreg(15, &koordinat_kiri_x[0], &koordinat_kiri_y[0], &m_kiri, &b_kiri, &r2_kiri);
    linreg(15, &koordinat_kanan_x[0], &koordinat_kanan_y[0], &m_kanan, &b_kanan, &r2_kanan);

    if (limit_kiri)
        r2_kiri = 0;

    if (limit_kanan)
        r2_kanan = 0;

    //==================================

    std_msgs::Float64MultiArray msg_curb_parameter_kiri;
    msg_curb_parameter_kiri.data.push_back(m_kiri);
    msg_curb_parameter_kiri.data.push_back(b_kiri);
    msg_curb_parameter_kiri.data.push_back(r2_kiri);
    msg_curb_parameter_kiri.data.push_back(avg_kiri);
    pub_curb_parameter_kiri.publish(msg_curb_parameter_kiri);

    std_msgs::Float64MultiArray msg_curb_parameter_kanan;
    msg_curb_parameter_kanan.data.push_back(m_kanan);
    msg_curb_parameter_kanan.data.push_back(b_kanan);
    msg_curb_parameter_kanan.data.push_back(r2_kanan);
    msg_curb_parameter_kanan.data.push_back(avg_kanan);
    pub_curb_parameter_kanan.publish(msg_curb_parameter_kanan);

    //==================================

    sensor_msgs::PointCloud2 msg_curb_points;
    pcl::toROSMsg(*curb_points, msg_curb_points);
    msg_curb_points.header.frame_id = "base_link";
    pub_curb_points.publish(msg_curb_points);

    //==================================

    visualization_msgs::Marker msg_marker_point;
    visualization_msgs::Marker msg_marker_line_kiri;
    visualization_msgs::Marker msg_marker_line_kanan;

    msg_marker_point.header.frame_id = "base_link";
    msg_marker_line_kiri.header.frame_id = "base_link";
    msg_marker_line_kanan.header.frame_id = "base_link";

    //==================================

    msg_marker_point.action = visualization_msgs::Marker::ADD;
    msg_marker_point.type = visualization_msgs::Marker::POINTS;
    msg_marker_point.pose.orientation.w = 1.0;
    msg_marker_point.scale.x = 0.1;
    msg_marker_point.scale.y = 0.1;
    msg_marker_point.color.a = 1.0;

    msg_marker_point.ns = "curb_detection_point_kiri";
    msg_marker_point.id = 0;
    msg_marker_point.color.r = 1.0;
    msg_marker_point.color.g = 0.0;
    msg_marker_point.color.b = 0.0;
    msg_marker_point.points.clear();
    for (int i = 0; i < 15; i++)
    {
        geometry_msgs::Point p;
        p.x = koordinat_kiri_x[i];
        p.y = koordinat_kiri_y[i];
        msg_marker_point.points.push_back(p);
    }
    pub_marker.publish(msg_marker_point);

    msg_marker_point.ns = "curb_detection_point_kanan";
    msg_marker_point.id = 1;
    msg_marker_point.color.r = 0.0;
    msg_marker_point.color.g = 1.0;
    msg_marker_point.color.b = 0.0;
    msg_marker_point.points.clear();
    for (int i = 0; i < 15; i++)
    {
        geometry_msgs::Point p;
        p.x = koordinat_kanan_x[i];
        p.y = koordinat_kanan_y[i];
        msg_marker_point.points.push_back(p);
    }
    pub_marker.publish(msg_marker_point);

    //==================================

    msg_marker_point.action = visualization_msgs::Marker::ADD;
    msg_marker_point.type = visualization_msgs::Marker::POINTS;
    msg_marker_point.pose.orientation.w = 1.0;
    msg_marker_point.scale.x = 0.5;
    msg_marker_point.scale.y = 0.5;
    msg_marker_point.color.a = 1.0;

    msg_marker_point.ns = "curb_detection_point_avg_kiri";
    msg_marker_point.id = 2;
    msg_marker_point.color.r = 1.0;
    msg_marker_point.color.g = 0.0;
    msg_marker_point.color.b = 1.0;
    msg_marker_point.points.clear();
    {
        geometry_msgs::Point p;
        p.x = 3;
        p.y = avg_kiri;
        msg_marker_point.points.push_back(p);
    }
    pub_marker.publish(msg_marker_point);

    msg_marker_point.ns = "curb_detection_point_avg_kanan";
    msg_marker_point.id = 3;
    msg_marker_point.color.r = 0.0;
    msg_marker_point.color.g = 1.0;
    msg_marker_point.color.b = 1.0;
    msg_marker_point.points.clear();
    {
        geometry_msgs::Point p;
        p.x = 3;
        p.y = avg_kanan;
        msg_marker_point.points.push_back(p);
    }
    pub_marker.publish(msg_marker_point);

    //==================================

    msg_marker_line_kiri.action = visualization_msgs::Marker::ADD;
    msg_marker_line_kiri.type = visualization_msgs::Marker::LINE_LIST;
    msg_marker_line_kiri.pose.orientation.w = 1.0;
    msg_marker_line_kiri.scale.x = 0.05;
    msg_marker_line_kiri.color.a = 1.00;

    msg_marker_line_kiri.ns = "curb_detection_line_kiri";
    msg_marker_line_kiri.id = 4;
    msg_marker_line_kiri.color.r = 0.0;
    msg_marker_line_kiri.color.g = 0.0;
    msg_marker_line_kiri.color.b = 1.0;
    msg_marker_line_kiri.points.clear();
    if (r2_kiri >= 0.5)
    {
        geometry_msgs::Point p;
        p.x = 0.0;
        p.y = m_kiri * 0.0 + b_kiri;
        msg_marker_line_kiri.points.push_back(p);
        p.x = 10.0;
        p.y = m_kiri * 10.0 + b_kiri;
        msg_marker_line_kiri.points.push_back(p);
    }
    else
    {
        msg_marker_line_kiri.action = visualization_msgs::Marker::DELETE;
    }
    pub_marker.publish(msg_marker_line_kiri);

    //==================================

    msg_marker_line_kanan.action = visualization_msgs::Marker::ADD;
    msg_marker_line_kanan.type = visualization_msgs::Marker::LINE_LIST;
    msg_marker_line_kanan.pose.orientation.w = 1.0;
    msg_marker_line_kanan.scale.x = 0.05;
    msg_marker_line_kanan.color.a = 1.00;

    msg_marker_line_kanan.ns = "curb_detection_line_kanan";
    msg_marker_line_kanan.id = 5;
    msg_marker_line_kanan.color.r = 0.0;
    msg_marker_line_kanan.color.g = 0.0;
    msg_marker_line_kanan.color.b = 1.0;
    msg_marker_line_kanan.points.clear();
    if (r2_kanan >= 0.5)
    {
        geometry_msgs::Point p;
        p.x = 0.0;
        p.y = m_kanan * 0.0 + b_kanan;
        msg_marker_line_kanan.points.push_back(p);
        p.x = 10.0;
        p.y = m_kanan * 10.0 + b_kanan;
        msg_marker_line_kanan.points.push_back(p);
    }
    else
    {
        msg_marker_line_kanan.action = visualization_msgs::Marker::DELETE;
    }
    pub_marker.publish(msg_marker_line_kanan);
}

//==============================================================================

int linreg(const int n, const double x[], const double y[], double *m, double *b, double *r2)
{
    double sumx = 0.0;
    double sumy = 0.0;
    double sumx2 = 0.0;
    double sumy2 = 0.0;
    double sumxy = 0.0;

    for (int i = 0; i < n; i++)
    {
        sumx += x[i];
        sumy += y[i];
        sumx2 += x[i] * x[i];
        sumy2 += y[i] * y[i];
        sumxy += x[i] * y[i];
    }

    //==================================

    double m_nominator = (n * sumxy - sumx * sumy);
    double b_nominator = (sumy * sumx2 - sumx * sumxy);
    double m_b_denominator = (n * sumx2 - sumx * sumx);

    double r_nominator = (sumxy - sumx * sumy / n);
    double r_denominator = (sumx2 - sumx * sumx / n) * (sumy2 - sumy * sumy / n);
    double r = r_nominator / sqrt(r_denominator);

    //==================================

    if (m_b_denominator == 0)
    {
        *m = 0;
        *b = 0;
        return 1;
    }
    else
    {
        *m = m_nominator / m_b_denominator;
        *b = b_nominator / m_b_denominator;
    }

    //==================================

    if (r_denominator == 0)
    {
        *r2 = 1;
        return 1;
    }
    else
    {
        *r2 = r * r;
    }

    return 0;
}