#include "pcl/common/common.h"
#include "pcl/filters/crop_box.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

//=====Prototype
void cllbck_sub_minimap_points(const sensor_msgs::PointCloud2ConstPtr &msg);

//=====Subscriber
ros::Subscriber sub_minimap_points;
//=====Publisher
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
    pub_curb_points = NH.advertise<sensor_msgs::PointCloud2>("/curb_points", 1);
    pub_marker = NH.advertise<visualization_msgs::Marker>("/marker", 4);

    AS.start();
    ros::waitForShutdown();
}

//==============================================================================

void cllbck_sub_minimap_points(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr minimap_points_raw(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *minimap_points_raw);

    float koordinat_kiri[5][2], koordinat_kanan[5][2];
    float sum_x_kiri, sum_x_kanan;
    float sum_y_kiri, sum_y_kanan;
    float sum_xy_kiri, sum_xy_kanan;
    float sum_x2_kiri, sum_x2_kanan;
    float sum_y2_kiri, sum_y2_kanan;

    //==================================

    pcl::PointCloud<pcl::PointXYZI>::Ptr curb_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sample_points(new pcl::PointCloud<pcl::PointXYZI>);

    // Memindai sisi kiri mobil
    for (int x = 0; x < 5; x++)
    {
        float x_start = x * 1.00 + 1.50;
        float x_stop = x * 1.00 + 1.75;

        for (int y = 0; y < 20; y++)
        {
            float y_start = y * 0.25 + 1.50;
            float y_stop = y * 0.25 + 1.75;

            //==========================

            pcl::CropBox<pcl::PointXYZI> crop_box;
            crop_box.setMin(Eigen::Vector4f(x_start, y_start, 0.0, 0.0));
            crop_box.setMax(Eigen::Vector4f(x_stop, y_stop, 0.5, 0.0));
            crop_box.setInputCloud(minimap_points_raw);
            crop_box.filter(*sample_points);

            //==========================

            pcl::PointXYZI min_point, max_point;
            pcl::getMinMax3D(*sample_points, min_point, max_point);

            //==========================

            koordinat_kiri[x][0] = x_start;
            koordinat_kiri[x][1] = y_start;

            if (max_point.z - min_point.z > 0.10)
                break;
        }

        *curb_points += *sample_points;
    }

    // Memindai sisi kanan mobil
    for (int x = 0; x < 5; x++)
    {
        float x_start = x * 1.00 + 1.50;
        float x_stop = x * 1.00 + 1.75;

        for (int y = 0; y < 20; y++)
        {
            float y_start = y * -0.25 - 1.75;
            float y_stop = y * -0.25 - 1.50;

            //==========================

            pcl::CropBox<pcl::PointXYZI> crop_box;
            crop_box.setMin(Eigen::Vector4f(x_start, y_start, 0.0, 0.0));
            crop_box.setMax(Eigen::Vector4f(x_stop, y_stop, 0.5, 0.0));
            crop_box.setInputCloud(minimap_points_raw);
            crop_box.filter(*sample_points);

            //==========================

            pcl::PointXYZI min_point, max_point;
            pcl::getMinMax3D(*sample_points, min_point, max_point);

            //==========================

            koordinat_kanan[x][0] = x_start;
            koordinat_kanan[x][1] = y_start;

            if (max_point.z - min_point.z > 0.10)
                break;
        }

        *curb_points += *sample_points;
    }

    //==================================

    for (int i = 0; i < 5; i++)
    {
        sum_x_kiri += koordinat_kiri[i][0];
        sum_y_kiri += koordinat_kiri[i][1];
        sum_xy_kiri += koordinat_kiri[i][0] * koordinat_kiri[i][1];
        sum_x2_kiri += koordinat_kiri[i][0] * koordinat_kiri[i][0];
        sum_y2_kiri += koordinat_kiri[i][1] * koordinat_kiri[i][1];

        sum_x_kanan += koordinat_kanan[i][0];
        sum_y_kanan += koordinat_kanan[i][1];
        sum_xy_kanan += koordinat_kanan[i][0] * koordinat_kanan[i][1];
        sum_x2_kanan += koordinat_kanan[i][0] * koordinat_kanan[i][0];
        sum_y2_kanan += koordinat_kanan[i][1] * koordinat_kanan[i][1];
    }

    float numerator_kiri, denominator_kiri;
    numerator_kiri = (5 * sum_xy_kiri) - (sum_x_kiri * sum_y_kiri);
    denominator_kiri = (5 * sum_x2_kiri) - (sum_x_kiri * sum_x_kiri);
    float m_kiri = numerator_kiri / denominator_kiri;
    numerator_kiri = (sum_y_kiri * sum_x2_kiri) - (sum_x_kiri * sum_xy_kiri);
    denominator_kiri = (5 * sum_x2_kiri) - (sum_x_kiri * sum_x_kiri);
    float c_kiri = numerator_kiri / denominator_kiri;

    float numerator_kanan, denominator_kanan;
    numerator_kanan = (5 * sum_xy_kanan) - (sum_x_kanan * sum_y_kanan);
    denominator_kanan = (5 * sum_x2_kanan) - (sum_x_kanan * sum_x_kanan);
    float m_kanan = numerator_kanan / denominator_kanan;
    numerator_kanan = (sum_y_kanan * sum_x2_kanan) - (sum_x_kanan * sum_xy_kanan);
    denominator_kanan = (5 * sum_x2_kanan) - (sum_x_kanan * sum_x_kanan);
    float c_kanan = numerator_kanan / denominator_kanan;

    //==================================

    sensor_msgs::PointCloud2 msg_curb_points;
    pcl::toROSMsg(*curb_points, msg_curb_points);
    msg_curb_points.header.frame_id = "base_link";
    pub_curb_points.publish(msg_curb_points);

    //==================================

    visualization_msgs::Marker msg_marker;

    msg_marker.header = msg->header;
    msg_marker.type = visualization_msgs::Marker::POINTS;
    msg_marker.action = visualization_msgs::Marker::ADD;
    msg_marker.ns = "curb_detection";

    msg_marker.id = 0;
    msg_marker.scale.x = 0.1;
    msg_marker.scale.y = 0.1;
    msg_marker.color.r = 0.0;
    msg_marker.color.g = 1.0;
    msg_marker.color.b = 0.0;
    msg_marker.color.a = 1.0;
    msg_marker.points.clear();
    for (int i = 0; i < 5; i++)
    {
        geometry_msgs::Point p;
        p.x = koordinat_kiri[i][0];
        p.y = koordinat_kiri[i][1];
        p.z = 0;
        msg_marker.points.push_back(p);
    }
    pub_marker.publish(msg_marker);

    msg_marker.id = 1;
    msg_marker.scale.x = 0.1;
    msg_marker.scale.y = 0.1;
    msg_marker.color.r = 1.0;
    msg_marker.color.g = 0.0;
    msg_marker.color.b = 0.0;
    msg_marker.color.a = 1.0;
    msg_marker.points.clear();
    for (int i = 0; i < 5; i++)
    {
        geometry_msgs::Point p;
        p.x = koordinat_kanan[i][0];
        p.y = koordinat_kanan[i][1];
        p.z = 0;
        msg_marker.points.push_back(p);
    }
    pub_marker.publish(msg_marker);

    msg_marker.header = msg->header;
    msg_marker.type = visualization_msgs::Marker::LINE_LIST;
    msg_marker.action = visualization_msgs::Marker::ADD;
    msg_marker.ns = "curb_detection";

    msg_marker.id = 2;
    msg_marker.scale.x = 0.05;
    msg_marker.color.r = 0.0;
    msg_marker.color.g = 0.0;
    msg_marker.color.b = 1.0;
    msg_marker.color.a = 1.0;
    msg_marker.points.clear();
    {
        geometry_msgs::Point p;
        p.x = 0;
        p.y = m_kiri * p.x + c_kiri;
        p.z = 0;
        msg_marker.points.push_back(p);
        p.x = 10;
        p.y = m_kiri * p.x + c_kiri;
        p.z = 0;
        msg_marker.points.push_back(p);
    }
    pub_marker.publish(msg_marker);

    msg_marker.id = 3;
    msg_marker.scale.x = 0.05;
    msg_marker.color.r = 0.0;
    msg_marker.color.g = 0.0;
    msg_marker.color.b = 1.0;
    msg_marker.color.a = 1.0;
    msg_marker.points.clear();
    {
        geometry_msgs::Point p;
        p.x = 0;
        p.y = m_kanan * p.x + c_kanan;
        p.z = 0;
        msg_marker.points.push_back(p);
        p.x = 10;
        p.y = m_kanan * p.x + c_kanan;
        p.z = 0;
        msg_marker.points.push_back(p);
    }
    pub_marker.publish(msg_marker);
}