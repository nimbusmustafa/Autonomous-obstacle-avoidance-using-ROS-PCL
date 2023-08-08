#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <algorithm>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class navigation
{
private:
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber lidar_sub;
  
    ros::Publisher cmd_vel_pub;

    float current_latitude;
    float current_longitude;
    double roll;
    double pitch;
    double yaw;
    float fright;
    float right;
    float fleft;
    float left;
    float front;
    float bright;
    float target_latitude;
    float target_longitude;
    float current_target;
    float num_of_samples;
    float distance;
    float X = 1.0;
    geometry_msgs::Twist cmd_vel;

    void imucallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        tf::Quaternion orientation_q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf::Matrix3x3(orientation_q).getRPY(roll, pitch, yaw);
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        current_latitude = msg->latitude;
        current_longitude = msg->longitude;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        num_of_samples = msg->ranges.size();

        right = std::min(*std::min_element(msg->ranges.begin(), msg->ranges.begin() + 72), 10.0f);
        fright = std::min(*std::min_element(msg->ranges.begin() + 73, msg->ranges.begin() + 144), 10.0f);
        front = std::min(*std::min_element(msg->ranges.begin() + 145, msg->ranges.begin() + 216), 10.0f);
        fleft = std::min(*std::min_element(msg->ranges.begin() + 217, msg->ranges.begin() + 288), 10.0f);
        left = std::min(*std::min_element(msg->ranges.begin() + 289, msg->ranges.begin() + 362), 10.0f);
    }


    float distancefn(float target_latitude, float target_longitude)
    {
        int R = 6371e3;
        float phi1 = current_latitude * M_PI / 180;
        float phi2 = target_latitude * M_PI / 180;
        float dphi = (target_latitude - current_latitude) * M_PI / 180;
        float dlambda = (target_longitude - current_longitude) * M_PI / 180;
        float a = std::sin(dphi / 2) * std::sin(dphi / 2) + std::cos(phi1) * std::cos(phi2) * std::sin(dlambda / 2) * std::sin(dlambda / 2);
        float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
        float distance1 = R * c;
        return distance1;
    }

    float targetfn(float target_latitude, float target_longitude, float currentlatitude, float currentlongitude)
    {
        int R = 6371e3;
        float phi1 = (currentlatitude)*M_PI / 180;
        float phi2 = (target_latitude)*M_PI / 180;
        float dphi = (target_latitude - currentlatitude) * M_PI / 180;
        float dlambda = (target_longitude - currentlongitude) * M_PI / 180;
        float y = std::sin(dlambda) * std::cos(phi2);
        float x = std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(dlambda);
        float theta = M_PI_2 - std::atan2(y, x);
        float target1 = std::fmod((theta * 180 / M_PI + 360), 360);
        return target1;
    }

    void goStraight()
    {
        if (distance > 0.5)
        {
            cmd_vel.linear.x = 1;
            cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(cmd_vel);
            ROS_INFO("going straight");
        }
        else
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel_pub.publish(cmd_vel);
            ROS_INFO("REACHED");
        }
    }

    void turn()
    {
        if (current_target - yaw > 0.07 || current_target - yaw <= -0.07)
        {
            if (current_target - yaw > 0)
            {
                cmd_vel.angular.z = 1 * std::abs(current_target - yaw);
                cmd_vel.linear.x = 0.0;
                ROS_INFO("turning left");

                cmd_vel_pub.publish(cmd_vel);
            }
            else
            {
                cmd_vel.angular.z = -1 * std::abs(current_target - yaw);
                cmd_vel.linear.x = 0.0;
                ROS_INFO("turning right");
                cmd_vel_pub.publish(cmd_vel);
            }
        }
        else
        {
            cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(cmd_vel);
            // ROS_INFO("GO STRAIGHT");
            goStraight();
        }
    }

    void handleCases()
    {
        if (fleft < (X + 0.5) && front > X && fright > X)
        {
            ROS_INFO(" CASE 1 - Front left");
            cmd_vel.linear.x = 0.5;
            cmd_vel.angular.z = -3.0;
            cmd_vel_pub.publish(cmd_vel);
        }
        else if (fleft > X && front < (X + 0.5) && fright > X)
        {
            ROS_INFO(" CASE 2 - Front");
            cmd_vel.linear.x = 1.0;
            cmd_vel.angular.z = -2.0;
            cmd_vel_pub.publish(cmd_vel);
        }
        else if (fleft > X && front > X && fright < (X + 0.5))
        {
            ROS_INFO(" CASE 3 - Front Right");
            cmd_vel.linear.x = 0.5;
            cmd_vel.angular.z = 3.0;
            cmd_vel_pub.publish(cmd_vel);
        }
        else if (fleft > X && front < X && fright < X)
        {
            ROS_INFO(" CASE 4 - Front and Front Right");
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 1.0;
            cmd_vel_pub.publish(cmd_vel);
        }
        else if (fleft < X && front > X && fright < X)
        {
            ROS_INFO(" CASE 5 - Front left and front right");
            cmd_vel.linear.x = 1.0;
            cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(cmd_vel);
        }
        else if (fleft < X && front < X && fright > X)
        {
            ROS_INFO(" CASE 6 - Front and Front Left");
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -1.0;
            cmd_vel_pub.publish(cmd_vel);
        }
        else if (fleft < X && front < X && fright < X)
        {
            ROS_INFO(" CASE 7 - Front and Front Left and Front Right");
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = -1.0;
            cmd_vel_pub.publish(cmd_vel);
        }
        else
        {
            ROS_INFO("ELSE FINAL CASE");
            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 1.0;
            cmd_vel_pub.publish(cmd_vel);
        }
    }

public:
    void run()
    {
        ros::NodeHandle nh;
        gps_sub = nh.subscribe("/gps/fix", 100, &navigation::gpsCallback, this);
        imu_sub = nh.subscribe("/imu", 100, &navigation::imucallback, this);
        lidar_sub = nh.subscribe("/laser_scan", 100, &navigation::laserCallback, this);
       
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/roverbot/cmd_vel", 100);
        ros::Rate rate(10.0);

        ROS_INFO("Enter Latitude: ");
        std::cin >> target_latitude;
        ROS_INFO("Enter Longitude: ");
        std::cin >> target_longitude;

        while (ros::ok())
        {
            current_target = targetfn(target_latitude, target_longitude, current_latitude, current_longitude) * M_PI / 180;
            distance = distancefn(target_latitude, target_longitude);

            if (current_target >= M_PI)
            {
                current_target -= 2 * M_PI;
            }
            ROS_INFO("distance=%f", distance);
            ROS_INFO("current angle: %f  target angle: %f", yaw, current_target);
            ROS_INFO("angle diff=%f", current_target - yaw);
            // ROS_INFO("no of samples =%f", num_of_samples);
            ROS_INFO("fright= %f", fright);
            ROS_INFO("fleft= %f", fleft);
            ROS_INFO("front= %f", front);
            ROS_INFO("right=%f", right);
            ROS_INFO("left=%f", left);

            if (distance > 1)
            {
                if (front < X || fright < X || fleft < X)
                {
                    handleCases();
                }
             
                else if ((front > X || fright > X || fleft > X || left > X) && right < X)
                {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.x = 2.0;
                }
                else if ((front > X || fright > X || fleft > X || right > X) && left < X)
                {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.x = -2.0;
                }
                else
                {
                    turn();
                }
            }
            else
            {
                cmd_vel.angular.z = 0;
                cmd_vel_pub.publish(cmd_vel);
                ROS_INFO("REACHED");
                goStraight();
            }

            rate.sleep();
            ros::spinOnce();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rover_navigation");
    navigation navigation1;
    navigation1.run();
    return 0;
}