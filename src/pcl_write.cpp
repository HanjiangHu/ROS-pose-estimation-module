#include <ros/ros.h>
#include <sstream>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#define PATH "home/huhanjiang/"
int count = 0;
void cloudCB(const sensor_msgs::PointCloud2 &input)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(input, cloud);
	//cloud.setKeepOrganized(true);
    std::ostringstream oss;
    oss << PATH << "pcd" << count << ".pcd";


    pcl::io::savePCDFileASCII (oss.str(), cloud);  
    std::cout << "hhhh"<<std:: endl;
    ros::Rate loop_rate(0.05);
    loop_rate.sleep();
}

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_write");

    ros::NodeHandle nh;
    ros::Subscriber bat_sub = nh.subscribe("/camera/depth_registered/points", 10, cloudCB);//pcl_output

    ros::spin();

    return 0;
}

