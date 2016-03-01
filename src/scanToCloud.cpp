#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/PointField.h"
#include <sensor_msgs/Image.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h> 
#include <boost/foreach.hpp>

#define R_VALUE 3000 // 30 METERS RANGE SO 3000 WILL GIVE 1 CM ACCURACY
#define THETA_VALUE 360 // 

int accumulator [R_VALUE][THETA_VALUE] = {};
//int *accuPtr = &accumulator[R_VALUE][THETA_VALUE/2];

unsigned long int temp = 0;
int max = 0;
float houghR = 0;
float houghTheta = 0;

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  ros::Publisher cloudxyz_;
  ros::Publisher image_pub_;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "base_scan", 10),
    laser_notifier_(laser_sub_,listener_, "base_laser_link", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/my_cloud",1);
    cloudxyz_ = n_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/my_cloudxyz",1);
    image_pub_ = n_.advertise<sensor_msgs::Image> ("/image_topic", 30);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZRGB> cloudp;
    sensor_msgs::Image image;
    
    //ROS_INFO("Angle Min :%f", scan_in->angle_min);
    //ROS_INFO("Angle Max :%f", scan_in->angle_max);
  
    try
    {
        projector_.transformLaserScanToPointCloud(
          "base_laser_link",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
    
    

    scan_pub_.publish(cloud);
    
   
  
    
    pcl::fromROSMsg(cloud, cloudp);
    // for (long unsigned int i = 0; i < cloudp.points.size(); ++i)
    // {
    // 	cloudp.points[i].r = 255;
    //   cloudp.points[i].g = 255;
    //   cloudp.points[i].b = 255;
    // }
	  pcl::toROSMsg (cloudp, image); //convert the cloud
    //ROS_INFO("size : %lu",cloudp.points.size());
    
   
// 	// ROS_INFO ("accumulator zeroed : %lu", temp);
//     for (long unsigned int i = 0; i < cloudp.points.size(); ++i)
//     {
//       /* code */
//       // ROS_INFO ("index %lu", i);
//       // ROS_INFO ("\t(%f, %f)\n", cloudp.points[i].x, cloudp.points[i].y);//, cloudp.points[i].z);
      
//       double x = cloudp.points[i].x;
//       double y = cloudp.points[i].y;
//       double theta_radians = atan2(y, x);
//       double r_meters = x * cos(theta_radians) + y * sin(theta_radians);
//       int r_cm = round(r_meters * 100);
//       int theta_degrees = round((theta_radians * 180) /3.14); //180 is actual formula but I want more resolution to vote accurately
//       //ROS_INFO ("r_cm: %d, r_meters: %f, theta_radians: %f, theta_degrees: %d", r_cm, r_meters, theta_radians, theta_degrees);
//       accumulator[r_cm][theta_degrees] = accumulator[r_cm][theta_degrees] + 1;

//      }
  	
//     //Finding local maxima
//     for (int i = 0; i < R_VALUE; ++i)
//     {
//     	/* code */
//     	for (int j = 0; j < THETA_VALUE; ++j)
//     	{
//     		/* code */
//     		if (accumulator[i][j] > max)
//     		{
//     			max = accumulator[i][j];
//     			houghR = i/100.0; //back to meters
//     			houghTheta = (j * 3.14)/180.0; //back to radians

//     		}

//     	}
//     }


// ROS_INFO ("Hough Paramenters max : %d, r : %f, theta %f", max, houghR, houghTheta);
// max = 0;
//     //ROS_INFO ("%d", max(accumulator));
//     for (int i = 0; i < R_VALUE; ++i)
//     {
//       /* code */
//       for (int j = 0; j < THETA_VALUE; ++j)
//       {
//         /* code */
//         // if (accumulator[i][j] > 0)
//         // 	ROS_INFO ("accumulator : %d", accumulator[i][j]);
//         temp = temp + accumulator[i][j];
//         accumulator[i][j] = 0;
        
//       }
//     }
//     //ROS_INFO ("accumulator current total value : %lu", temp);
//  //    temp = 0;
	
// 	for (long unsigned int i = 0; i < cloudp.points.size(); ++i)
//     {
//       float x = cloudp.points[i].x;
//       float y = cloudp.points[i].y;
//     	if ( ((x * cos(atan2(y,x)) + y * sin(atan2(y,x))) >= houghR - 1 ) && ((x * cos(atan2(y,x)) + y * sin(atan2(y,x))) <= houghR + 1))
//     	{
//     		//ROS_INFO ("%f", round(x * cos(atan2(y,x)) + y * sin(atan2(y,x))));
//     		cloudp.points[i].x = 255;
//       	cloudp.points[i].y = 255;
//     	}
//     	else
//     	{
//     		//ROS_INFO ("I am in");
//         cloudp.points[i].x = 255;
//         cloudp.points[i].y = 255;
//     	}
//     }    

    cloudxyz_.publish(cloudp);
    image_pub_.publish (image); //publish our cloud image
  
   }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;
}
