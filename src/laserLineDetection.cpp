 /*
 @File: laserLineDetection.cpp
 @Author: Santosh
 @Description: Eventually to find hallways using laser data
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "laser_geometry/laser_geometry.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sstream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#define DEBUG
#define MAXROWS 1000
#define MAXCOLS 1000
#define RESOLUTION_FACTOR 2

int laserOriginX = MAXCOLS/2;
int laserOriginY = MAXROWS/2;
int shiftedOriginX = -MAXCOLS/2;
int shiftedOriginY = -MAXROWS/2;

using namespace cv;
using namespace std;

//laser_geometry::LaserProjection projector_;

void hough_lines(Mat img)
  {
    Mat dst, cdst;
    Canny(img, dst, 50, 200, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR);
    // float r_values[], theta_values[]; 
    // int count = 0;
    
    vector<Vec2f> lines;
    HoughLines(dst, lines, 1, CV_PI/180, 140, 0, 0 );

    ROS_INFO ("# lines : %lu", lines.size());

    // for( size_t i = 0; i < lines.size(); i++ )
    // {
    // 	float rtemp = lines[i][0], theta = lines[i][1];
    // 	rvalue[count] = lines[j][0];
    // 	theta_values[count] = lines[j][1];
    // 	for ( size_t j = i; j < lines.size(); j++ )
    // 	{
    // 		if (theta == lines[j][1])
    // 		{
    // 			if (abs(rtemp - lines[j][0])<=6.0)
    // 			{

    // 				rvalue[count] = lines[j][0];
    // 				theta_values[count] = lines[j][1];
    // 			}
    // 		}
    // 	}
    // }
    // float tempr = lines [0][0];
    // float temptheta = lines[0][1];

    int counter = 1;
    int max = 0;
    float modeOfTheta = lines[0][1];
    for (int pass = 0; pass < lines.size() - 1; pass++)
    {
        if ( lines[pass][1] == lines[pass+1][1] )
        {
            counter++;
            if ( counter > max )
            {
                max = counter;
                modeOfTheta = lines[pass][1];
            }
        } else
            counter = 1; // reset counter.
    }
    ROS_INFO ("mode of theta is %f", modeOfTheta);

    float comp = 0;
    int count = 0;
    float r[2] = {};

    for( size_t i = 0; i < lines.size(); i++ )
    {
      ROS_INFO ("I am in loop");
      float rho = lines[i][0], theta = lines[i][1];
      if ((theta == modeOfTheta) && abs(comp - rho) >=6 && count < 2)
      {
      	ROS_INFO("I am in condition");
      	r[count] = rho;
      	comp = rho;
      	count++;
      }
      //ROS_INFO ("r[%lu] = %f and theta[%lu] = %f", i, rho, i, theta);
      // Point pt1, pt2;
      // double a = cos(theta), b = sin(theta);
      // double x0 = a*rho, y0 = b*rho;
      // pt1.x = cvRound(x0 + 1000*(-b));
      // pt1.y = cvRound(y0 + 1000*(a));
      // pt2.x = cvRound(x0 - 1000*(-b));
      // pt2.y = cvRound(y0 - 1000*(a));
      // line( img, pt1, pt2, Scalar(0,0,255), 2, CV_AA);
    }
    ROS_INFO ("size of r : %lu", sizeof(r)/sizeof(float));

    for (int i = 0; i < sizeof(r)/sizeof(float); i++)
    {
    	ROS_INFO ("r values [%d] : %f", i, r[i]);
    	if (r[i] > 0)
    	{
			Point pt1, pt2;
			double a = cos(modeOfTheta), b = sin(modeOfTheta);
			double x0 = a*r[i], y0 = b*r[i];
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			line( img, pt1, pt2, Scalar(0,0,255), 2, CV_AA);
		}
    }
    imshow("detected lines", img);
    waitKey(3);
    // for( size_t i = 0; i < lines.size(); i++ )
    // {
    //   float rho = lines[i][0], theta = lines[i][1];
    //   ROS_INFO ("r[%lu] = %f and theta[%lu] = %f", i, rho, i, theta);
    //   Point pt1, pt2;
    //   double a = cos(theta), b = sin(theta);
    //   double x0 = a*rho, y0 = b*rho;
    //   pt1.x = cvRound(x0 + 1000*(-b));
    //   pt1.y = cvRound(y0 + 1000*(a));
    //   pt2.x = cvRound(x0 - 1000*(-b));
    //   pt2.y = cvRound(y0 - 1000*(a));
    //   line( img, pt1, pt2, Scalar(0,0,255), 2, CV_AA);
    // }
    // imshow("detected lines", img);
    // waitKey(3);
  }

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laserMsg)
{
	//ROS_INFO("I am called!");
	Mat image(MAXROWS,MAXCOLS, CV_8UC1, Scalar(255));
	if ( !image.data )
    {
        printf("No image data \n"); 
    }
	// sensor_msgs::PointCloud2 cloud;
	// cloud.header.frame_id = "sensor_frame";
	//ros::NodeHandle n;
	//ros::Publisher cloud_publisher = n.advertise<sensor_msgs::PointCloud2>("/cloud", 100);
	
	// projector_.projectLaser(*laserMsg, cloud);
	// cloud_publisher.publish(cloud);
	

#ifdef DEBUG
	ROS_INFO("Frame id : [%s]", laserMsg->header.frame_id.c_str());
	ROS_INFO("Angle Min :%f", laserMsg->angle_min);
	ROS_INFO("Angle Max :%f", laserMsg->angle_max);
	ROS_INFO("Angle Increment :%f", laserMsg->angle_increment);
	ROS_INFO("Scan Time :%f", laserMsg->scan_time);
	ROS_INFO("Max range :%f", laserMsg->range_max);
	ROS_INFO("Min range :%f", laserMsg->range_min);
	
	for (int i = 0; i < laserMsg->ranges.size(); i = i + 5)
	{
	/* code */
		ROS_INFO("Just printing ranges [%d] : %f", i, laserMsg->ranges[i] );
	}
	for (int i = 0; i < laserMsg->intensities.size(); i = i + 5)
	{
	/* code */
		ROS_INFO("Just printing intensities [%d] : %f", i, laserMsg->intensities[i] );
	}
#endif 
	double staringAngle = laserMsg->angle_min;
	double endingAngle = laserMsg->angle_max;
	double angleIncrement = laserMsg->angle_increment;
	for (int i = 0; i < laserMsg->ranges.size(); ++i)
	{
		/* code */
		double currentAngle = staringAngle + i*angleIncrement;
		double r = laserMsg->ranges[i]*100; //range in centemeters
		//ROS_INFO ("At current angle = %f, the range is %f", currentAngle, r);
		int x = r * cos(currentAngle);
		int y = r * sin(currentAngle);
		//ROS_INFO ("The X = %d and Y = %d when the origin is at the base laser \n r = %f and current angle = %f ", x, y, r, currentAngle);
		int shiftedX = abs((x - shiftedOriginX)/RESOLUTION_FACTOR);
		int shiftedY = abs((y - shiftedOriginY)/RESOLUTION_FACTOR);
		if ((shiftedX <= MAXCOLS && shiftedX >= 0) && (shiftedY <= MAXROWS && shiftedX >= 0))
		{
			/* code */
			image.at<uchar>(shiftedY, shiftedX) = 0;
		}
		
	}

    /*For testing purpose only*/

    // for (int i = 0; i < image.rows; ++i)
    // {
    // 	/* code */
    // 	uchar *p = image.ptr(i);
    // 	for (int j = 0; j < image.cols; ++j)
    // 	{
    // 		/* code */
    		
    // 		if (j == i)
    // 		{
    // 			/* code */
    // 			p[j] = 0;
    // 		}
    // 	}
    // }

    hough_lines(image);
    // namedWindow("Display Image", WINDOW_AUTOSIZE );
    // imshow("Display Image", image);
    // waitKey(0);
}



 int main(int argc, char **argv)
 {
 ros::init(argc, argv, "laserLineDetection");
 ros::NodeHandle n;
 
 ros::Subscriber laser = n.subscribe("/base_scan", 100, laserCallBack);

 ros::Rate rate(10);
 while (ros::ok())
 	{
 		#ifdef DEBUG
 		ROS_INFO("Hurray! I am running!");
 		#endif
 		ros::spinOnce();
    	rate.sleep();
 	}
return 0;
 }