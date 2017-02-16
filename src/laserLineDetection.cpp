 /*
 @File: laserLineDetection.cpp
 @Author: Santosh
 @Description: To find hallways using laser data
 */
#include <iostream>
#include <vector>
#include <string>
#include <stdint.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "laser_geometry/laser_geometry.h"
//#include <pcl/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>

#include "hallway/hallwayMsg.h"
#include "hallway/markerMsg.h"
#include "geometry_msgs/Point.h"

//#define DEBUG
#define MAXROWS 700
#define MAXCOLS 700
#define RESOLUTION_FACTOR 3

int laserOriginX = MAXCOLS/2;
int laserOriginY = MAXROWS/2;
int shiftedOriginX = -MAXCOLS*RESOLUTION_FACTOR/2;
int shiftedOriginY = -MAXROWS*RESOLUTION_FACTOR/2;

uint8_t debugCount = 0;

geometry_msgs::Point p1, p2;

using namespace cv;
using namespace std;

string ss;

ros::Publisher hallwayPublisher;
ros::Publisher marker_pub;

//laser_geometry::LaserProjection projector_;
void publishHallwayData(vector<float> r, float modeOfTheta);
void hough_lines(Mat img);
//void hallwayMarkers(geometry_msgs::Point p1, geometry_msgs::Point p2);

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laserMsg)
{
	ss = laserMsg->header.frame_id.c_str();
	//ROS_INFO("I am called!");
	Mat image(MAXROWS,MAXCOLS, CV_8UC1, Scalar(255));
	Mat flipped;
	for (int i = -2; i < 3; ++i)
	{
		for (int j = -2; j < 3; ++j)
		{
			image.at<uchar>(laserOriginY+j, laserOriginX+i) = 0;
		}
	}
		for (int i = 3; i < 6; ++i)
		{
			image.at<uchar>(laserOriginY, laserOriginX+i) = 0;
		}
	
	
	if ( !image.data )
    {
        printf("No image data \n"); 
    }
	

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
	// Convert LaserScan messages into an Image
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
	flip (image, flipped, 0);
    hough_lines(flipped);
    //hough_lines(image);
    // namedWindow("Display Image", WINDOW_AUTOSIZE );
    // imshow("Display Image", image);
    // waitKey(0);
}



 int main(int argc, char **argv)
 {
 	ros::init(argc, argv, "laserLineDetection");
 	ros::NodeHandle n;
 	hallwayPublisher = n.advertise<hallway::hallwayMsg>("hallway_data", 100);
 	ros::Subscriber laser = n.subscribe("/base_scan", 100, laserCallBack);
    marker_pub = n.advertise<hallway::markerMsg>("hallway_marker_points", 100);
  

 	ros::spin();
	return 0;
 }

 void hough_lines(Mat img)
  {
    Mat dst, cdst;
    Canny(img, dst, 50, 200, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR); 
    float modeOfTheta;
    float comp = 0;
    int count = 0;   
    vector<float> r;
    vector<Vec2f> lines;
   

    HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

    //ROS_INFO ("# lines : %lu", lines.size());

    int counter = 1;
    int max = 0;
    if (lines.size() > 0)
    {
    	/* code */
    	modeOfTheta = lines[0][1];
    	//to find mode of the theta values which will be the slope of the hallway
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
    //ROS_INFO ("mode of theta is %f", modeOfTheta);
    }
       

    // To collect the two r values of the lines with slope as modeOfTheta (only the hallway lines)

    for( size_t i = 0; i < lines.size(); i++ )
    {
      //ROS_INFO ("I am in loop");
      float rho = lines[i][0], theta = lines[i][1];
      if ((theta == modeOfTheta) && abs(comp - rho) >= 6 && count < 2)
      {
      	//ROS_INFO("I am in condition");
      	r.push_back(rho);
      	comp = rho;
      	count++;
      }
    }
    //ROS_INFO ("size of r : %lu", r.size());

    for (int i = 0; i < r.size(); i++)
    {
    	//ROS_INFO ("r values [%d] : %f", i, r[i]);
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

   	publishHallwayData (r, modeOfTheta);
   	//ROS_INFO ("I am here! : %d", debugCount++);
    imshow("Hallway", img);
    waitKey(10);
    
    /*
   	Original hough transform that detects lines ands their duplicates
   	*/

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


void publishHallwayData(vector<float> r, float modeOfTheta)
{
	hallway::hallwayMsg msg;
    //stringstream ss;
    std::vector<double> slope;
    std::vector<double> yIntercept;
    //ss << "base_link";
    std::vector<geometry_msgs::Point> markerPointsL;
    std::vector<geometry_msgs::Point> markerPointsR;

    hallway::markerMsg markMsg;
    

    
    // ROS_INFO ("theta : %f", modeOfTheta);
    // for (int i = 0; i < r.size(); ++i)
    // {
    // 	ROS_INFO ("rvalue [%d] is %f", i, r[i]);
    // }
    
    if (r.size() == 2) //if hallway 
    {
    	for (int i = 0; i < r.size(); i++)
    	{
    	//ROS_INFO ("r values [%d] : %f", i, r[i]);
    		if (r[i] > 0)
    		{
				Point pt1, pt2;
				double a = cos(modeOfTheta), b = sin(modeOfTheta);
				double x0 = a*r[i], y0 = b*r[i];
				int backToBaseX = (x0 - laserOriginX)*RESOLUTION_FACTOR;
				int backToBaseY = (y0 - laserOriginY)*RESOLUTION_FACTOR;
				pt1.x = cvRound(backToBaseX + 1000*(-b));
				pt1.y = cvRound(backToBaseY + 1000*(a));
				pt2.x = cvRound(backToBaseX - 1000*(-b));
				pt2.y = cvRound(backToBaseY - 1000*(a));
				//line( img, pt1, pt2, Scalar(0,0,255), 2, CV_AA);
				if (pt2.x != pt1.x)// to avoid floating point exception (core dump)
				{
					ROS_INFO ("i value : %d", i);
					//ROS_INFO ("RAW Point 1 :-: x: %d, y: %d", pt1.x, pt1.y);
					//ROS_INFO ("RAW Point 2 :-: x: %d, y: %d", pt2.x, pt2.y);
                    //- to eliminate the mirroring of the markers
                    p1.x = -pt1.x*0.01; // I don't remember why 0.01, Guess m to cm conversion.
                    p1.y = pt1.y*0.01;
                    p1.z = 0;
                    p2.x = -pt2.x*0.01;
                    p2.y = pt2.y*0.01;
                    p2.z = 0;
                    ROS_INFO ("Point 1 :-: x: %f, y: %f", p1.x, p1.y);
                    ROS_INFO ("Point 2 :-: x: %f, y: %f", p2.x, p2.y);
                    if (i == 0)
                    {
                        markerPointsL.push_back(p1);
                        markerPointsL.push_back(p2);
                    }
                    else if (i == 1)
                    {
                        markerPointsR.push_back(p1);
                        markerPointsR.push_back(p2);
                    }
                    // hallwayMarkers(p1, p2);
					slope.push_back((float)(pt2.y - pt1.y)/(float)(pt2.x - pt1.x));
					yIntercept.push_back(pt1.y - slope[i]*pt1.x);// * RESOLUTION_FACTOR);
					ROS_INFO ("slope: %f and Y-Intercept: %f", slope[i], yIntercept[i]);
				}
			}
    	}
        //Check to see if there is any valid data. if not core dump will occur. 
        //publish hallway data
    	if (slope.size() > 0 && yIntercept.size() > 0)
    	{
    		msg.frame_id = ss;
    		msg.slope_hallwayL = slope[0];
    		msg.intercept_hallwayL = yIntercept[0];
    		msg.width_hallway = abs(r[0]-r[1]) * RESOLUTION_FACTOR;
            msg.hallwayPointL1 = p1;
            msg.hallwayPointL2 = p2;

            if (markerPointsL.size() == 2)
            {
                markMsg.pointL1 = markerPointsL[0];
                markMsg.pointL2 = markerPointsL[1];
            }
            if (markerPointsR.size() == 2)
            {
                markMsg.pointR1 = markerPointsR[0];
                markMsg.pointR2 = markerPointsR[1];
            }
            ROS_INFO("frame_id is [%s]", msg.frame_id.c_str());
    		hallwayPublisher.publish(msg);
            marker_pub.publish(markMsg);
    	}
    	
    }

}

