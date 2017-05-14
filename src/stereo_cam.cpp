// File:   stereo_cam.cpp
// Author: Raghava Kumar
// Date:   5/13/2016

#include <stdlib.h>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "stereo_cam");
	ros::NodeHandle n;

	// Publishing on topic 'scan' at a 50Hz rate
	ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan",100);
	ros::Rate loop_rate(50);

	int count = 0;
	while(ros::ok()){
		// Create and populate message
		sensor_msgs::LaserScan scan;

		// Header
		scan.header.stamp = ros::Time::now();
		scan.header.frame_id = "stereo_cam";

		// The following parameters were adapted from the Sick LMS111,
		// converting its range from 270 to 360 degrees

		// 360 degrees, 0.5 degree increment
		scan.angle_min = -3.14159;
		scan.angle_max = 3.14159;
		scan.angle_increment = 0.00872665;

		// 50Hz scan rate
		scan.scan_time = 0.02;

		// 0.1 <= range <= 30 (m)
		scan.range_min = 0.1;
		scan.range_max = 30.0;

		// Actual data -- ranges only
		scan.ranges = std::vector<float>(720,0);

		for(int i = 0; i < 720; i++){
			// Everything is approximately 5m away
			scan.ranges.data()[i] = ((float)rand()/ RAND_MAX)*0.2 + 5.0;
		}

		pub.publish(scan);

		// Debug
		ROS_INFO("Scanning! Count = %d", count);

		// Limit publishing rate
		loop_rate.sleep();
		
		count++;
	}
}
