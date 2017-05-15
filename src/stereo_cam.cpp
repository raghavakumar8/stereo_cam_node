// File:   stereo_cam.cpp
// Author: Raghava Kumar
// Date:   5/13/2016

#include <stdlib.h>
#include <vector>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termio.h>
#include <signal.h>

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "sensor_msgs/LaserScan.h"

#define WIDTH 447
#define HEIGHT 370

// Global Variables
int fr, fw;

image_transport::Subscriber im_sub;
image_transport::Publisher im_pub;

unsigned char wr_buf[WIDTH*HEIGHT];
unsigned char rd_buf[WIDTH*HEIGHT];

// Function Prototypes
void allwrite(int fd, unsigned char *buf, int len);
void allread(int fd, unsigned char *buf, int len);

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	ROS_INFO("Recieved!");
	
	// Send this to the xillybus module
	for (int i = 0; i < WIDTH*HEIGHT; i++){
		wr_buf[i] = (* msg).data.data()[i];
	}

	//allwrite(fw, wr_buf, WIDTH*HEIGHT);

	// Generate Image
	sensor_msgs::Image img;

	img.header.stamp = ros::Time::now();
	img.header.frame_id = "stereo_cam";

	img.height = HEIGHT;
	img.width = WIDTH;
	
	img.encoding = "mono8";
	img.is_bigendian = 0;
	
	img.step = WIDTH;
        img.data = std::vector<uint8_t>(WIDTH*HEIGHT);

	// Fill image by reading from xillybus module
	//allread(fr, rd_buf, WIDTH*HEIGHT);

	for (int i = 0; i < WIDTH*HEIGHT; i++){
		img.data.data()[i] = 255;
		//img.data.data()[i] = rd_buf[i];	
	}
	
	im_pub.publish(img);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "stereo_cam");
	ros::NodeHandle n;
	
	// Open read/write files 
	fw = open("/dev/xillybus_write_32", O_WRONLY);
	fr = open("/dev/xillybus_read_32", O_RDONLY);

	// Error checking
	if (fw < 0) ROS_INFO("Could not open xillybus_write_32");
	if (fr < 0) ROS_INFO("Could not open xillybus_read_32");

	image_transport::ImageTransport it(n);

	// Subscribe to camera images, publish stereo images 
	im_sub = it.subscribe("camera/image", 1, imageCallback);	
	im_pub = it.advertise("stereo/image", 1);

	// Wait for callbacks!
	ros::spin();

	// Close read/write files
	close(fw);
	close(fr);

	/*ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan",100);
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
		
		// Get callbacks	
		ros::spinOnce();
		
		// Limit publishing rate
		loop_rate.sleep();
		
		count++;
	}*/
}

void allwrite(int fd, unsigned char *buf, int len) {
  int sent = 0;
  int rc;

  while (sent < len) {
    rc = write(fd, buf + sent, len - sent);

    if ((rc < 0) && (errno == EINTR))
      continue;

    if (rc < 0) {
      perror("allwrite() failed to write");
      exit(1);
    }

    if (rc == 0) {
      fprintf(stderr, "Reached write EOF (?!)\n");
      exit(1);
    }

    sent += rc;
  }
}

void allread(int fd, unsigned char *buf, int len) {
  int recd = 0;
  int rc;

  while (recd < len) {
    rc = write(fd, buf + recd, len - recd);

    if ((rc < 0) && (errno == EINTR))
      continue;

    if (rc < 0) {
      perror("allread() failed to read");
      exit(1);
    }

    recd += rc;
  }
}
