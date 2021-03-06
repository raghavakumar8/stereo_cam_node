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

unsigned char wr_buf[8*WIDTH*HEIGHT];
unsigned char rd_buf[4*WIDTH*HEIGHT];

// Function Prototypes
void allwrite(int fd, unsigned char *buf, int len);
void allread(int fd, unsigned char *buf, int len);

void prep_send_buf(unsigned char *buf, int x, int y, unsigned char l, unsigned char r);
void parse_recv_buf(unsigned char *buf, int *x, int *y, unsigned char *s);

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	// Send this to the xillybus module
	int lin_addr, lin_addr_l, lin_addr_r;

	for (int y = 0; y < HEIGHT; y++){
		for (int x = 0; x < WIDTH; x++){
			lin_addr = y*WIDTH + x;
			lin_addr_l = y*WIDTH*2 + x;
			lin_addr_r = lin_addr_l + WIDTH;
			prep_send_buf(wr_buf + (lin_addr*8), x, y, (* msg).data.data()[lin_addr_l], (* msg).data.data()[lin_addr_r]);
		}
	}

	allwrite(fw, wr_buf, 8*WIDTH*HEIGHT);

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
	read(fr, rd_buf, 4*WIDTH*HEIGHT);

	int x, y;
	unsigned char s;

	for (int i = 0; i < WIDTH*HEIGHT; i++){
		parse_recv_buf(rd_buf + i*4, &x, &y, &s);
		img.data.data()[y*WIDTH + x] = s;
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
    rc = read(fd, buf + recd, len - recd);

    if ((rc < 0) && (errno == EINTR))
      continue;

    if (rc < 0) {
      perror("allread() failed to read");
      exit(1);
    }

    recd += rc;
  }
}

void prep_send_buf(unsigned char *buf, int x, int y, unsigned char l, unsigned char r) {
  buf[0] = x&0xFF;
  buf[1] = (x>>8)&0xFF;
  buf[2] = y&0xFF;
  buf[3] = (y>>8)&0xFF;

  buf[4] = r;
  buf[5] = l;
  buf[6] = 0;
  buf[7] = 0x80;
}

void parse_recv_buf(unsigned char *buf, int *x, int *y, unsigned char *s) {
  *s = buf[0];
  *x = buf[1] + ((buf[2]&0x03)<<8);
  *y = (buf[2]>>2) + ((buf[3]&0x0F)<<6);
}

