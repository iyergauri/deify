#include "ros/ros.h"
#include <std_msgs/String.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <depth_image_proc/depth_traits.h>
#include <sound_play/sound_play.h>
#include <unistd.h>

using namespace std;

ros::Publisher cmdpub;
ros::Subscriber depthSubscriber;
ros::Subscriber speechRecognitionSubscriber;
//sound_play::SoundClient sc;

bool done = false;
double turnDir = 1;

// Depth data from PCL
float currDepth = 999;
float depthX = 0;
float depthY = 0;

// Blob data for potential target blob from CMVision
double maxBlob = 0;
double center = 0.0;
double leftBlob = -1;
double rightBlob = -1;
double top = -1;
double bottom = -1;
double blobCount = 0;

// Thresholds for depth calculation
const float MIN_Y = 0.1;
const float MAX_Y = 0.5;
const float MIN_X = -0.2;
const float MAX_X = 0.2;
const float TOO_CLOSE = .90;



void sleepok(int t, ros::NodeHandle &nh)
 {
   if (nh.ok())
       sleep(t);
 }

void depthCallback(const sensor_msgs::ImageConstPtr& depthMsg) {
  ROS_INFO_STREAM("depth callback");
  currDepth = 999;

  // Process the x axis of the depth image to detect any obstacles' positions
  uint32_t imageWidth = depthMsg->width;
  float xRadiansPerPixel = 60.0/57.0/imageWidth;
  float sinPixelX[imageWidth];
  for (int x = 0; x < imageWidth; ++x) {
    sinPixelX[x] = sin((x - imageWidth/ 2.0)  * xRadiansPerPixel);
  }

  // Process the y axis of the depth image to detect any obstacles' positions
  uint32_t imageHeight = depthMsg->height;
  float yRadiansPerPixel = 45.0/57.0/imageWidth;
  float sinPixelY[imageHeight];
  for (int y = 0; y < imageHeight; ++y) {
    sinPixelY[y] = sin((imageHeight / 2.0 - y)  * yRadiansPerPixel);
  }

  const float* depthRow = reinterpret_cast<const float*>(&depthMsg->data[0]);
  int rowStep = depthMsg->step / sizeof(float);

  // Parse entire depth image to find closest object's depth
  for (int v = 0; v < (int) depthMsg->height; ++v, depthRow += rowStep) {
    for (int u = 0; u < (int) depthMsg->width; ++u) {
      float depth = depth_image_proc::DepthTraits<float>::toMeters(depthRow[u]);
      if (!depth_image_proc::DepthTraits<float>::valid(depth)) continue;
      float yVal = sinPixelY[v] * depth;
      float xVal = sinPixelX[u] * depth;

      // Store depth if obstacle is close enough and if it is the smallest one seen so far
      if ( yVal > MIN_Y && yVal < MAX_Y &&
      xVal > MIN_X && xVal < MAX_X) {
        if (depth < currDepth) {
          currDepth = depth;
          depthX = u;
          depthY = v;
        }
      }
    }
  }
  //ROS_INFO_STREAM(currDepth);

  geometry_msgs::Twist T;
  if (currDepth <= TOO_CLOSE) {
    ROS_INFO_STREAM("stop");
    T.linear.x = 0;
    cmdpub.publish(T);
  }
  else {
    ROS_INFO_STREAM("move");
    T.linear.x = 2;
    T.angular.z = 0;
    cmdpub.publish(T);
  }
}

void voiceCallback(const std_msgs::String recogMsg) {
   
  geometry_msgs::Twist T;
  ros::NodeHandle n;
  sound_play::SoundClient sc;

  if (recogMsg.data == "left") {
    ROS_INFO_STREAM("FOUND LEFT");
    // T.angular.z = 2;
    T.linear.x = 0.5;
    cmdpub.publish(T);
    sleepok(1, n);
    sc.say("Turning left");
    sleepok(2, n);
    
  }
  else if (recogMsg.data == "right") {
    ROS_INFO_STREAM("FOUND RIGHT");
    T.angular.z = -2;
    cmdpub.publish(T);
    sleepok(1, n);
    sc.say("Turning right");
    sleepok(2, n);
  }
  else {
    sleepok(1, n);
    sc.say("Please speak more clearly.");
    sleepok(2, n);
  }
}

void maneuver() {
  geometry_msgs::Twist T;
  if (currDepth <= TOO_CLOSE) {
    ROS_INFO_STREAM("stop");
    T.linear.x = 0;
    cmdpub.publish(T);
  }
  else {
    ROS_INFO_STREAM("move");
    T.linear.x = 2;
    T.angular.z = 0;
    cmdpub.publish(T);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "hello_ros");
  ros::NodeHandle n;
  ROS_INFO_STREAM("HELLO");

  sound_play::SoundClient sc;

  depthSubscriber = n.subscribe<sensor_msgs::Image>("camera/depth/image_rect", 1, &depthCallback);
  speechRecognitionSubscriber = n.subscribe("/recognizer/output", 100, &voiceCallback);
  cmdpub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
  ros::Rate rate(5);

  while (ros::ok()) {
    ros::spinOnce();
    //maneuver();
    //sleepok(1, n);
    //sc.say("Hello");
    //sleepok(2, n);
    rate.sleep();
  }
  
  return 0;
}
