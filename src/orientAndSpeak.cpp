/************************
 * Filename: orientAndSpeak.cpp
 * 
 * Students:
 * - Gauri Iyer
 * - Christopher D'Ambrosia
 * 
 * Final Project
 * 
 * Description: This file defines the behavior for the deify node in
 * the deify ROS package. This node navigates a Turtlebot that is 
 * encouraging human engagement in DEI themes. When the Turtlebot 
 * locates a participant, it will both orient its "gaze" (the laptop
 * screen) and vocalize to invite participation. 
 * 
 * Instructions:
 * (Don't forget to source devel/setup.bash in every terminal!)
 * In 4 terminals, run:
 *    1. roscore
 *    2. roslaunch turtlebot_bringup minimal.launch
 *    3. roscd astra_launch
 *       roslaunch astra_launch astra_pro.launch
 *    4. roscd sound_play
 *       roslaunch sound_play soundplay_node.launch
 * In a 5th terminal, inside the workspace containining deify, run:
 *    - catkin_make
 *    - rosrun deify orientAndSpeak
 ************************/

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <depth_image_proc/depth_traits.h>
#include <sound_play/sound_play.h>
#include <unistd.h>

using namespace std;

// Node publisher + PCL and Speech subscribers
ros::Publisher cmdpub;
ros::Subscriber depthSubscriber;
ros::Subscriber speechRecognitionSubscriber;

// Depth data from PCL
float currDepth = 999;

// Thresholds for depth calculation
const float MIN_Y = 0.1;
const float MAX_Y = 0.5;
const float MIN_X = -0.2;
const float MAX_X = 0.2;
const float X_SCALE = 10.0;
const float Z_SCALE = 2.0;
const float NEAR_PERSON = 0.6;
const float TOO_FAR = 2.0;

// Keep track of wait state so robot doesn't run away from humans when
// it's not supposed to.
bool wait = false;

/************************
 * void sleepok(int t, ros::NodeHandle &nh)
 * 
 * Purpose: sleep while the Turtlebot "says" things, so that it doesn't
 * accidentally interpret its' own speech as commands.
 * 
 * @param int t - number of seconds to sleep for
 * @param ros::NodeHandle &n - ensures the handle is still active
 * 
 * @return none
 ************************/
void sleepok(int t, ros::NodeHandle &n) {
  if (n.ok())
    sleep(t);
}

/************************
 * Name: void depthCallback(const sensor_msgs::ImageConstPtr& depthMsg)
 *
 * Purpose: This function detects the closest object to the TurtleBot based 
 * on the Astra camera's point cloud.
 *
 * @param const sensor_msgs::ImageConstPtr& depthMsg - contains the depth 
 * information from the Point Cloud Library node.
 *
 * @return none
************************/
void depthCallback(const sensor_msgs::ImageConstPtr &depthMsg) {
  // ROS_INFO_STREAM("depth callback");
  sound_play::SoundClient sc;
  ros::NodeHandle n;
  currDepth = 999;

  // Process the x axis of the depth image to detect any obstacles' positions
  uint32_t imageWidth = depthMsg->width;
  float xRadiansPerPixel = 60.0 / 57.0 / imageWidth;
  float sinPixelX[imageWidth];
  for (int x = 0; x < imageWidth; ++x) {
    sinPixelX[x] = sin((x - imageWidth / 2.0) * xRadiansPerPixel);
  }

  // Process the y axis of the depth image to detect any obstacles' positions
  uint32_t imageHeight = depthMsg->height;
  float yRadiansPerPixel = 45.0 / 57.0 / imageWidth;
  float sinPixelY[imageHeight];
  for (int y = 0; y < imageHeight; ++y) {
    sinPixelY[y] = sin((imageHeight / 2.0 - y) * yRadiansPerPixel);
  }

  // X,Y,Z of the centroid
  float x = 0.0;
  float y = 0.0;
  float z = 1e6;

  // Number of points observed
  unsigned int pts = 0;

  const float *depthRow = reinterpret_cast<const float *>(&depthMsg->data[0]);
  int rowStep = depthMsg->step / sizeof(float);

  // Parse entire depth image to find avg depth
  for (int v = 0; v < (int)depthMsg->height; ++v, depthRow += rowStep) {
    for (int u = 0; u < (int)depthMsg->width; ++u) {
      float depth = depth_image_proc::DepthTraits<float>::toMeters(depthRow[u]);
      if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > TOO_FAR)
        continue;
      float yVal = sinPixelY[v] * depth;
      float xVal = sinPixelX[u] * depth;

      // Store depth if obstacle is close enough and if it is the smallest one seen so far
      if (yVal > MIN_Y && yVal < MAX_Y &&
          xVal > MIN_X && xVal < MAX_X) {
        x += xVal;
        y += yVal;
        z = std::min(z, depth);
        pts++;

        if (depth < currDepth)
          currDepth = depth;
      }
    }
  }

  geometry_msgs::Twist T;

  // Actions change depending on current/past state of robot.
  // Make sure there are enough points to actually detect object.
  if (pts > 4000) {
    x /= pts;
    y /= pts;

    // Case 1: Robot is newly obstructed by a person in the way.
    // Ask the person to start our survey.
    if (currDepth <= NEAR_PERSON and !wait)
    {
      wait = true;
      T.linear.x = 0;
      cmdpub.publish(T);
      ROS_INFO_STREAM("Do you have time for a quick survey?");
      sc.say("Do you have time for a quick survey?");
    }

    // Case 2: Robot is still near the person, and has already announced
    // its presence. Hang out until the person's done with the study.
    // else if (currDepth <= NEAR_PERSON and wait)
    // {
    //   // ROS_INFO_STREAM("Hanging out :-)");
    // }

    // Case 3: Robot is too far from anyone.
    else if (currDepth > TOO_FAR) {
      ROS_INFO_STREAM("points detected, but robot is too far from anything.");
      T.linear.x = 0;
      cmdpub.publish(T);
      wait = false;
    }

    // Case 4: Robot found a person to move towards.
    else
    {
      ROS_INFO_THROTTLE(1, "Turning towards centroid at x = %f", x);
      T.angular.z = -x * X_SCALE;
      cmdpub.publish(T);
    }
  }

  // Case 5: Not enough points, so just stop.
  else {
    ROS_INFO_STREAM("not enough points, stopping");
    T.linear.x = 0;
    T.angular.z = 0;
    cmdpub.publish(T);
  }
}

/************************
 * Name: int main(int argc, char** argv)
 *
 * Purpose: This function initiates the Turtlebot's survey journey
 * receiving camera metrics.
 *
 * @param int argc - unused
 * @param char** argv - unused
 *
 * @return 0 (exit on success)
************************/
int main(int argc, char **argv) {
  ros::init(argc, argv, "hello_ros");
  ros::NodeHandle n;
  ROS_INFO_STREAM("HELLO");

  sound_play::SoundClient sc;

  depthSubscriber = n.subscribe<sensor_msgs::Image>("camera/depth/image_rect", 10, &depthCallback);
  cmdpub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

  ros::Rate rate(5);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
