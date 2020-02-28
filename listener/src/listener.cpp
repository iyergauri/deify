/************************
 * Filename: listener.cpp
 * 
 * Students:
 * - Gauri Iyer
 * - Sam Scudder
 * - Shreeman Hariharan
 * 
 * HW 11: Implementation and Robot-Focused Testing
 * 
 * Description: This file defines the behavior for the listener node in
 * the listener ROS package. This node navigates a Turtlebot that is 
 * trying to escape from humanity by asking for help. The Turtlebot only
 * has one automated task: move forward. Humans are required to remove
 * obstacles from its path and recommend which way to turn.
 * 
 * Instructions:
 * (Don't forget to source devel/setup.bash in every terminal!)
 * In 5 terminals, run:
 *    1. roscore
 *    2. roslaunch turtlebot_bringup minimal.launch
 *    3. roslaunch astra_launch astra_pro.launch
 *    4. roslaunch sound_play soundplay_node.py
 *    5. roslaunch pocketsphinx turtlebot_voice_cmd.launch
 * In a 6th terminal, inside the workspace containining listener, run:
 *    - catkin_make
 *    - rosrun listener listener
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
float depthX = 0;
float depthY = 0;

// Thresholds for depth calculation
const float MIN_Y = 0.1;
const float MAX_Y = 0.5;
const float MIN_X = -0.2;
const float MAX_X = 0.2;
const float TOO_CLOSE = 0.8;

// Keep track of blocked state so robot can ask for help
bool blocked = false;
bool justTurned = false;
bool announceFreedom = false;

/************************
 * void sleepok(int t, ros::NodeHandle &nh)
 * 
 * Purpose: sleep while the Turtlebot "says" things, so that it doesn't
 * accidentally interpret its' own speech as commands.
 * 
 * @param int t - number of seconds to sleep for
 * @param ros::NodeHandle &nh - ensures the handle is still active
 * 
 * @return none
 ************************/
void sleepok(int t, ros::NodeHandle &nh){
   if (nh.ok())
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
void depthCallback(const sensor_msgs::ImageConstPtr& depthMsg) {
  // ROS_INFO_STREAM("depth callback");
  sound_play::SoundClient sc;
  ros::NodeHandle n;
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
  

  // Actions change depending on current/past state of robot.
  // Case 1: Robot is newly obstructed. Announce that robot is stuck, and 
  // ask for help.
  geometry_msgs::Twist T;
  if (currDepth <= TOO_CLOSE and !blocked) {
    blocked = true;
    T.linear.x = 0;
    cmdpub.publish(T);
    sc.say("Please help me leave!");
    // sc.say("Help, I'm stuck! Could you remove the obstacle, or tell me to go left or right?");
    sleepok(7, n);
  }

  // Case 2: Robot is previously obstructed, and has already announced that it
  // needed help. Do nothing; wait for human to help!
  else if (currDepth <= TOO_CLOSE and blocked) {
    // do nothing
  } 

  // Case 3: Robot is not obstructed.
  else {

    // Case 3A: Robot was previously obstructed - thank the human!
    if (blocked) {
      sleepok(2, n);
      if (justTurned) {
        sc.say("Thank you for unblocking me!");
        sleepok(2, n);
      }
      else {
        announceFreedom = true;
      }
    }

    // Move forward
    justTurned = false;
    blocked = false;
    T.linear.x = 0.3;
    T.angular.z = 0;
    if (announceFreedom) {
      sc.say("I'm freeeeeeeee!");
      sleepok(2, n); 
      announceFreedom = false;
    }
    cmdpub.publish(T);


  }
}

/************************
 * Name: void voiceCallback(const std_msgs::String recogMsg)
 *
 * Purpose: This function detects human voice commands, limited to
 * "left" and "right".
 *
 * @param const std_msgs::String recogMsg - contains the audio  
 * information as text from the pocketsphinx node.
 *
 * @return none
************************/
void voiceCallback(const std_msgs::String recogMsg) {
  geometry_msgs::Twist T;
  ros::NodeHandle n;
  sound_play::SoundClient sc;

  if (recogMsg.data == "left") {
    justTurned = true;
    ROS_INFO_STREAM("FOUND LEFT");
    T.angular.z = 2;
    cmdpub.publish(T);    
  }

  else if (recogMsg.data == "right") {
    justTurned = true;
    ROS_INFO_STREAM("FOUND RIGHT");
    T.angular.z = -2;
    cmdpub.publish(T);
  }

  else {
    justTurned = false;
    // sleepok(1, n);
    sc.say("What?");
    sleepok(2, n);
  }
}

/************************
 * Name: int main(int argc, char** argv)
 *
 * Purpose: This function initiates the Turtlebot's escape by 
 * receiving camera and voice metrics.
 *
 * @param int argc - unused
 * @param char** argv - unused
 *
 * @return 0 (exit on success)
************************/
int main(int argc, char** argv) {
  ros::init(argc, argv, "hello_ros");
  ros::NodeHandle n;
  ROS_INFO_STREAM("HELLO");

  sound_play::SoundClient sc;

  depthSubscriber = n.subscribe<sensor_msgs::Image>("camera/depth/image_rect", 10, &depthCallback);
  speechRecognitionSubscriber = n.subscribe("/recognizer/output", 100, &voiceCallback);
  cmdpub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
  
  ros::Rate rate(5);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
