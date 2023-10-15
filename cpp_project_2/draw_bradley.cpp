
#include <cstdlib> // Use for the absolute value method abs()
#include <iostream> // Use for in out stream
#include "ros/ros.h" // Used to connect to ros
#include "geometry_msgs/Twist.h" // Used to establish linear and angular velocity like before
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation
#include "turtlesim/Pose.h" // Used to find current pose

using namespace std;//Set namespace to std

geometry_msgs::Twist velCommand; // Linear and angular velocity in m/s 
geometry_msgs::Pose2D current; // Used to find the current x and y position of the turtle
geometry_msgs::Pose2D desired; // Used to establish the desired x and y position of the turtle

const double distanceTolerance = 0.1; // Used to say how close in meters is okay to say the turtle is at the goal
 
// Initializes Twist variables (x, y, and z linear and angular velocity to 0 to start)
void setup() {
  velCommand.linear.x = 0.0;
  velCommand.linear.y = 0.0;
  velCommand.linear.z = 0.0;
  velCommand.angular.x = 0.0;
  velCommand.angular.y = 0.0;
  velCommand.angular.z = 0.0;
}
 
// Used to get the distance between the current x coordinate and the desired x coordinate
double getDistanceToGoal() {
  return desired.x - current.x;
}

// Used to get the distance between the current y coordinate and the desired y coordinate
double getYDistanceToGoal() {
  return desired.y - current.y;
}
 
// Sets the velocity of the turtle, stopping if it is at the goal
void setVelocity() {

  //If the x or y distances to the goal are greater than the distance tolerance, a velocity will be set
  if ((abs(getDistanceToGoal()) > distanceTolerance) || (abs(getYDistanceToGoal() > distanceTolerance))) {
 
    //The linear velocity of the turtle is established, slowing down as it gets distance gets smaller
    velCommand.linear.x = K_l * getDistanceToGoal();
    velCommand.linear.y = K_l * getYDistanceToGoal();
  }
  else {
    //Stops the turtle if it is at the goal
    velCommand.linear.x = 0;
    velCommand.linear.y = 0;
  }
}

// Used to check if the turtle is at the goal by checking if the absolute value of the distance to goal is less than the tolerance
bool atTheGoal(){
    return (abs(getDistanceToGoal()) < distanceTolerance) && (abs(getYDistanceToGoal() < distanceTolerance));
}
 
// This updates the x and y position of the turtle as well as its theta (not entirely sure what that is in this context
void updatePose(const turtlesim::PoseConstPtr &currentPose) {
  current.x = currentPose->x;
  current.y = currentPose->y;
  current.theta = currentPose->theta;
}
 
int main(int argc, char **argv) {

  //Initializes the linear and angular velocity
  setup(); 

  //Initializes the desired coordinates to be (9, 5.5444) 
  desired.x = 9;
  desired.y = 5.5444; //Note that the starting position of the turtle is at (5.5444, 5.5444)
 
  // Initiate ROS
  ros::init(argc, argv, "go_to_goal_x");
     
  // Creates a node handle
  ros::NodeHandle node;
 
  // Subscribe to the turtle's pose, allowing it to be updated
  ros::Subscriber currentPoseSub = node.subscribe("turtle1/pose", 0, updatePose);
 
  // Create a publisher that publishes velocity commands to the turtle.
  ros::Publisher velocityPub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);
  
  // Sets the loop rate to 10 cycles per second
  ros::Rate loop_rate(10); 
 
  // Keep running the while loop below as long as the ROS Master is active. 
  while (ros::ok()) {
 
    // Listens if ros is still ok
    ros::spinOnce();
 
    // Update the velocities of the turtle using previous function
    setVelocity();
 
    // Publish these velocities to velCommand
    velocityPub.publish(velCommand);
 
    // Not entirely sure why, but without setting the loop rate to sleep(),
    // the turtle does not go the correct directions
    loop_rate.sleep();

    // Break the while loop if the turtle is at the goal
    if (atTheGoal())
        break;
  }

  // Sets the new desired location to by (9,10)
  desired.x = 9;
  desired.y = 10;

  while (ros::ok()) {
 
    // Here is where we call the callbacks that need to be called.
    ros::spinOnce();
 
    // After we call the callback function to update the robot's pose, we 
    // set the velocity values for the robot.
    setVelocity();
 
    // Publish the velocity command to the ROS topic
    velocityPub.publish(velCommand);
 
    loop_rate.sleep(); 

    //Break the while loop if the turtle is at the goal
    if (atTheGoal())
        break;
  }

  return 0;
}
