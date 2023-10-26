
#include <cstdlib> // Use for the absolute value method abs()
#include <iostream> // Use for in out stream
#include "ros/ros.h" // Used to connect to ros
#include "geometry_msgs/Twist.h" // Used to establish linear and angular velocity like before
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation
#include "turtlesim/Pose.h" // Used to find current pose
#include <turtlesim/SetPen.h>
#include <std_msgs/String.h>
#include <turtlesim/TeleportAbsolute.h>
//#include "turtlesim/SetPen.h
//#include "turtlesim/TeleportAbsolute.h"

using namespace std;//Set namespace to std

geometry_msgs::Twist velCommand; // Linear and angular velocity in m/s 
geometry_msgs::Pose2D current; // Used to find the current x and y position of the turtle
geometry_msgs::Pose2D desired; // Used to establish the desired x and y position of the turtle


const double distanceTolerance = 0.01; // Used to say how close in meters is okay to say the turtle is at the goal
 
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

void setAngle(double a){
    velCommand.angular.z = a;
}

// Sets the velocity of the turtle, stopping if it is at the goal
void setVelocity() {

  //If the x or y distances to the goal are greater than the distance tolerance, a velocity will be set
  if ((abs(getDistanceToGoal()) > distanceTolerance) || (abs(getYDistanceToGoal() > distanceTolerance))) {
 
    //The linear velocity of the turtle is established, slowing down as it gets distance gets smaller
    velCommand.linear.x = getDistanceToGoal();
    velCommand.linear.y = getYDistanceToGoal();
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

 // turtlesim:TeleportAbsolute telport_pose;//Used to teleport the t
 
   // ros::ServiceClient client_pose;

   ros::ServiceClient pen = node.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");

    turtlesim::SetPen pen_state;


  //client_pose = node.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
 
  // Subscribe to the turtle's pose, allowing it to be updated
  ros::Subscriber currentPoseSub = node.subscribe("turtle1/pose", 0, updatePose);
 
  // Create a publisher that publishes velocity commands to the turtle.
  ros::Publisher velocityPub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);
  
  // Sets the loop rate to 10 cycles per second

    float coordinates [50][2] = {{6.5,5.55},{6.5,6.5},{5.5,6.5},{5.45,5.45},//square one
                                 {5.5,3.5},{6.5,4},{6.49,5},{5.5,5},{5.49,3},//teleport to square two (5.5,5)
                                 {4.5,7},{6.75,7},{7.25,6.5},//Beginning of b
                                 {7.27,5.75},{6.75,5.25},{7.25,4.75},{7.27,4}, {6.75,3.5},//
                                 {4.5,3.5},{5,4},{4.9,6.5},{4.5,7},//
                                 {4,7.5},{5,7.75},{7,7.75},{7.75,7.5},{7.78,2.5},{7.25,3},{6.75,2.75},//
                                 {6,2.5},{5,2.75},{4.5,3},{4,3.5},{4,7.5}};//teleport to shield
 

  ros::Rate loop_rate(10); 
/*
    teleport_pose.request.x = 1.0;
    teleport_pose.request.y = 1.0;
    teleport_pose.request.theta = 0.0;*/

  int i;

pen_state.request.off = 0;
pen_state.request.r = 255;
pen_state.request.g = 255;
pen_state.request.b = 255;

pen.call(pen_state);


  for (i = 0; i < 4; i++){
    cout << coordinates [i][0] << "\n";
    cout << coordinates [i][1]<< "\n\n";

    desired.x = coordinates [i][0];
    desired.y = coordinates [i][1];
 
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
  }

  pen_state.request.off = 1;
  pen.call(pen_state);

  i=4;

cout << coordinates [i][0] << "\n";
cout << coordinates [i][1]<< "\n\n";

desired.x = coordinates [i][0];
desired.y = coordinates [i][1];

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

pen_state.request.off = 0;
pen_state.request.r = 255;
pen_state.request.g = 255;
pen_state.request.b = 255;

pen.call(pen_state);
 
for (i = 5; i < 9; i++){
    cout << coordinates [i][0] << "\n";
    cout << coordinates [i][1]<< "\n\n";

    desired.x = coordinates [i][0];
    desired.y = coordinates [i][1];
 
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
  }

pen_state.request.off = 1;
pen.call(pen_state);

i=9;

cout << coordinates [i][0] << "\n";
cout << coordinates [i][1]<< "\n\n";

desired.x = coordinates [i][0];
desired.y = coordinates [i][1];

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

pen_state.request.off = 0;
pen_state.request.r = 255;
pen_state.request.g = 255;
pen_state.request.b = 255;

pen.call(pen_state);
 
for (i = 10; i < 21; i++){
    cout << coordinates [i][0] << "\n";
    cout << coordinates [i][1]<< "\n\n";

    desired.x = coordinates [i][0];
    desired.y = coordinates [i][1];
 
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
  }

pen_state.request.off = 1;
pen.call(pen_state);

i=21;

cout << coordinates [i][0] << "\n";
cout << coordinates [i][1]<< "\n\n";

desired.x = coordinates [i][0];
desired.y = coordinates [i][1];

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

pen_state.request.off = 0;
pen_state.request.r = 255;
pen_state.request.g = 255;
pen_state.request.b = 255;

pen.call(pen_state);

    for (i = 22; i < 33; i++){
    cout << coordinates [i][0] << "\n";
    cout << coordinates [i][1]<< "\n\n";

    desired.x = coordinates [i][0];
    desired.y = coordinates [i][1];

 
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
 
  }



  return 0;
}
