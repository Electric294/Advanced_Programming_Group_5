
#include <cstdlib> // Use for the absolute value method abs()
#include <iostream> // Use for in out stream
#include "ros/ros.h" // Used to connect to ros
#include "geometry_msgs/Twist.h" // Used to establish linear and angular velocity like before
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation
#include "turtlesim/Pose.h" // Used to find current pose
#include <turtlesim/SetPen.h>//Used to lift the pen off the canvas and set color

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

void goToGoals(){}
 
// Used to get the distance between the current x coordinate and the desired x coordinate
double getDistanceToGoal() {
  return desired.x - current.x;
}

// Used to get the distance between the current y coordinate and the desired y coordinate
double getYDistanceToGoal() {
  return desired.y - current.y;
}

/*
void setAngle(double a){
    velCommand.angular.z = a;
}*/

// Sets the velocity of the turtle, stopping if it is at the goal
void setVelocity() {

  //If the x or y distances to the goal are greater than the distance tolerance, a velocity will be set
  if ((abs(getDistanceToGoal()) > distanceTolerance) || (abs(getYDistanceToGoal() > distanceTolerance))) {
 
    //The linear velocity of the turtle is established, slowing down as it gets distance gets smaller
    velCommand.linear.x = 2 * getDistanceToGoal();
    velCommand.linear.y = 2 * getYDistanceToGoal();
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

  // Initializes the linear and angular velocity using previously declared function
  setup(); 

  // Initiate ROS
  ros::init(argc, argv, "go_to_goal_x");
     
  // Creates a node handle
  ros::NodeHandle node;

  // Creates a service server that tracks the pen state (on or off and color)
  turtlesim::SetPen pen_state;

  // Creates a service client to call the pen state server and reflect it on the turtle
  ros::ServiceClient pen = node.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");

 
  // Subscribe to the turtle's pose, allowing it to be updated
  ros::Subscriber currentPoseSub = node.subscribe("turtle1/pose", 0, updatePose);
 
  // Create a publisher that publishes velocity commands to the turtle.
  ros::Publisher velocityPub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);
  

  // Creates a multidimensional array that notes the desired coordinates of the Bradley shape
  float coordinates [33][2] = {{6.5,5.55},{6.5,6.5},{5.5,6.5},{5.45,5.45},//square one
                                 {5.5,3.55},{6.5,4},{6.49,5},{5.5,5},{5.49,2.95},//square two
                                 {4.5,7},{6.75,7},{7.25,6.5},//B shape beginning
                                 {7.27,5.75},{6.75,5.25},{7.25,4.75},{7.27,4}, {6.75,3.5},//B shape continued
                                 {4.5,3.5},{5,4},{4.9,6.5},{4.5,7},//B shape continued
                                 {4,7.5},{5,7.75},{7,7.75},{7.75,7.5},{7.78,2.5},{7.25,3},{6.75,2.75},//B shape final
                                 {6,2.5},{5,2.75},{4.5,3},{4,3.5},{4,7.5}};//shield shape
 
  // Sets the loop rate to 10 cycles per second
  ros::Rate loop_rate(10); 

  //Set a counter variable to iterate through the multidimensional array
  int i;

  // Turns the pen state on and sets the color to white using max RGB values
  pen_state.request.off = 0;
  pen_state.request.r = 255;
  pen_state.request.g = 255;
  pen_state.request.b = 255;

  // Has the client call the pen state and reflect it on the turtle
  pen.call(pen_state);

  // Sets up a for loop iterating through the first 4 coordinates that represent the first square
  for (i = 0; i < 4; i++){
    /*
    // Prints the coordinates to the terminal
    cout << coordinates [i][0] << "\n";
    cout << coordinates [i][1]<< "\n\n";*/

    // Sets the desired coordinates (goal) to be the coordinates found in the array that i is iterating through 
    desired.x = coordinates [i][0];
    desired.y = coordinates [i][1];
 
    // This while loop moves the turtle to the goal while the ROS master is active
    while (ros::ok()) {
    
        // Listens if ros is still active
        ros::spinOnce();
    
        // Sets the linear velocity of the turtle using previous function that stops if at the goal
        setVelocity();
    
        // Use the velocityPub publisher to publish the state of velCommand (the turtle) to the topic of turtle1/cmd_vel
        velocityPub.publish(velCommand);
    
        // Not entirely sure why, but without setting the loop rate to sleep(),
        // the turtle does not go the correct directions
        loop_rate.sleep();

        // Break the while loop if the turtle is at the goal
        if (atTheGoal())
            break;
    }
  }


// Turns the pen state to off
pen_state.request.off = 1;

// Publishes this pen state to pen
pen.call(pen_state);

// Set the iterator to be the coordinate the turtle will move to with no drawing
i=4;

// Print the coordinates to the terminal
/*
cout << coordinates [i][0] << "\n";
cout << coordinates [i][1]<< "\n\n";*/

// Set the coordinates be the 4th set of coordinates
desired.x = coordinates [i][0];
desired.y = coordinates [i][1];

// While loop to move the turtle to this goal, where the next shape starts
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

// Sets the pen_state to be on
pen_state.request.off = 0;

// Publishes the pen_state to pen
pen.call(pen_state);
 
// For loop to iterate through coordinates 5 - 8, which represents the second square
for (i = 5; i < 9; i++){

    /*
    // Print the coordinates to the terminal
    cout << coordinates [i][0] << "\n";
    cout << coordinates [i][1]<< "\n\n";*/

    // Sets the desired coordinates (goal) to be the coordinates found in the array that i is iterating through 
    desired.x = coordinates [i][0];
    desired.y = coordinates [i][1];
 
    // While loop to move the turtle to each set of coordinates
    while (ros::ok()) {
    
        // Listens if ros is still running
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

// Turns the pen state to off
pen_state.request.off = 1;

// Publishes this pen state to pen
pen.call(pen_state);

// Sets i to be the set of coordinates to move to without drawing
i=9;

// Prints the coordinates to the terminal
/*
cout << coordinates [i][0] << "\n";
cout << coordinates [i][1]<< "\n\n";*/

// Set the coordinates be the 9th set of coordinates
desired.x = coordinates [i][0];
desired.y = coordinates [i][1];

// While loop same as before to move the turtle until it is where the next shape will start
while (ros::ok()) {

    // Listens if ros is still running
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

// Set the pen_state to be back on
pen_state.request.off = 0;

// Publishes this pen state to pen
pen.call(pen_state);
 
for (i = 10; i < 21; i++){

    /*
    // Prints the coordinates to the terminal
    cout << coordinates [i][0] << "\n";
    cout << coordinates [i][1]<< "\n\n";*/

     // Sets the desired coordinates (goal) to be the coordinates found in the array that i is iterating through 
    desired.x = coordinates [i][0];
    desired.y = coordinates [i][1];
 
    // While loop to move the turtle to each set of coordinates in the shape
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

// Sets the pen state to be off
pen_state.request.off = 1;

// Publishes this pen state to pen
pen.call(pen_state);

// Sets i to 21, the next set of coordinates to move to without drawing
i=21;

/*
// Print this set of coordinates to the screen
cout << coordinates [i][0] << "\n";
cout << coordinates [i][1]<< "\n\n";*/

// Set the goal to be these coordinates
desired.x = coordinates [i][0];
desired.y = coordinates [i][1];

// While loop to move the turtle to this set of coordinates
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

// Sets the pen state to back on
pen_state.request.off = 0;

// Publish this pen state to pen
pen.call(pen_state);

// For loop to iterate through coordinates 22 - 33, the shape of the shield
for (i = 22; i < 33; i++){

    /*
    // Print coordinates to the terminal
    cout << coordinates [i][0] << "\n";
    cout << coordinates [i][1]<< "\n\n";*/

     // Sets the desired coordinates (goal) to be the coordinates found in the array that i is iterating through 
    desired.x = coordinates [i][0];
    desired.y = coordinates [i][1];

 
    // While loop to move the turtle to each set of coordinates
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
