#include <iostream> //Used for cin and cout
#include <ros/ros.h> //Used for the ros specific aspects like time and ROS_Info
#include <actionlib/client/simple_action_client.h> //Used to create a client to receive coordinates
#include <actionlib_msgs/GoalStatus.h> //Used to read the status of whether the robot has reached the goal
#include <move_base_msgs/MoveBaseAction.h> //Used to create a client to receive coordinates
#include <move_base_msgs/MoveBaseGoal.h> //Used to send the goal to client
#include <string> //Used to create strings

//Creates a template to create a client with.
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Function used to create a client and send provided coordinates to
void sendInputGoal(float x, float y){

    //Creates a client connecting to the "move_base" topic in ROS/Gazebo
    MoveBaseClient actionClient1("move_base", true);

    //Creates a MoveBaseGoal used to send the coordinates to the client
    move_base_msgs::MoveBaseGoal goal;

    //Prints a ROS message informing the user that they're waiting for the client to connect to the server
    ROS_INFO("Waiting for the action server to start");

    //Loops printing a message while waiting on the server to connect to the client during a set time
    while(!actionClient1.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //Prints a message informing that the server has connected and that the provided goal is now being sent
    ROS_INFO("Action server started, sending the goal");

    //Sets the frame_id to be "map," necessary to connect to Gazebo
    goal.target_pose.header.frame_id = "map";

    //Sets the stamp to reflect the time in which the function was called
    goal.target_pose.header.stamp = ros::Time::now();

    //Prints out the stamp to the user
    std::cout << "Stamp: " << goal.target_pose.header.stamp << "\n";

    //Sets the x and y positions for the provided MoveBaseGoal object to be the provided x and y floats, setting z to 0
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0;

    //Prints the assigned x, y, and z positions
    std::cout << "Target pose x: " << goal.target_pose.pose.position.x << "\n";
    std::cout << "Target pose y: " << goal.target_pose.pose.position.y << "\n";
    std::cout << "Target pose z: " << goal.target_pose.pose.position.z << "\n";

    //Sets the x, y, z, and w orientations
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = -1.0;
    goal.target_pose.pose.orientation.w = 1.0;

    //Prints the assigned orientation
    std::cout << "Target pose x orientation: " << goal.target_pose.pose.orientation.x << "\n";
    std::cout << "Target pose y orientation: " << goal.target_pose.pose.orientation.y << "\n";
    std::cout << "Target pose z orientation: " << goal.target_pose.pose.orientation.z << "\n";

    //Sends the goal to the client
    actionClient1.sendGoal(goal);

    //Creates a variable that checks the state of the SimpleClientGoalState
    actionlib::SimpleClientGoalState stateResult = actionClient1.getState();

    //This one's goal is to print the state result, but haven't figured out how to convert this to string yet.  May end up abandoning.
    ROS_INFO("state_result: " );

    //Creates a variable that is the time at this point
    ros::Time startTime = ros::Time::now();

    //Creates a variable that will reflect the elapsed time since the start
    ros::Duration elapsedTime;

    //Runs a while loop while the goal is being moved towards, establishing the elapsed time between the current time and the start time
    while (stateResult == actionlib::SimpleClientGoalState::ACTIVE){
        stateResult = actionClient1.getState();
        elapsedTime = ros::Time::now() - startTime;
    }

    //Also meant to print the state result.  May also end up abandoning
    ROS_INFO("[Result] State: ");

    //Prints elapsed time once completed
    if (stateResult == actionlib::SimpleClientGoalState::SUCCEEDED){
        std::cout << "Elapsed time: " << elapsedTime << "\n";
    }

    //Prints a message if an error results in the process being aborted
    if (stateResult == actionlib::SimpleClientGoalState::ABORTED){
        std::cout << "Something went wrong on server side.\nElapsed time: " << elapsedTime << "\n";
    }

}

int main(int argc, char *argv[]){

    //Initializes a ROS node
    ros::init(argc, argv, "send_goal");

    //Creates a node handle
    ros::NodeHandle node;

    //Creates floats that will store the provided x and y
    float in1, in2;

    //Asks the user for provided x and y coordinates, storing them in established floats
    std::cout << "What x coordinate would you like to go to? (1 - 10) ";
    std::cin >> in1;
    std::cout << "\n";

    std::cout << "What y coordinate would you like to go to? (1 - 10) ";
    std::cin >> in2;
    std::cout << "\n";

    //Calls the function using the provided coordinates
    sendInputGoal(in1, in2);


    return 0;
}