 #include<iostream> //Used to allow cin and cout
    #include "ros/ros.h" //Used to interact with ros aspects
    #include<geometry_msgs/Twist.h> //Used to allow specific commands to be sent to the turtle
     
    using namespace std; //uses std namespace for all aspects not already declared

    int main(int argc, char *argv[])
    {

        ros::init(argc, argv, "turtle_time");  //Initializes a ros node
        ros::NodeHandle n; //Declares an object that allows the program to interact with turtlesim

        //Declares a publisher used to send an object to the turtle program
        ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

        ROS_INFO("The circling begins..."); //Publishes a silly message to ros


        //Declare a variable to accept a user input
        int radius = 0;

        //Ask the user for how big they'd like the circle to be
        cout << "\nHow big would you like the circle to be?\n\n";
        cin >> radius;	
        cout << "\n";

        while(ros::ok()) //Loops the program while ROS has not shut down the program
        {
            geometry_msgs::Twist turtle; //Declare a Twist object named turtle
     
            turtle.linear.x = radius; //Sets how far forward the turtle is going to what the user provided, the radius
            turtle.linear.y = 0.0;  //Sets how far in the y direction the turtle's going, 0
            turtle.linear.z = 0.0; //Sets how far in the z direction the turtle's going, 0
     
            turtle.angular.x = 0; //Sets the angular velocity of the turtle in the x direction, 0
            turtle.angular.y = 0; //Sets the angular velocity of the turtle in the y diection, 0
            turtle.angular.z = 2; //Sets turtle's angular velocity to 2 so it will draw a circle

            vel_pub.publish(turtle); //Sends the declared turtle object to the publisher declared earlier
     
            //Has the program listen for any callback from turtlesim, namely if it's shut down
            ros::spinOnce();
        }
        return 0; //Returns 0 to end the program
    }

