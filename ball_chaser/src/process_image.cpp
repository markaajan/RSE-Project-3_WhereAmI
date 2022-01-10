#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
enum prevstate{NONE, LEFT, MIDDLE, RIGHT};
prevstate previous = NONE;
int white_pixel = 255;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");
}

// This callback function continuously executes and reads the image data

bool check_left(const sensor_msgs::Image img)
{
    int limit = (img.width);
    for (unsigned int j = 0; j < img.height; j++)
    {
        for (unsigned int i = 0; i < limit; i+=3)
        {
            if (img.data[j*img.step + i] == white_pixel && img.data[j*img.step + i + 1] == white_pixel && img.data[j*img.step + i + 2] == white_pixel)
            {
                previous = LEFT;
                return true;
            }
        }
    }
    return false;
}

bool check_middle(const sensor_msgs::Image img)
{
    int left_limit = (img.width);
    int right_limit = (img.width)*2;
    for (unsigned int j = 0; j < img.height; j++)
    {
        for (unsigned int i = left_limit; i < right_limit; i+=3)
        {
            if (img.data[j*img.step + i] == white_pixel && img.data[j*img.step + i + 1] == white_pixel && img.data[j*img.step + i + 2] == white_pixel)
            {
                previous = MIDDLE;
                return true;
            }
        }
    }
    return false;
}

bool check_right(const sensor_msgs::Image img)
{
    int left_limit = (img.width)*2;
    int right_limit = (img.width)*3;
    for (unsigned int j = 0; j < img.height; j++)
    {
        for (unsigned int i = left_limit; i < right_limit; i+=3)
        {
            if (img.data[j*img.step + i] == white_pixel && img.data[j*img.step + i + 1] == white_pixel && img.data[j*img.step + i + 2] == white_pixel)
            {
                previous = RIGHT;
                return true;
            }
        }
    }
    return false;
}


void process_image_callback(const sensor_msgs::Image img)
{


    if (previous == LEFT|| previous == NONE)
    {
        if(check_left(img))
            drive_robot(0.0,0.5);
        else if(check_middle(img))
            drive_robot(0.8,0.0);
        else if(check_right(img))
            drive_robot(0.0,-0.5);
        else
        {
            previous = NONE;
            drive_robot(0.0,0.0);
        }
    }
    else if (previous == MIDDLE)
    {
        if(check_middle(img))
            drive_robot(0.8,0.0);
        else if(check_left(img))
            drive_robot(0.0,0.5);
        else if(check_right(img))
            drive_robot(0.0,-0.5);
        else
        {
            previous = NONE;
            drive_robot(0.0,0.0);
        }
    }
    else if (previous == RIGHT)
    {
        if(check_right(img))
            drive_robot(0.0,-0.5);
        else if(check_middle(img))
            drive_robot(0.8,0.0);
        else if(check_left(img))
            drive_robot(0.0,0.5);
        else
        {
            previous = NONE;
            drive_robot(0.0,0.0);
        }
    }         
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
