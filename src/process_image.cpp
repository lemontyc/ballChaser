#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request command_robot service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x    = lin_x;
    srv.request.angular_z   = ang_z;
    
    // Call the command_robot service and pass the requested velocities
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int img_zone_size = img.width/3;     // Divide the image in 3 sections
    bool ball_detected = false;         // Used to stop the robot
    
    

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for (int i = 0; i < img.height * img.width; i++) 
    {
        if (img.data[i * 3] == white_pixel && img.data[(i * 3) + 1] == white_pixel && img.data[(i * 3) + 2] == white_pixel) 
        {
            // Calculate pixel row position
            int j = i % img.width;
            if(j > img_zone_size * 2)       // Ball detected to the right
            {
                drive_robot(0.0, -0.2);
            }else if (j > img_zone_size)    // Ball detected to mid
            {
                drive_robot(0.2, 0.0);
            }else                           // Ball detected to the left
            {
                drive_robot(0.0, 0.2);
            }
            ball_detected = true;
            break;
        }
    }

    // No image detected, request full stop
    if(!ball_detected)
    {
        drive_robot(0.0, 0.0);
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