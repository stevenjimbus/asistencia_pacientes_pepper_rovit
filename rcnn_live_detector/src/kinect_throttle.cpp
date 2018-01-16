/********************************************************************************

Software License Agreement (BSD 3-Clause)

Copyright (c) 2016, Antonio Coratelli
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rcnn_live_detector/paquete_imagenes.h"
#include <sstream>






// General Defines
#define NAME "rgbd_and_depth_throttle"

// Default Param Defines
#define RATE 4

// Input Defines

#define RGB_RECT_IN   "/camera/rgb/image_rect_color"
#define DEPTH_RECT_IN "/camera/depth/image_rect"
#define BUFFER_IN 1

// Output Defines
#define RGB_RECT_OUT   "/rgb/rect_out"
#define DEPTH_RECT_OUT "/depth/rect_out"
#define BUFFER_OUT 1

// Global Variables

//ros::Publisher pub_rgb_rect;
//ros::Publisher pub_depth_rect;
ros::Publisher pub_rgb_and_depth;

double rate;
double secs;
double last_sent;


sensor_msgs::Image      rgb_rect;
sensor_msgs::Image      depth_rect;
rcnn_live_detector::paquete_imagenes paquete_imagenes_rgb_depth;

bool var1 = false;

// Callbacks
void callback_rgb_rect(const sensor_msgs::Image& data)
{
    rgb_rect = data;
    var1 = true;

}
void callback_depth_rect(const sensor_msgs::Image& data)
{
    depth_rect = data;

    if (var1)
    {
        if (data.header.stamp.toSec() >= last_sent + secs)
            {   
                paquete_imagenes_rgb_depth.imagenRGB=rgb_rect;
                paquete_imagenes_rgb_depth.imagenDEPTH=depth_rect;
                pub_rgb_and_depth.publish(paquete_imagenes_rgb_depth);
                last_sent = data.header.stamp.toSec();
            }
    }
    var1 = false;
}

// Main

int main(int argc, char **argv)
{
    ros::init(argc, argv, NAME);
    ros::NodeHandle n;

    ROS_INFO("Initializing node '%s' ...", NAME);

    // Read Params
    n.param<double>("rate", rate, RATE);
    secs = 1.0 / rate;
    ROS_INFO("Time between frames: %f seconds.", secs);

    // Initialize Variables
    last_sent = 0;

   
    ros::Subscriber sub_rgb_rect   = n.subscribe(RGB_RECT_IN,   BUFFER_IN, callback_rgb_rect);
    ros::Subscriber sub_depth_rect = n.subscribe(DEPTH_RECT_IN, BUFFER_IN, callback_depth_rect);

    // Advertise Publishers
    //pub_rgb_rect   = n.advertise<sensor_msgs::Image     >(RGB_RECT_OUT,   BUFFER_OUT);    
    pub_rgb_and_depth = n.advertise<rcnn_live_detector::paquete_imagenes>("conjunto_imagenes_rgb_depth", BUFFER_OUT);



    ros::spin();
    return 0;
}