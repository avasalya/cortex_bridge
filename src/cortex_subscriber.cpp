

// ros / cortex headers
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "cortex.h"

// marker headers
#include "MarkerHandler.h"
#include "TransformHandler.h"
#include "CortexHandler.h"

#include "cortex_bridge/Markers.h"

// service headers
#include "cortex_bridge/cortexSetOrigin.h"

// general headers
#include <tuple>
#include <time.h>
#include <sstream>
#include "std_msgs/String.h"



 void cortex_subscriber_callback(cortex_bridge::Markers marker_array)
 {
    ROS_INFO("I heard");
 }
  
// void newFrameCallback ( sFrameOfData* FrameOfData );


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cortex_subscriber");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("novis_markers", 1000, cortex_subscriber_callback);

    ros::spin();

    return 0;
}