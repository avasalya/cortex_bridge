/*
	Copyright 2015 Tyler Sorey, ARL, University of Nevada, Reno, USA

    This file is part of cortex_bridge.

    cortex_bridge is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    cortex_bridge is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/

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
#include <visualization_msgs/MarkerArray.h>

// service headers
#include "cortex_bridge/cortexSetOrigin.h"

// general headers
#include <tuple>
#include <sstream>
#include <time.h>


// Globals
tf::TransformBroadcaster *Cortex_broadcaster;

char** bodyOfInterest = NULL;
int NumBodiesOfInterest = 0;
float** bodyOriginOffset = NULL; // X, Y, Z, aX, aY, aZ W

ros::Publisher Cortex_markers;
ros::Time glob;



// call backs
bool setOriginCallback ( cortex_bridge::cortexSetOrigin::Request& req, cortex_bridge::cortexSetOrigin::Response& resp );


/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// NEW FRAME CALLBACK ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void newFrameCallback(sFrameOfData* FrameOfData)
{
	// publish markers
	cortex_bridge::Markers marker_array;
	marker_array = CreateMarkerArray_novis ( FrameOfData );
	Cortex_markers.publish(marker_array);

	// free data frame memory
	Cortex_FreeFrame ( FrameOfData );
}




// main driver
int main(int argc, char* argv[])
{

	// initialize ros / variables
	ros::init(argc, argv, "cortex_bridge");
	ros::NodeHandle nh;
	ros::ServiceServer service;
	glob = ros::Time::now();
	std::string myIP, cortexIP;

	// Get IPs from param file
	ros::NodeHandle param_nh("~");
	if ( !param_nh.getParam("local_ip", myIP) )
		ROS_ERROR ("Could not find IP of this machine in launch file.");
	if ( !param_nh.getParam("cortex_ip", cortexIP) )
		ROS_ERROR ("Could not find IP of cortex machine in launch file.");

	Cortex_markers = nh.advertise<cortex_bridge::Markers>("novis_markers", 1000);

	// broadcaster for transforms
	Cortex_broadcaster = new tf::TransformBroadcaster();

	// service to set origin for body
	service = nh.advertiseService("cortexSetOrigin", setOriginCallback);

	// set loop rate
	// ros::Rate loop_rate(60); //150
	
	// Initialize cortex connection
	if ( InitializeCortexConnection ( myIP.c_str(), cortexIP.c_str() ) != 0 )
	{
		ROS_INFO ( "Error: Unable to initialize ethernet communication" );
		return -1;
	}

	// get all the possible bodies to track
	NumBodiesOfInterest = GetKnownBodies ( bodyOfInterest );	
	ROS_INFO("Num Bodies Of Interest = %d", NumBodiesOfInterest);

	if ( NumBodiesOfInterest < 0 )
	{
		// clean up cortex connection
		Cortex_Exit();
		ROS_ERROR ("Are you sure you added body defs in Cortex?");
		return -1;
	}

	// init body offsets
	bodyOriginOffset = new float*[NumBodiesOfInterest];
	for ( int i = 0; i < NumBodiesOfInterest; ++i )
	{
		bodyOriginOffset[i] = new float[7];
		for ( int j = 0; j < 7; ++j )
			bodyOriginOffset[i][j] = 0.0;
	}

	
	sFrameOfData* getCurrentFrame;

	getCurrentFrame =  Cortex_GetCurrentFrame();  // Can POLL for the current frame.

	// ROS_INFO("number of markers defined in bodyTable: %d",getCurrentFrame->BodyData[0].nMarkers);
	// ROS_INFO("number of markers defined in bodyWand: %d",getCurrentFrame->BodyData[1].nMarkers);

	// get cortex frame rate information
	GetCortexFrameRate();

	// set callbacks and handlers
	InitializeCortexHandlers();

	void *pResponse;
	int nBytes;
	ROS_INFO("*** Starting live mode ***\n");
	Cortex_Request("LiveMode", &pResponse, &nBytes);

	int count = 0;
	ros::Rate rt(200);
	while(ros::ok())
	{
		auto getCurFrame = Cortex_GetCurrentFrame();
		newFrameCallback(getCurFrame);

		ros::spinOnce(); // or ros::spin() w/o above while(ros::ok())

		rt.sleep();
		++count;
	}


	// clean up cortex connection
	ROS_INFO ("Cortex_Exit");
	Cortex_Exit();

	// clean up memory
	for ( int i = 0; i < NumBodiesOfInterest; ++i )
	{
		delete bodyOfInterest[i];
		delete bodyOriginOffset[i];
		ROS_WARN("deleting body Interest 1");
	}
	delete bodyOfInterest;
	delete bodyOriginOffset;
	ROS_WARN("deleting body Interest 2");
	return 0;
}




/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// ORIGIN SRV CALLBACK ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
bool setOriginCallback ( cortex_bridge::cortexSetOrigin::Request& req, cortex_bridge::cortexSetOrigin::Response& resp )
{
	ROS_INFO ( "Request to set origin for body: %s", req.subject_name.c_str() );
	// initialize variables
	tf::TransformListener tf_listener;
	tf::StampedTransform transform;
	tf::Quaternion orientation ( 0, 0, 0, 0 );
	tf::Vector3 position ( 0, 0, 0 );
	std::string bodyToSet;

	// get body index
	int bodyIndex = FindBodyIndex ( req.subject_name.c_str(), NumBodiesOfInterest, bodyOfInterest );

	// build name of tracked segment
	if ( req.segment_name.empty() )
		bodyToSet = "/" + req.subject_name;
	else
		bodyToSet = "/" + req.subject_name + "/" + req.segment_name;

	// gather data of body so we can get the average "current" position and set it as origin
	int numSuccessfulMeasurements = 0;
	ros::Duration timeout ( 0.1 );
	ros::Duration poll ( 0.1 / 240.0 );

	for ( int i = 0; i < req.n_measurements; ++i )
	{
		if ( tf_listener.waitForTransform("/world", bodyToSet, ros::Time::now(), timeout, poll) )
		{
			tf_listener.lookupTransform ( "/world", bodyToSet, ros::Time(0), transform );
			orientation += transform.getRotation();
			position += transform.getOrigin();
			numSuccessfulMeasurements ++;
		}
	}

	// average data to get a "precise" origin
	orientation /= numSuccessfulMeasurements;
	orientation.normalize();
	position /= numSuccessfulMeasurements;

	// create response
	// don't call it "successful" if no measurements were gathered
	if ( numSuccessfulMeasurements == 0 )
		resp.success = false;

	else
	{
		resp.success = true;
		resp.pose.header.stamp = ros::Time::now();
		resp.pose.header.frame_id = "/world";
		resp.pose.pose.position.x = position.x();
		resp.pose.pose.position.y = position.y();
		resp.pose.pose.position.z = position.z();
		resp.pose.pose.orientation.x = orientation.x();
		resp.pose.pose.orientation.y = orientation.y();
		resp.pose.pose.orientation.z = orientation.z();
		resp.pose.pose.orientation.w = orientation.w();

		// set offsets for the body for future frame publishing
		bodyOriginOffset[bodyIndex][0] += position.x();
		bodyOriginOffset[bodyIndex][1] += position.y();
		bodyOriginOffset[bodyIndex][2] += position.z() - req.z_offset;

		//tf::Matrix3x3 mat(orientation);
		double roll, pitch, yaw;
		transform.getBasis().getRPY ( roll, pitch, yaw );
		bodyOriginOffset[bodyIndex][3] += roll;
		bodyOriginOffset[bodyIndex][4] += pitch;
		bodyOriginOffset[bodyIndex][5] += yaw;
	}
	ROS_INFO ( "Response to set origin call sent." );
	return resp.success;
}