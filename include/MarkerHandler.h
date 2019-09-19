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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "cortex_bridge/Markers.h"
#include "cortex_bridge/Marker.h"

visualization_msgs::MarkerArray CreateMarkerArray_vis ( sFrameOfData* FrameOfData );
cortex_bridge::Markers CreateMarkerArray_novis ( sFrameOfData* FrameOfData );
