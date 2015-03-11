/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2015 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Repository name: hmi_2015
 * \note
 * ROS package name: hmi_table_localisation
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: 11.03.2015
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include "hmi_table_localisation/table_localisation.h"
#include <vector>

TableLocalization::TableLocalization(ros::NodeHandle& nh)
		: node_handle_(nh)
{
	laser_scan_sub_ = node_handle_.subscribe("laser_scan_in", 0, &TableLocalization::callback, this);
}

TableLocalization::~TableLocalization()
{

}

#define DEBUG_OUTPUT
void TableLocalization::callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg)
{
	// detect reflector markers (by intensity thresholding)
	std::vector<std::vector<Point2d> > segments;
	std::vector<Point2d> segment;
	bool in_reflector_segment = false;
	for (unsigned int i = 0; i < laser_scan_msg->ranges.size(); ++i)
	{
		double angle = laser_scan_msg->angle_min + i * laser_scan_msg->angle_increment; //[rad]
		double dist = laser_scan_msg->ranges[i];
		double y = dist * sin(angle);
		double x = dist * cos(angle);
		if (laser_scan_msg->intensities[i] < 4048.0 && in_reflector_segment==true)
		{
			// finish reflector segment
			segments.push_back(segment);
			segment.clear();
			in_reflector_segment = false;
		}
		if (laser_scan_msg->intensities[i] >= 4048.0)
		{
			segment.push_back(Point2d(x, y));
			in_reflector_segment = true;
		}
	}

#ifdef DEBUG_OUTPUT
	std::cout << "Segments with high intensity:\n";
	for (size_t i=0; i<segments.size(); ++i)
	{
		for (size_t j=0; j<segments[i].size(); ++j)
			std::cout << segments[i][j] << "\t";
		std::cout << std::endl;
	}
#endif

	// compute center coordinates of reflectors
	std::vector<Point2d> reflectors(segments.size());
	for (size_t i=0; i<segments.size(); ++i)
	{
		for (size_t j=0; j<segments[i].size(); ++j)
			reflectors[i] += segments[i][j];
		reflectors[i] /= (double)segments[i].size();
	}

#ifdef DEBUG_OUTPUT
	std::cout << "Reflector centers:\n";
	for (size_t i=0; i<reflectors.size(); ++i)
	{
		std::cout << reflectors[i] << std::endl;
	}
#endif

	// determine coordinate system generated by reflectors (laser scanner coordinate system: x=forward, y=left, z=up)
	bool publish_tf = false;
	tf::StampedTransform transform_table_reference;
	if (reflectors.size()==2)
	{
		publish_tf = true;
		// offset in laser scanner coordinate system
		tf::Vector3 origin((reflectors[0].x+reflectors[1].x)*0.5, (reflectors[0].y+reflectors[1].y)*0.5, 0.);
		// direction of x-axis in laser scanner coordinate system
		Point2d normal(reflectors[1].y-reflectors[0].y, reflectors[0].x-reflectors[1].x);
		if (normal.x*origin.getX() + normal.y*origin.getY() < 0)
			normal *= -1.;
		double angle = atan2(normal.y, normal.x);
		// transform
		transform_table_reference.setOrigin(origin);
		transform_table_reference.setRotation(tf::Quaternion(tf::Vector3(0,0,1), angle));
	}
	else if (reflectors.size() > 2)
	{
		ROS_WARN("Regression with >2 reflectors not implemented");
	}

	// publish coordinate system on tf
	if (publish_tf == true)
	{
		transform_broadcaster_.sendTransform(tf::StampedTransform(transform_table_reference.inverse(), laser_scan_msg->header.stamp, "/table_reference", laser_scan_msg->header.frame_id));
	}
}
