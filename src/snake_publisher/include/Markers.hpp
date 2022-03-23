#ifndef MARKERS_HPP
#define MARKERS_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"

/**
 * Markers.hpp
 *
 * This file holds classes that relate to encapsulating ROS2 and rviz
 * functionality into easy to use classes meant for grid rendering.
*/ 


/**
 * Struct meant to easily allow you to modify a single cube in a CubeArray.
*/ 
struct Cube {
	Cube() { 
		SetPos(0,0,0);
		SetColor(1,1,1,1);
	}

	void SetPos(int x, int y, int z) {
		point.x = x;
		point.y = y;
		point.z = z;
	}

	/**
	 * Sets the color of the cube. Each parameter must be from 0.0 - 1.0.
	*/ 
	void SetColor(float r, float g, float b, float a) {
		color.r = r;
		color.g = g;
		color.b = b;
		color.a = a;
	}

	geometry_msgs::msg::Point point;
	std_msgs::msg::ColorRGBA color;
};

/**
 * Wrapper for a CubeArray that allows you to add cubes via the Cube struct.
*/ 
class GridMarker : public visualization_msgs::msg::Marker {
public:
	GridMarker() {
		type = visualization_msgs::msg::Marker::CUBE_LIST;
		pose.position.x = 0;
		pose.position.y = 0;
		pose.position.z = 0;
		pose.orientation.x = 0;
		pose.orientation.y = 0;
		pose.orientation.z = 0;
		pose.orientation.w = 0;
		scale.x = 1;
		scale.y = 1;
		scale.z = 1;
		color.a = 1;
		color.r = 1;
		color.g = 1;
		color.b = 1;
	}

	void AddCube(Cube cube) {
		points.push_back(cube.point);
		colors.push_back(cube.color);
	}
};

/**
 * Node solely meant to publish markers to rviz.
*/ 
class MarkerPublisher : public rclcpp::Node {
public:
	MarkerPublisher() : Node("marker_publisher") {
			publisher = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 0);
	}

	/**
	 * Publish a marker with proper headers.
	*/ 
	void PublishMarker(visualization_msgs::msg::Marker marker) {
		marker.ns = "gameboard";
		marker.id = 0;
		marker.header.frame_id = "map";
		marker.header.stamp = this->now();
		marker.action = visualization_msgs::msg::Marker::ADD;
		publisher->publish(marker);
	}

private:
	std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> publisher;
};

#endif
