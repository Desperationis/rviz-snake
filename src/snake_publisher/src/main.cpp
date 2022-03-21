#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"

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

	void SetColor(float r, float g, float b, float a) {
		color.r = r;
		color.g = g;
		color.b = b;
		color.a = a;
	}

	geometry_msgs::msg::Point point;
	std_msgs::msg::ColorRGBA color;
};

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

class MarkerPublisher : public rclcpp::Node {
public:
	MarkerPublisher() : Node("marker_publisher") {
			publisher = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 0);
	}

	/**
	 * Publish a marker with proper headers.
	*/ 
	void PublishMarker(visualization_msgs::msg::Marker marker) {
		publisher->publish(markerArray);
		marker.ns = "gameboard";
		marker.id = 0;
		marker.header.frame_id = "map";
		marker.header.stamp = this->now();
		marker.action = visualization_msgs::msg::Marker::ADD;
		publisher->publish(marker);
	}

private:
	std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> publisher;
	visualization_msgs::msg::Marker markerArray;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MarkerPublisher>();
	GridMarker grid;

	for(int i = -8; i <= 4; i++) {
		for(int j = -4; j <= 8; j++) {
			Cube cube;
			cube.SetPos(i, j, 0);
			float color = 1;
			cube.SetColor(color,color,color,1);

			grid.AddCube(cube);
		}
	}

	node->PublishMarker(grid);

	rclcpp::spin(node);
	rclcpp::shutdown();
}
