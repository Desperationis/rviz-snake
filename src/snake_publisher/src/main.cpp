#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "Markers.hpp"

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
