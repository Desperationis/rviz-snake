#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class ShapePublisher : public rclcpp::Node {
public:
	ShapePublisher() : Node("shape_publisher") {
			publisher = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 0);

			RCLCPP_INFO(this->get_logger(), "Publishing marker...");
			visualization_msgs::msg::Marker marker;
			marker.header.frame_id = "map";
			marker.header.stamp = this->now();
			marker.lifetime = rclcpp::Duration::from_seconds(1);
			marker.ns = "my_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::msg::Marker::SPHERE;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.pose.position.x = 1;
			marker.pose.position.y = 1;
			marker.pose.position.z = 1;
			marker.pose.orientation.x = 0;
			marker.pose.orientation.y = 0;
			marker.pose.orientation.z = 0;
			marker.pose.orientation.w = 0;
			marker.scale.x = 1.5;
			marker.scale.y = 1.5;
			marker.scale.z = 1.5;
			marker.color.a = 0.5;
			marker.color.r = 0.5;
			marker.color.g = 1;
			marker.color.b = 1;
			publisher->publish(marker);
			RCLCPP_INFO(this->get_logger(), "Published marker.");
	}

private:
	std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> publisher;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ShapePublisher>());
	rclcpp::shutdown();
}
