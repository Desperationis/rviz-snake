#include <memory>
#include <thread>
#include <functional>
#include <chrono>
#include <vector>
#include <ncurses.h>
#include <csignal>
#include <list>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "Markers.hpp"
#include "SnakeGame.hpp"

void TermCleanUp(int signum) {
	if(signum == SIGINT) {
		endwin();
	}
	rclcpp::shutdown();
	abort();
}

int main(int argc, char** argv) {
	std::signal(SIGINT, TermCleanUp);

	rclcpp::init(argc, argv);
	auto node = std::make_shared<MarkerPublisher>();
	auto game = std::make_shared<GameNode>(node);

	initscr();
	noecho();
    nodelay(stdscr, TRUE);
	keypad(stdscr, TRUE);
	cbreak();
	clear();

	printw("Press WASD to control the snake.");
	refresh();

	auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
	executor->add_node(node);
	executor->add_node(game);
	executor->spin();

	endwin();
	rclcpp::shutdown();
}
