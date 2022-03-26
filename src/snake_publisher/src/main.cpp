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
	// Clean up on CTRL+C
	std::signal(SIGINT, TermCleanUp);
	srand(time(0));

	rclcpp::init(argc, argv);
	auto node = std::make_shared<MarkerPublisher>();
	auto game = std::make_shared<GameNode>(node);

	// Set up ncurses to take input as fast as possible without echo
	initscr();
	noecho();
    nodelay(stdscr, TRUE);
	keypad(stdscr, TRUE);
	cbreak();
	clear();

	// Prompt
	printw("Press WASD to control the snake. Use ENTER to respawn.");
	refresh();

	// Run the nodes 
	auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
	executor->add_node(node); // Publisher for rviz2
	executor->add_node(game); // Game
	executor->spin();

	endwin();
	rclcpp::shutdown();
}
