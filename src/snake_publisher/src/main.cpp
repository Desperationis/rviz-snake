#include <memory>
#include <thread>
#include <functional>
#include <chrono>
#include <vector>
#include <ncurses.h>
#include <list>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "Markers.hpp"

using namespace std::chrono_literals;

/**
 * Wrapper for GridMarker so that origin is at (0,0), x-axis is positive to the
 * right, y-axis is positive downwards, and grid positions are forced to snap.
*/ 
class SnakeGrid {
private:
	int sideLength;
	int worldXOffset, worldYOffset;
	enum GRID_PIECES {EMPTY, SNAKE, FRUIT};
	std::vector<std::vector<GRID_PIECES>> gridElements;

public:
	SnakeGrid(int sideLength) {
		this->sideLength = sideLength;
		worldXOffset = -sideLength / 2;
		worldYOffset = -sideLength / 2;
		
		for(int i = 0; i < sideLength; i++) {
			gridElements.push_back(std::vector<GRID_PIECES>());
			for(int j = 0; j < sideLength; j++) {
				gridElements[i].push_back(EMPTY);
			}
		}
	}

	void ReserveSnake(int x, int y) {
		if (x < sideLength && y < sideLength) {
			gridElements[y][x] = SNAKE;
		}
	}

	void Draw(std::shared_ptr<MarkerPublisher> publisher) {
		GridMarker grid;
		for(size_t i = 0; i < gridElements.size(); i++) {
			for(size_t j = 0; j < gridElements[i].size(); j++) {
				GRID_PIECES type = gridElements[i][j];
				Cube cube;
				switch(type) {
				case EMPTY:
					break;
				case SNAKE:
					cube.SetColor(0, 1, 0, 1);
					break;
				case FRUIT:
					break;
				};

				cube.SetPos(i + worldXOffset, j + worldYOffset, 0); 
				grid.AddCube(cube);
			}
		}

		publisher->PublishMarker(grid);
	}
};

class Snake {
public:
	enum DIRECTION {LEFT, RIGHT, UP, DOWN};

	Snake(int x, int y) {
		geometry_msgs::msg::Point p;
		p.x = x;
		p.y = y;
		body.push_front(p);

		currentDirection = RIGHT;
	}

	std::list<geometry_msgs::msg::Point> GetBody() {
		return body;
	}

	void Update() {
		geometry_msgs::msg::Point nextPoint;
		nextPoint = body.front();
		switch(currentDirection) {
		case LEFT:
			nextPoint.x -= 1;
			break;
		case RIGHT:
			nextPoint.x += 1;
			break;
		case UP:
			nextPoint.y -= 1;
			break;
		case DOWN:
			nextPoint.y += 1;
			break;
		};
		body.push_front(nextPoint);
		body.pop_back();
	}

	void SetDirection(DIRECTION dir) {
		currentDirection = dir;
	}

private:
	// First element is head, final element is tail.
	std::list<geometry_msgs::msg::Point> body;
	DIRECTION currentDirection;
};


class GameNode : public rclcpp::Node {
public:
	GameNode(std::shared_ptr<MarkerPublisher> publisher) : Node("game_runner"), snake(5,5) {
		snake.SetDirection(Snake::RIGHT);
		this->timer = this->create_wall_timer(1000ms, std::bind(&GameNode::Loop, this));
		this->publisher = publisher;
	}

	void Loop() {
		SnakeGrid grid(15);
		for(auto body : snake.GetBody()) {
			grid.ReserveSnake(body.x, body.y);
		}

		snake.Update();
		grid.Draw(publisher);
	}

private:
	Snake snake;
	rclcpp::TimerBase::SharedPtr timer;
	std::shared_ptr<MarkerPublisher> publisher;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MarkerPublisher>();
	auto game = std::make_shared<GameNode>(node);

	auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
	executor->add_node(node);
	executor->add_node(game);
	executor->spin();

	rclcpp::shutdown();
}
