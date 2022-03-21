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
		this->renderTimer = this->create_wall_timer(80ms, std::bind(&GameNode::Loop, this));
		this->inputTimer = this->create_wall_timer(10ms, std::bind(&GameNode::UserInput, this));
		this->publisher = publisher;
	}

	void Loop() {
		SnakeGrid grid(15);
		snake.Update();
		for(auto body : snake.GetBody()) {
			grid.ReserveSnake(body.x, body.y);
		}

		grid.Draw(publisher);
	}

	void UserInput() {
		int c = getch();
		if (c != -1) {
			if(c == 'w')
				snake.SetDirection(Snake::UP);
			if(c == 'a')
				snake.SetDirection(Snake::LEFT);
			if(c == 's')
				snake.SetDirection(Snake::DOWN);
			if(c == 'd')
				snake.SetDirection(Snake::RIGHT);
		}
	}

private:
	Snake snake;
	rclcpp::TimerBase::SharedPtr renderTimer, inputTimer;
	std::shared_ptr<MarkerPublisher> publisher;
};

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
