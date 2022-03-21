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

	/**
	 * Determine whether a point is within the bounds of [0, side_length).
	*/ 
	bool InBounds(int x, int y) {
		return x < sideLength && x >= 0 && 
			y < sideLength && y >= 0;

	}

	void ReserveSnake(int x, int y) {
		if (this->InBounds(x, y)) {
			gridElements[y][x] = SNAKE;
		}
	}

	void ReserveFruit(int x, int y) {
		if (this->InBounds(x, y)) {
			gridElements[y][x] = FRUIT;
		}
	}

	/**
	 * Returns the side length of the grid.
	*/ 
	int GetSideLength() {
		return sideLength;
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
					cube.SetColor(1, 0, 0, 1);
					break;
				};

				cube.SetPos(i + worldXOffset, j + worldYOffset, 0); 
				grid.AddCube(cube);
			}
		}

		publisher->PublishMarker(grid);
	}
};

struct Fruit {
	Fruit(int x, int y) {
		p.x = x;
		p.y = y;
	}

	geometry_msgs::msg::Point p;
};

class Snake {
public:
	enum DIRECTION {LEFT, RIGHT, UP, DOWN};

	Snake(int x, int y) {
		Respawn(x, y);
	}

	std::list<geometry_msgs::msg::Point> GetBody() {
		return body;
	}

	void Update(SnakeGrid& snakeGrid, std::vector<Fruit>& fruits) {
		if(!dead) {
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
			
			dead = WillDie(nextPoint.x, nextPoint.y, snakeGrid); // TODO; Pass in grid

			if(!dead) {
				body.push_front(nextPoint);

				bool onFruit = false;
				for(size_t i = 0; i < fruits.size(); i++) {
					auto fruit = fruits[i];
					if(fruit.p.x == nextPoint.x && fruit.p.y == nextPoint.y) {
						onFruit = true;
						fruits.erase(fruits.begin() + i);
						break;
					}
				}

				if(!onFruit) {
					body.pop_back();
				}
			}
		}
	}

	void SetDirection(DIRECTION dir) {
		currentDirection = dir;
	}

	bool IsDead() {
		return dead;
	}

	bool WillDie(int x, int y, SnakeGrid& snakeGrid) {
		// TODO; Check bounds && self-intersection
		return !snakeGrid.InBounds(x, y);
	}

	void Respawn(int x, int y) {
		body.clear();
		
		geometry_msgs::msg::Point p;
		p.x = x;
		p.y = y;
		body.push_front(p);

		currentDirection = RIGHT;
	}

private:
	// First element is head, final element is tail.
	std::list<geometry_msgs::msg::Point> body;
	bool dead;
	DIRECTION currentDirection;
};


class GameNode : public rclcpp::Node {
public:
	GameNode(std::shared_ptr<MarkerPublisher> publisher) : Node("game_runner"), snake(5,5) {
		snake.SetDirection(Snake::RIGHT);
		this->renderTimer = this->create_wall_timer(80ms, std::bind(&GameNode::Loop, this));
		this->inputTimer = this->create_wall_timer(10ms, std::bind(&GameNode::UserInput, this));
		this->publisher = publisher;

		fruits.push_back(Fruit(1,1));
	}

	void Loop() {
		SnakeGrid grid(15);
		snake.Update(grid, fruits);
		for(auto body : snake.GetBody()) {
			grid.ReserveSnake(body.x, body.y);
		}

		for(auto fruit : fruits) {
			grid.ReserveFruit(fruit.p.x, fruit.p.y);
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
	std::vector<Fruit> fruits;
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
