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
public:
	enum GRID_PIECES {EMPTY, SNAKE, FRUIT};
private:
	int sideLength;
	int worldXOffset, worldYOffset;
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

	GRID_PIECES GetReserved(int x, int y) {
		return gridElements[y][x];
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
					cube.SetPos(i + worldXOffset, j + worldYOffset, 0); 
					break;
				case SNAKE:
					cube.SetPos(i + worldXOffset, j + worldYOffset, 1); 
					cube.SetColor(0, 1, 0, 1);
					break;
				case FRUIT:
					cube.SetPos(i + worldXOffset, j + worldYOffset, 1); 
					cube.SetColor(1, 0, 0, 1);
					break;
				};

				grid.AddCube(cube);
			}
		}

		publisher->PublishMarker(grid);
	}

	void Clear() {
		for(int i = 0; i < sideLength; i++) {
			for(int j = 0; j < sideLength; j++) {
				gridElements[i][j] = EMPTY;
			}
		}
	}
};

struct Fruit {
	Fruit(int x, int y) {
		p.x = x;
		p.y = y;
	}

	geometry_msgs::msg::Point p;
};

class FruitManager {
public:
	FruitManager() {}

	/**
	 * Randomly spawn a single fruit that is not occupied.
	*/ 
	void SpawnFruit(SnakeGrid snakeGrid) {
		while(true) {
			int x = rand() % snakeGrid.GetSideLength();
			int y = rand() % snakeGrid.GetSideLength();

			if(snakeGrid.GetReserved(x, y) == SnakeGrid::EMPTY) {
				fruits.push_back(Fruit(x, y));
				break;
			}
		}
	}

	/**
	 * Try to eat a fruit at a specific point. If there is not fruit at that
	 * point, return false. If there is, return true and erase the fruit from
	 * existence.
	*/ 
	bool Eat(int x, int y) { 
		for(size_t i = 0; i < fruits.size(); i++) {
			auto fruit = fruits[i];
			if(fruit.p.x == x && fruit.p.y == y) {
				fruits.erase(fruits.begin() + i);
				return true;
			}
		}

		return false;
	}

	std::vector<Fruit> GetFruits() {
		return fruits;
	}


private:
	std::vector<Fruit> fruits;
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

	void Update(SnakeGrid& snakeGrid, FruitManager& fruitManager) {
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

				bool fruitEaten = fruitManager.Eat(nextPoint.x, nextPoint.y);

				if(!fruitEaten) {
					body.pop_back();
				}
				else {
					fruitManager.SpawnFruit(snakeGrid);
				}
			}
		}
	}

	void SetDirection(DIRECTION dir) {
		currentDirection = dir;
	}

	DIRECTION GetDirection() {
		return currentDirection;
	}

	bool IsDead() {
		return dead;
	}

	bool WillDie(int x, int y, SnakeGrid& snakeGrid) {
		// TODO; Check bounds && self-intersection
		if(!snakeGrid.InBounds(x, y)) 
			return true;

		for(auto point : body) {
			if (point.x == x && point.y == y) {
				return true;
			}
		}

		return false;
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
	GameNode(std::shared_ptr<MarkerPublisher> publisher) : Node("game_runner"), snake(5,5), snakeGrid(15) {
		snake.SetDirection(Snake::RIGHT);
		this->renderTimer = this->create_wall_timer(80ms, std::bind(&GameNode::Loop, this));
		this->inputTimer = this->create_wall_timer(10ms, std::bind(&GameNode::UserInput, this));
		this->publisher = publisher;

		fruitManager.SpawnFruit(snakeGrid);
	}

	void Loop() {
		snake.Update(snakeGrid, fruitManager);
		for(auto body : snake.GetBody()) {
			snakeGrid.ReserveSnake(body.x, body.y);
		}

		for(auto fruit : fruitManager.GetFruits()) {
			snakeGrid.ReserveFruit(fruit.p.x, fruit.p.y);
		}

		snakeGrid.Draw(publisher);
		snakeGrid.Clear();
	}

	void UserInput() {
		int c = getch();
		auto currentDirection = snake.GetDirection();
		if (c != -1) {
			if(c == 'w' && currentDirection != Snake::DOWN)
				snake.SetDirection(Snake::UP);
			if(c == 'a' && currentDirection != Snake::RIGHT)
				snake.SetDirection(Snake::LEFT);
			if(c == 's' && currentDirection != Snake::UP)
				snake.SetDirection(Snake::DOWN);
			if(c == 'd' && currentDirection != Snake::LEFT)
				snake.SetDirection(Snake::RIGHT);
		}
	}

private:
	Snake snake;
	SnakeGrid snakeGrid;
	rclcpp::TimerBase::SharedPtr renderTimer, inputTimer;
	std::shared_ptr<MarkerPublisher> publisher;
	FruitManager fruitManager;
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
