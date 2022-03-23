#ifndef SNAKEGAME_HPP
#define SNAKEGAME_HPP

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
 * Snake.hpp
 *
 * This files holds classes that directly relate to running "Snake". Everything
 * is managed interally as simply points, and only get turned into cubes the
 * moment they are rendered; This allows for multiple possible ways to render
 * the game.
 */ 


/**
 * Wrapper for GridMarker so that origin is at the topleft, x-axis is positive
 * to the right, y-axis is positive downwards, and the grid is drawn. This is
 * where points are turned into cubes.
*/ 
class SnakeGrid {
public:
	enum GRID_PIECES {EMPTY, SNAKE, FRUIT};
private:
	int sideLength; 
	int worldXOffset, worldYOffset;
	std::vector<std::vector<GRID_PIECES>> gridElements;
public:
	/**
	 * Creates a grid that is at maximum size `sideLength` on each side.
	*/ 
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

	/**
	 * Reserve a specific spot to be drawn as a snake in the next frame, given
	 * that it is in bounds. If it is not in bounds, nothing will happen.
	*/ 
	void ReserveSnake(int x, int y) {
		if (this->InBounds(x, y)) {
			gridElements[y][x] = SNAKE;
		}
	}

	/**
	 * Reserve a specific spot to be drawn as a fruit in the next frame, given
	 * that it is in bounds. If it is not in bounds, nothing will happen.
	*/ 
	void ReserveFruit(int x, int y) {
		if (this->InBounds(x, y)) {
			gridElements[y][x] = FRUIT;
		}
	}

	/**
	 * Get the type of piece that is currently reserved. By default, every
	 * square on the grid is reserved EMPTY on every frame.
	*/ 
	GRID_PIECES GetReserved(int x, int y) {
		return gridElements[y][x];
	}

	/**
	 * Returns the side length of the grid.
	*/ 
	int GetSideLength() {
		return sideLength;
	}

	/**
	 * Draws the grid by iterating through all reserved portions and drawing a
	 * specific cube based on its type. For SNAKE and FRUIT, the square is
	 * elevated by 1 unit in order to give the apperance of 3D.
	*/ 
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

	/**
	 * Completely clears the grid with EMPTY tiles.
	*/ 
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

/**
 * Manager for controlling the spawning and erasing of fruits on the board.
*/ 
class FruitManager {
public:
	FruitManager() {}

	/**
	 * Randomly spawn a single fruit that is not occupied by a SNAKE tile.
	 * TODO: Prevent this from being an infinite loop should a player good
	 * enough completely fill up the board.
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

	/**
	 * Get the list of points that occupy fruits.
	*/ 
	std::vector<Fruit> GetFruits() {
		return fruits;
	}


private:
	std::vector<Fruit> fruits;
};

/**
 * The Snake. Body is completely represented by points.
*/
class Snake {
public:
	enum DIRECTION {LEFT, RIGHT, UP, DOWN};

	Snake(int x, int y) {
		Respawn(x, y);
	}

	std::list<geometry_msgs::msg::Point> GetBody() {
		return body;
	}

	/**
	 * Updates the snake by a single frame. Essentially, it determines when it
	 * dies, how to move, and how to eat fruits.
	*/ 
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

	/**
	 * Checks whether or not the head is out of bounds our intersected its own
	 * body to determine if it died.
	*/ 
	bool WillDie(int x, int y, SnakeGrid& snakeGrid) {
		if(!snakeGrid.InBounds(x, y)) 
			return true;

		for(auto point : body) {
			if (point.x == x && point.y == y) {
				return true;
			}
		}

		return false;
	}

	/**
	 * Respawn the snake at a specific point.
	*/ 
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


/**
 * ROS Node that actually runs the game. Due to not being able to create a
 * standard game loop with a delay, this node runs the loop via wall timers and
 * asynchronously receives input from the terminal via ncurses.
*/ 
class GameNode : public rclcpp::Node {
public:
	GameNode(std::shared_ptr<MarkerPublisher> publisher) : Node("game_runner"), snake(5,5), snakeGrid(15) {
		snake.SetDirection(Snake::RIGHT);
		this->renderTimer = this->create_wall_timer(80ms, std::bind(&GameNode::Loop, this));
		this->inputTimer = this->create_wall_timer(10ms, std::bind(&GameNode::UserInput, this));
		this->publisher = publisher;

		fruitManager.SpawnFruit(snakeGrid);
	}

	/**
	 * Game loop.
	*/
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

	/**
	 * Sole place for handling user input.
	*/ 
	void UserInput() {
		int c = getch();
		auto currentDirection = snake.GetDirection();
		if (c != -1) {
			// TODO: Fix bug where mashing multiple inputs in one frame
			// bypasses the "don't go backwards" rule and you die due to
			// turning 180 degrees.
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

#endif
