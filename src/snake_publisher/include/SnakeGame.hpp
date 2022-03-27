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
 * Details the color of each game piece.
*/ 
struct GamePieceColors {
	std_msgs::msg::ColorRGBA snakeColor;
	std_msgs::msg::ColorRGBA fruitColor;
	std_msgs::msg::ColorRGBA gridColor;
};


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
	void Draw(std::shared_ptr<MarkerPublisher> publisher, GamePieceColors colors) {
		GridMarker grid;
		for(size_t i = 0; i < gridElements.size(); i++) {
			for(size_t j = 0; j < gridElements[i].size(); j++) {
				GRID_PIECES type = gridElements[i][j];
				Cube cube;
				cube.SetPos(i + worldXOffset, j + worldYOffset, 1); 
				
				switch(type) {
				case EMPTY:
					cube.SetPos(i + worldXOffset, j + worldYOffset, 0); 
					cube.color = colors.gridColor;
					break;
				case SNAKE:
					cube.color = colors.snakeColor;
					break;
				case FRUIT:
					cube.color = colors.fruitColor;
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
	FruitManager() {
		requestedFruit = 0;
	}

	/**
	 * Randomly spawn a single fruit that is not occupied by a SNAKE tile.
	 * TODO: Prevent this from being an infinite loop should a player good
	 * enough completely fill up the board.
	*/ 
	void SpawnFruit(SnakeGrid snakeGrid) {
		while(requestedFruit > 0) {
			int x = rand() % snakeGrid.GetSideLength();
			int y = rand() % snakeGrid.GetSideLength();

			if(snakeGrid.GetReserved(x, y) == SnakeGrid::EMPTY) {
				fruits.push_back(Fruit(x, y));
				snakeGrid.ReserveFruit(x, y);
				requestedFruit--;
			}
		}
	}

	/**
	 * Request a single fruit to be drawn the next frame. Can be called
	 * multiple times for multiple fruits.
	*/ 
	void RequestFruit() {
		requestedFruit++;
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

	// This variable holds the amount of requested fruit that should be spawned
	// on the next frame.
	int requestedFruit;
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
			actualDirection = currentDirection;
			
			dead = WillDie(nextPoint.x, nextPoint.y, snakeGrid);

			if(!dead) {
				body.push_front(nextPoint);

				bool fruitEaten = fruitManager.Eat(nextPoint.x, nextPoint.y);

				if(!fruitEaten) {
					body.pop_back();
				}
				else {
					fruitManager.RequestFruit();
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

	DIRECTION GetActualDirection() {
		return actualDirection;
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
		dead = false;

		currentDirection = RIGHT;
	}

private:
	// First element is head, final element is tail.
	std::list<geometry_msgs::msg::Point> body;
	bool dead;
	DIRECTION currentDirection;

	// User input and the game are asynchronous as run at different hertz, so
	// this gives the direction the snake is headed based on the last frame so
	// that the user doesn't circumnavigate the input-checking code to prevent
	// crashing directly backwards.
	DIRECTION actualDirection; 
};


/**
 * ROS Node that actually runs the game. Due to not being able to create a
 * standard game loop with a delay, this node runs the loop via wall timers and
 * asynchronously receives input from the terminal via ncurses.
*/ 
class GameNode : public rclcpp::Node {
private:
	void OnGameFPSChange(const rclcpp::Parameter& p) {
		SetGameFPS(p.as_int());
	}

	void OnInputFPSChange(const rclcpp::Parameter& p) {
		SetInputFPS(p.as_int());
	}

	void SetGameFPS(int FPS) {
		int millisecondsToDelay = static_cast<int>(1000.0 / FPS);
		auto duration = std::chrono::duration<int, std::milli>(millisecondsToDelay);
		this->renderTimer = this->create_wall_timer(duration, std::bind(&GameNode::Loop, this));
	}

	void SetInputFPS(int FPS) {
		int millisecondsToDelay = static_cast<int>(1000.0 / FPS);
		auto duration = std::chrono::duration<int, std::milli>(millisecondsToDelay);
		this->inputTimer = this->create_wall_timer(duration, std::bind(&GameNode::UserInput, this));
	}

public:
	GameNode(std::shared_ptr<MarkerPublisher> publisher) : Node("game_node"), snake(5,5), snakeGrid(15) {
		snake.SetDirection(Snake::RIGHT);

		this->declare_parameter("game_fps", 12);
		this->declare_parameter("input_fps", 100);

		this->declare_parameter("snake_color_r", 0);
		this->declare_parameter("snake_color_g", 255);
		this->declare_parameter("snake_color_b", 0);

		this->declare_parameter("fruit_color_r", 255);
		this->declare_parameter("fruit_color_g", 0);
		this->declare_parameter("fruit_color_b", 0);

		this->declare_parameter("grid_color_r", 255);
		this->declare_parameter("grid_color_g", 255);
		this->declare_parameter("grid_color_b", 255);

		this->SetGameFPS(get_parameter("game_fps").as_int());
		this->SetInputFPS(get_parameter("input_fps").as_int());

		// Handle parameter changes
		paramSubscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
		gameFPSHandle = paramSubscriber->add_parameter_callback("game_fps", std::bind(&GameNode::OnGameFPSChange, this, std::placeholders::_1));
		inputFPSHandle = paramSubscriber->add_parameter_callback("input_fps", std::bind(&GameNode::OnInputFPSChange, this, std::placeholders::_1));

		this->publisher = publisher;

		fruitManager.RequestFruit();
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

		fruitManager.SpawnFruit(snakeGrid);
		for(auto fruit : fruitManager.GetFruits()) {
			snakeGrid.ReserveFruit(fruit.p.x, fruit.p.y);
		}

		snakeGrid.Draw(publisher, GetColors());
		snakeGrid.Clear();
	}

	/**
	 * Sole place for handling user input.
	*/ 
	void UserInput() {
		int c = getch();
		auto currentDirection = snake.GetActualDirection();
		if (c != -1) {
			if(c == 'w' && currentDirection != Snake::DOWN)
				snake.SetDirection(Snake::UP);
			if(c == 'a' && currentDirection != Snake::RIGHT)
				snake.SetDirection(Snake::LEFT);
			if(c == 's' && currentDirection != Snake::UP)
				snake.SetDirection(Snake::DOWN);
			if(c == 'd' && currentDirection != Snake::LEFT)
				snake.SetDirection(Snake::RIGHT);
			if(c == '\n') 
				snake.Respawn(5,5);
		}
	}

	GamePieceColors GetColors() {
		GamePieceColors colors;
		colors.snakeColor.r = this->get_parameter("snake_color_r").as_int() / 255;
		colors.snakeColor.g = this->get_parameter("snake_color_g").as_int() / 255;
		colors.snakeColor.b = this->get_parameter("snake_color_b").as_int() / 255;
		colors.snakeColor.a = 1;
		
		colors.gridColor.r = this->get_parameter("grid_color_r").as_int() / 255;
		colors.gridColor.g = this->get_parameter("grid_color_g").as_int() / 255;
		colors.gridColor.b = this->get_parameter("grid_color_b").as_int() / 255;
		colors.gridColor.a = 1;

		colors.fruitColor.r = this->get_parameter("fruit_color_r").as_int() / 255;
		colors.fruitColor.g = this->get_parameter("fruit_color_g").as_int() / 255;
		colors.fruitColor.b = this->get_parameter("fruit_color_b").as_int() / 255;
		colors.fruitColor.a = 1;

		return colors;
	}

private:
	Snake snake;
	SnakeGrid snakeGrid;
	std::shared_ptr<MarkerPublisher> publisher;
	FruitManager fruitManager;


	rclcpp::TimerBase::SharedPtr renderTimer, inputTimer;
	std::shared_ptr<rclcpp::ParameterEventHandler> paramSubscriber;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> gameFPSHandle;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> inputFPSHandle;
};

#endif
