#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "Markers.hpp"

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
		gridElements[y][x] = SNAKE;
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


int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MarkerPublisher>();

	SnakeGrid grid(15);
	grid.Draw(node);

	rclcpp::spin(node);
	rclcpp::shutdown();
}
