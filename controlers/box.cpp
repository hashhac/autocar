#include <iostream>
#include <cmath>
#include <vector>

class Box {
public:
    Box(int length_mm, int width_mm) {
        // Convert dimensions from mm to cm and divide by 10 to get chunks
        length_chunks_ = length_mm / 100;
        width_chunks_ = width_mm / 100;

        // Initialize the 2D array
        array_.resize(length_chunks_, std::vector<int>(width_chunks_, 0));
    }

    void setTarget(int x_cm, int y_cm) {
        target_x_ = x_cm / 10;
        target_y_ = y_cm / 10;
    }

    double checkDistanceToTarget(int x_cm, int y_cm) {
        int x_chunk = x_cm / 10;
        int y_chunk = y_cm / 10;
        double distance = std::sqrt(std::pow(target_x_ - x_chunk, 2) +
                                    std::pow(target_y_ - y_chunk, 2));
        return distance;
    }

    void markReached(int x_cm, int y_cm) {
        int x_chunk = x_cm / 10;
        int y_chunk = y_cm / 10;
        if (x_chunk >= 0 && x_chunk < length_chunks_ && y_chunk >= 0 && y_chunk < width_chunks_) {
            array_[x_chunk][y_chunk] = 1;
        }
    }

    std::pair<int, int> findNearestZero(int x_cm, int y_cm) {
        int x_chunk = x_cm / 10;
        int y_chunk = y_cm / 10;
        double min_distance = std::numeric_limits<double>::max();
        std::pair<int, int> nearest(-1, -1);

        for (int i = 0; i < length_chunks_; ++i) {
            for (int j = 0; j < width_chunks_; ++j) {
                if (array_[i][j] == 0) {
                    double distance = std::sqrt(std::pow(i - x_chunk, 2) + std::pow(j - y_chunk, 2));
                    if (distance < min_distance) {
                        min_distance = distance;
                        nearest = {i, j};
                    }
                }
            }
        }

        return nearest;
    }

    int getChunks(void) {
        return length_chunks_ * width_chunks_;
    }

private:
    int length_chunks_;
    int width_chunks_;
    int target_x_;
    int target_y_;
    std::vector<std::vector<int>> array_;
};

int main() {
    // Dimensions in mm
    int length_mm = 1200;
    int width_mm = 2000;

    Box box(length_mm, width_mm);

    // Set target position in cm
    // Set target position in cm
    box.setTarget(50, 25);

    // Check distance to target from a given position in cm
    double distance = box.checkDistanceToTarget(30, 20);
    std::cout << "Distance to target: " << distance << " chunks" << std::endl;
    std::cout << "Number of total chunks: " << box.getChunks() << std::endl;

    // Mark a position as reached
    box.markReached(0, 20);

    // Find the nearest non-zero cube from a given position
    auto nearest = box.findNearestZero(0, 20);
    if (nearest.first != -1 && nearest.second != -1) {
        std::cout << "Nearest non-zero cube is at: (" << nearest.first * 10 << " cm, " << nearest.second * 10 << " cm)" << std::endl;
    } else {
        std::cout << "No non-zero cubes found." << std::endl;
    }

    return 0;
}