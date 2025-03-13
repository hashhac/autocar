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
    int length_mm = 1000;
    int width_mm = 500;

    Box box(length_mm, width_mm);

    // Set target position in cm
    box.setTarget(50, 25);

    // Check distance to target from a given position in cm
    double distance = box.checkDistanceToTarget(30, 20);
    std::cout << "Distance to target: " << distance << " chunks" << std::endl;
    std::cout<< "Number of total chunks: " << box.getChunks() << std::endl;

    return 0;
}