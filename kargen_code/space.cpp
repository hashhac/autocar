#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

class Box {
public:
    Box(int length_mm, int width_mm) {
        // Convert dimensions from mm to cm and divide by 10 to get chunks
        length_chunks_ = length_mm / 10;
        width_chunks_ = width_mm / 10;

        // Initialize the 2D array
        array_.resize(length_chunks_, std::vector<int>(width_chunks_, 0));
        box_clear();
    }
    void box_clear(){
        //fills the top of the array sides and base with 1s
        for (int i = 0; i < length_chunks_; ++i) {
            for (int j = 0; j < width_chunks_; ++j) {
                if (i == 0 || i == length_chunks_ - 1 || j == 0 || j == width_chunks_ - 1) {
                    array_[i][j] = 1;
                } else {
                    array_[i][j] = 0;
                }
            }
        }
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
    void printBox() {
        for (int i = 0; i < length_chunks_; ++i) {
            for (int j = 0; j < width_chunks_; ++j) {
                cout << array_[i][j] << " ";
            }
            cout << endl;
        }
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
    cout << "Total chunks: " << box.getChunks() << endl; 
    box.printBox();
    return 0;
}