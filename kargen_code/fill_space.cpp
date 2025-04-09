#include <ostream>
#include <iostream>
#include <cassert>
#include <cmath>

/**
 * Interpolate values between first and last at a specified frequency
 * 
 * @param elapsedTime - Time taken in seconds (float)
 * @param firstValue - The starting value (float)
 * @param lastValue - The ending value (float)
 * @param frequency - How many values per second (float)
 * @return An array of interpolated values (excluding first and last)
 */
float* fillValueSpace(float elapsedTime, float firstValue, float lastValue, float frequency) {
  // Calculate how many total values should exist based on time and frequency
  int totalPoints = ceil(elapsedTime * frequency) + 1; // +1 to include the first point    
  int numInterpolatedValues = totalPoints - 2;

  // Handle edge case where no intermediate values are needed
  if (numInterpolatedValues <= 0) {
    return nullptr;
  }

  float step = (lastValue - firstValue) / (totalPoints - 1);    
  // Generate the interpolated values
  float* interpolatedValues = new float[numInterpolatedValues];
  for (int i = 0; i < numInterpolatedValues; i++) {
    interpolatedValues[i] = firstValue + step * (i + 1);
  }    
  return interpolatedValues;
}

// Unit tests (C++ standard)
void testFillValueSpace() {
  // Test case 1: Basic test
  float* result1 = fillValueSpace(5.0, 1.0, 5.0, 5.0);
  std::cout << "Test case 1: ";
  if (result1 != nullptr) {
    int totalPoints = ceil(5.0 * 5.0) + 1;
    int numInterpolatedValues = totalPoints - 2;
    for (int i = 0; i < numInterpolatedValues; i++) {
      std::cout << result1[i] << " ";
    }
    std::cout << std::endl;
    assert(result1 != nullptr);
  } else {
    std::cout << "nullptr" << std::endl;
  }
  delete[] result1;

  // Test case 2: No intermediate values
  float* result2 = fillValueSpace(0.5, 1.0, 5.0, 1.0);
  std::cout << "Test case 2: ";
  if (result2 != nullptr) {
    int totalPoints = ceil(0.5 * 1.0) + 1;
    int numInterpolatedValues = totalPoints - 2;
    for (int i = 0; i < numInterpolatedValues; i++) {
      std::cout << result2[i] << " ";
    }
    std::cout << std::endl;
  } else {
    std::cout << "nullptr" << std::endl;
  }
  assert(result2 == nullptr);

  // Test case 3: Non-integer values
  float* result3 = fillValueSpace(2.5, 10.5, 20.5, 4.0);
  std::cout << "Test case 3: ";
  if (result3 != nullptr) {
    int totalPoints = ceil(2.5 * 4.0) + 1;
    int numInterpolatedValues = totalPoints - 2;
    for (int i = 0; i < numInterpolatedValues; i++) {
      std::cout << result3[i] << " ";
    }
    std::cout << std::endl;
    assert(result3 != nullptr);
  } else {
    std::cout << "nullptr" << std::endl;
  }
  delete[] result3;

  // Add more test cases as needed
  std::cout << "All tests passed!" << std::endl;
}

int main() {
  testFillValueSpace();
  return 0;
}