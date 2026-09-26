#include <cassert>
#include <cmath>
#include <iostream>

#include "tram_odometry/uncertainty.hpp"

int main() {
  using tram_odometry::commonScaleDistanceVariance;
  assert(commonScaleDistanceVariance(100.0, 100.0, 0.01) == 0.0);
  assert(commonScaleDistanceVariance(99.0, 100.0, 0.01) == 0.0);
  assert(commonScaleDistanceVariance(5100.0, 100.0, 0.0) == 0.0);
  assert(std::abs(commonScaleDistanceVariance(5100.0, 100.0, 0.01) -
                  2500.0) < 1e-9);
  assert(std::abs(commonScaleDistanceVariance(2600.0, 100.0, 0.01) -
                  625.0) < 1e-9);
  std::cout << "PASS: common wheel-scale uncertainty grows with distance\n";
}
