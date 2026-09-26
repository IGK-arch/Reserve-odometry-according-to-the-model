#include <cassert>
#include <iostream>

#include "tram_odometry/startup_output_gate.hpp"

int main() {
  using tram_odometry::shouldHoldPositionForStartup;

  // GNSS arrives 40 ms after the first input: suppress temporary relative
  // coordinates, then publish a correctly anchored current position.
  assert(shouldHoldPositionForStartup(true, false, 100.02, 100.0, 5.0));
  assert(!shouldHoldPositionForStartup(true, true, 100.04, 100.0, 5.0));

  // If startup GNSS is absent, switch to relative position after the window.
  assert(shouldHoldPositionForStartup(true, false, 104.99, 100.0, 5.0));
  assert(!shouldHoldPositionForStartup(true, false, 105.01, 100.0, 5.0));

  // An unanchored short bag cannot be scored in absolute coordinates.
  assert(shouldHoldPositionForStartup(true, false, 101.30, 100.0, 5.0));
  // Operators can explicitly request immediate relative odometry.
  assert(!shouldHoldPositionForStartup(false, false, 100.02, 100.0, 5.0));

  std::cout << "PASS: startup position gate (GNSS, no GNSS, short bag)\n";
}
