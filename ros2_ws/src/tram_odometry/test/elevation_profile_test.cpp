#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include "tram_odometry/elevation_profile.hpp"

int main(int argc, char ** argv) {
  namespace fs = std::filesystem;
  const auto fixture = fs::temp_directory_path() / "tram_elevation_profile_test.csv";
  auto write = [&fixture](const std::string & text) {
    std::ofstream file(fixture);
    file << text;
  };
  tram_odometry::ElevationProfile profile;
  assert(profile.empty());
  assert(profile.sample(5.0, 0.0, 123.0) == 123.0);
  write("x,y,z\n0,0,100\n10,0,110\n20,0,110\n");
  assert(profile.load(fixture.string()));
  assert(profile.size() == 3);
  // Values are already base_link altitude; no antenna offset is applied.
  assert(std::abs(profile.sample(5.0, 0.0, 200.0) - 105.0) < 1e-9);
  assert(std::abs(profile.sample(5.0, 4.0, 200.0) - 105.0) < 1e-9);
  // Opposite rails use the same surveyed altitude with a 5..10 m fade.
  assert(std::abs(profile.sample(5.0, 7.5, 200.0) - 152.5) < 1e-9);
  assert(profile.sample(5.0, 10.0, 200.0) == 200.0);
  assert(profile.sample(5.0, 100.0, 200.0) == 200.0);
  assert(profile.sample(-11.0, 0.0, 200.0) == 200.0);
  assert(profile.sample(31.0, 0.0, 200.0) == 200.0);
  assert(profile.sample(std::numeric_limits<double>::quiet_NaN(), 0.0, 200.0) == 200.0);
  assert(profile.sample(5.0, std::numeric_limits<double>::infinity(), 200.0) == 200.0);
  // A failed reload disables the profile rather than retaining a stale datum.
  write("x,y,z\n0,0,100\n10,0,nan\n");
  assert(!profile.load(fixture.string()));
  assert(profile.empty());
  assert(profile.sample(5.0, 0.0, 200.0) == 200.0);
  write("x,y,z\n0,0,100\n10,0,110junk\n");
  assert(!profile.load(fixture.string()));
  write("x,y,z\n0,0,100\n0,0,110\n");
  assert(!profile.load(fixture.string()));
  write("x,y,z\n0,0,100\n");
  assert(!profile.load(fixture.string()));
  write("x,y,z\r\n0,0,100\r\n10,0,110\r\n");
  assert(profile.load(fixture.string()));
  assert(!profile.load(fixture.string() + ".missing"));
  fs::remove(fixture);
  if (argc > 1) {
    assert(profile.load(argv[1]));
    assert(profile.size() == 4709);
    assert(std::abs(profile.sample(99156.1816, 85020.8159, 0.0) - 172.1878) < 1e-6);
    assert(std::abs(profile.sample(103346.625, 85631.4236, 0.0) - 168.41287) < 1e-6);
    // The terminal loops are absent from the official profile.
    assert(profile.sample(103635.9, 86057.5, 165.3) == 165.3);
    assert(profile.sample(99015.6, 84949.1, 171.3) == 171.3);
  }
  std::cout << "elevation_profile_test passed\n";
}
