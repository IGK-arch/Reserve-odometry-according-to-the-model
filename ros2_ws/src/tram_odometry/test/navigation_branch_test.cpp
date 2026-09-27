#include "tram_odometry/navigation.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace tram_odometry;
void require(bool condition, const std::string & message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string & message) {
  require(std::isfinite(actual) && std::abs(actual - expected) < tolerance,
          message + ": actual=" + std::to_string(actual) + " expected=" + std::to_string(expected));
}
struct Fixture {
  std::filesystem::path dir = std::filesystem::temp_directory_path() / "tram_navigation_branch_test";
  std::filesystem::path primary = dir / "primary.csv";
  std::filesystem::path alternate = dir / "alternate.csv";
  std::filesystem::path curve = dir / "curve.csv";
  std::filesystem::path grade = dir / "grade.csv";
  Fixture() {
    std::filesystem::create_directories(dir);
    write(primary, "direction,s,x,y,z\nout,0,0,0,0\nout,200,200,0,0\n");
    write(alternate, "direction,s,x,y,z\nout,0,0,20,0\nout,200,200,20,0\n");
    write(curve, "direction,s,x,y,z\nout,0,0,0,0\nout,10,10,0,0\nout,20,10,10,0\n");
    write(grade, "direction,s,x,y,z\nout,0,0,0,0\nout,125,100,0,75\n");
  }
  ~Fixture() {std::filesystem::remove_all(dir);}
  static void write(const std::filesystem::path & path, const std::string & content) {
    std::ofstream file(path); file << content;
  }
  NavigationConfig config() const {
    NavigationConfig c;
    c.map_file = primary.string(); c.alternate_map_file = alternate.string();
    c.output_projection = "enu"; c.route_direction = "out";
    c.anchor_residual_decay_m = 0;
    return c;
  }
};
void fix(Navigation & nav, const NavigationConfig & c, const Point3 & enu,
         double stamp, bool rover = true, int status = 2) {
  geo::EnuDatum datum({c.map_datum_lat_deg, c.map_datum_lon_deg, c.map_datum_alt_m});
  const auto p = datum.fromEnu({enu.x, enu.y, enu.z});
  nav.submitFix(rover, p.latitude_deg, p.longitude_deg, p.height_m, status, stamp);
}
void observe(Navigation & nav, const NavigationConfig & c, const Point3 & enu,
             double stamp, bool rover = true, int status = 2) {
  nav.submitVehicle('C', 0, stamp);
  fix(nav, c, enu, stamp, rover, status);
  nav.submitVehicle('C', 0, stamp + .001);
}
void anchor(Navigation & nav, const NavigationConfig & c, double start = 100) {
  nav.submitVehicle('F', 0, start);
  for (int i = 0; i < 3; ++i) observe(nav, c, {50, 0, 0}, start + .1 * i, false);
  require(nav.output().anchored, "startup must anchor");
  near(nav.output().position.x, 59.873, .001, "startup base_link x");
}
Point3 roverAt(double master_s, double y = 20) {return {master_s + 12.436, y, 0};}
void branchWindow(Navigation & nav, const NavigationConfig & c, double start = 102) {
  for (int i = 0; i < 3; ++i) observe(nav, c, roverAt(50), start + .1 * i);
}
}  // namespace

int main() {
  Fixture fixture;
  std::vector<std::pair<std::string, std::function<void()>>> tests;
  tests.push_back({"rigid rover match on a curve retains master s", [&] {
    RouteMap map; require(map.load(fixture.curve.string()), "load curve");
    const double shift = 12.436 / std::sqrt(2.0);
    const Point3 query{10 + shift, shift, 0};
    const auto match = map.nearestAntenna(query, "out", 12.436, 10, 15);
    near(match.s, 10, 1e-9, "rover rigid s");
    near(match.distance_m, 0, 1e-9, "rover rigid residual");
    require(std::abs(map.nearest(query, "out").s - 12.436 - match.s) > 3,
            "fixture must expose the incorrect along-s antenna shift");
  }});
  tests.push_back({"antenna offset uses a unit 3D tangent", [&] {
    RouteMap map; require(map.load(fixture.grade.string()), "load grade");
    auto match = map.nearestAntenna({40 + 12.436 * .8, 0, 30 + 12.436 * .6},
                                    "out", 12.436, 50, 20);
    near(match.s, 50, 1e-9, "grade rigid s");
    near(match.distance_m, 0, 1e-9, "grade residual");
  }});
  tests.push_back({"matching stays inside longitudinal prior", [&] {
    RouteMap map; require(map.load(fixture.primary.string()), "load route");
    auto match = map.nearestAntenna({70, 0, 0}, "out", 0, 50, 5);
    near(match.s, 55, 1e-9, "prior clipped s");
    near(match.distance_m, 15, 1e-9, "prior clipped residual");
    require(map.nearestAntenna({70, 0, 0}, "out", 0, 500, 5).direction.empty(),
            "no candidate outside route extent");
    require(map.nearestAntenna({70, 0, 0}, "return", 0, 50, 5).direction.empty(),
            "do not use the opposite direction implicitly");
  }});
  tests.push_back({"three fresh rover fixes select alternate before active gate", [&] {
    auto c = fixture.config(); Navigation nav(EstimatorConfig{}, c); anchor(nav, c);
    observe(nav, c, roverAt(50), 102);
    require(nav.output().branch_switches == 0, "one fix must not switch");
    observe(nav, c, roverAt(50), 102.1);
    require(nav.output().branch_switches == 0, "two fixes must not switch");
    observe(nav, c, roverAt(50), 102.2);
    require(nav.output().branch == "alternate" && nav.output().branch_switches == 1,
            "third coherent fix must choose alternate despite 20m active residual");
    near(nav.output().position.x, 59.873, .001, "switch preserves master s");
    near(nav.output().position.y, 20, .001, "switch uses alternate XY");
    near(nav.state().distance_m, 0, 1e-9, "GNSS must not change wheel distance");
    near(nav.state().velocity_mps, 0, 1e-9, "GNSS must not change wheel velocity");
  }});
  tests.push_back({"ambiguous parallel or shared geometry cannot switch", [&] {
    auto c = fixture.config(); c.alternate_map_file = c.map_file;
    Navigation nav(EstimatorConfig{}, c); anchor(nav, c);
    for (int i = 0; i < 6; ++i) observe(nav, c, roverAt(50, 0), 102 + .1 * i);
    require(nav.output().branch_switches == 0, "equal candidates must stay primary");
  }});
  tests.push_back({"contradictory and separated fixes reset pending evidence", [&] {
    auto c = fixture.config(); Navigation nav(EstimatorConfig{}, c); anchor(nav, c);
    observe(nav, c, roverAt(50), 102);
    observe(nav, c, roverAt(50, 0), 102.1);
    observe(nav, c, roverAt(50), 102.2);
    observe(nav, c, roverAt(50), 104);
    observe(nav, c, roverAt(50), 104.1);
    require(nav.output().branch_switches == 0, "separate windows must not accumulate");
    observe(nav, c, roverAt(50), 104.2);
    require(nav.output().branch_switches == 1, "one complete new window must switch");
  }});
  tests.push_back({"stale future duplicate and invalid fixes provide no evidence", [&] {
    auto c = fixture.config(); Navigation nav(EstimatorConfig{}, c); anchor(nav, c);
    nav.submitVehicle('C', 0, 102);
    fix(nav, c, roverAt(50), 100);     // stale
    fix(nav, c, roverAt(50), 103);     // future beyond permitted tolerance
    fix(nav, c, roverAt(50), 102, true, -1);
    fix(nav, c, roverAt(50), 102);     // first valid observation
    fix(nav, c, roverAt(50), 102);     // duplicates cannot add evidence
    fix(nav, c, roverAt(50), 102);
    nav.submitVehicle('C', 0, 102.01);
    require(nav.output().branch_switches == 0, "bad fixes must not switch");
    require(nav.output().gnss_rejected >= 5, "bad fixes must be diagnosed");
    observe(nav, c, roverAt(50), 102.1);
    require(nav.output().branch_switches == 0, "only two genuine fixes exist");
    observe(nav, c, roverAt(50), 102.2);
    require(nav.output().branch_switches == 1, "valid window must still work after rejects");
  }});
  tests.push_back({"large backward reset clears map selection and GNSS timestamps", [&] {
    auto c = fixture.config(); Navigation nav(EstimatorConfig{}, c); anchor(nav, c);
    branchWindow(nav, c); require(nav.output().branch_switches == 1, "prepare alternate");
    nav.submitVehicle('C', 0, 10);
    require(!nav.output().anchored && nav.output().branch == "primary", "reset restores primary");
    require(nav.output().branch_switches == 0 && nav.output().gnss_corrections == 0 &&
            nav.output().gnss_rejected == 0, "reset clears diagnostics");
    anchor(nav, c, 10.1);
    near(nav.output().position.y, 0, .001, "reset uses original primary geometry");
    observe(nav, c, roverAt(50), 12);
    require(nav.output().branch_switches == 0, "reset clears old branch evidence");
  }});
  tests.push_back({"incoherent alternate fixes cannot confirm a branch", [&] {
    auto c = fixture.config(); Navigation nav(EstimatorConfig{}, c); anchor(nav, c);
    observe(nav, c, roverAt(30), 102);
    observe(nav, c, roverAt(70), 102.1);
    observe(nav, c, roverAt(35), 102.2);
    require(nav.output().branch_switches == 0,
            "three alternate fixes with 40m jumps are not a coherent GNSS window");
  }});
  tests.push_back({"clamped alternate match at prior boundary cannot switch", [&] {
    auto c = fixture.config(); Navigation nav(EstimatorConfig{}, c); anchor(nav, c);
    for (int i = 0; i < 3; ++i) observe(nav, c, roverAt(91), 102 + .1 * i);
    require(nav.output().branch_switches == 0,
            "an alternate candidate clipped to prior+40 is not a valid local match");
  }});
  tests.push_back({"nonfinite GNSS correction configuration is rejected", [&] {
    auto c = fixture.config();
    for (double NavigationConfig::* field : {
        &NavigationConfig::gnss_correction_max_age_s, &NavigationConfig::gnss_correction_gate_m,
        &NavigationConfig::gnss_correction_lateral_gate_m, &NavigationConfig::gnss_correction_gain,
        &NavigationConfig::gnss_correction_max_step_m}) {
      for (double value : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
        auto invalid = c; invalid.*field = value;
        bool threw = false;
        try {Navigation nav(EstimatorConfig{}, invalid);} catch (const std::invalid_argument &) {threw = true;}
        require(threw, "nonfinite correction configuration must throw");
      }
    }
  }});
  tests.push_back({"rover-only startup uses rigid curve geometry", [&] {
    auto c = fixture.config(); c.map_file = fixture.curve.string();
    // This artificial sharp corner explicitly defines body heading as local tangent.
    c.body_heading_lookahead_m = 0;
    c.alternate_map_file.clear(); c.rover_fallback_delay_s = 0;
    Navigation nav(EstimatorConfig{}, c);
    nav.submitVehicle('F', 0, 100);
    const double rover_shift = 12.436 / std::sqrt(2.0);
    for (int i = 0; i < 3; ++i)
      observe(nav, c, {10 + rover_shift, rover_shift, 0}, 100 + .1 * i);
    require(nav.output().anchored && nav.output().anchor_source == "rover_fallback",
            "rover-only startup must anchor");
    const double base_shift = 9.873 / std::sqrt(2.0);
    near(nav.output().position.x, 10 + base_shift, .002, "curved rover startup base x");
    near(nav.output().position.y, base_shift, .002, "curved rover startup base y");
  }});
  tests.push_back({"frozen GNSS position cannot cancel healthy wheel travel", [&] {
    auto c = fixture.config(); c.alternate_map_file.clear();
    Navigation nav(EstimatorConfig{}, c);
    nav.submitVehicle('F', 18, 100);  // 5 m/s; first wheel initializes velocity.
    auto drive = [&nav](double stamp) {
      nav.submitVehicle('C', 0, stamp);
      nav.submitVehicle('F', 18, stamp);
      nav.submitVehicle('R', 18, stamp);
    };
    for (int i = 0; i < 3; ++i) {
      const double stamp = 100 + .1 * i;
      drive(stamp);
      fix(nav, c, {50 + nav.state().distance_m, 0, 0}, stamp, false);
      nav.submitVehicle('C', 0, stamp + .001);
    }
    require(nav.output().anchored, "moving startup must anchor");
    for (int i = 3; i <= 10; ++i) drive(100 + .1 * i);
    nav.submitVehicle('C', 0, 101.001);
    const double before_distance = nav.state().distance_m;
    const double before_position = nav.output().position.x;
    const Point3 frozen = roverAt(50 + before_distance, 0);
    for (int i = 1; i <= 20; ++i) {
      const double stamp = 101 + .1 * i;
      drive(stamp);
      fix(nav, c, frozen, stamp);  // Fresh timestamps but repeated stale coordinates.
      nav.submitVehicle('C', 0, stamp + .001);
    }
    const double travelled = nav.state().distance_m - before_distance;
    const double published_travel = nav.output().position.x - before_position;
    require(travelled > 9, "fixture must actually drive for nearly 10 m");
    require(published_travel >= .8 * travelled,
            "frozen GNSS erased wheel travel: published=" + std::to_string(published_travel) +
            " wheel=" + std::to_string(travelled));
    require(nav.output().gnss_rejected > 0, "frozen fixes must be diagnosed");
  }});
  tests.push_back({"disabled rover input cannot anchor or correct", [&] {
    auto c = fixture.config(); c.use_rover_fallback = false; c.alternate_map_file.clear();
    Navigation nav(EstimatorConfig{}, c); nav.submitVehicle('F', 0, 100);
    for (int i = 0; i < 3; ++i) observe(nav, c, roverAt(50, 0), 102 + .1 * i);
    require(!nav.output().anchored, "disabled rover must not initialize the pure core");
    for (int i = 0; i < 3; ++i) observe(nav, c, {50, 0, 0}, 103 + .1 * i, false);
    require(nav.output().anchored && nav.output().anchor_source == "master",
            "master startup must remain available");
    const auto before = nav.output();
    for (int i = 0; i < 5; ++i) observe(nav, c, roverAt(58, 0), 104 + .1 * i);
    require(nav.output().gnss_corrections == before.gnss_corrections,
            "disabled rover must not provide periodic corrections");
    near(nav.output().position.x, before.position.x, .001, "disabled rover preserves pose");
  }});
  int failed = 0;
  for (const auto & test : tests) {
    try {test.second(); std::cout << "PASS: " << test.first << '\n';}
    catch (const std::exception & e) {++failed; std::cerr << "FAIL: " << test.first << ": " << e.what() << '\n';}
  }
  std::cout << tests.size() - failed << '/' << tests.size() << " navigation branch tests passed\n";
  return failed == 0 ? 0 : 1;
}
