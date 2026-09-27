// Independent read-only review fixture. Compile with -DHAS_KALMAN=1 for KF.
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#define private public
#include "tram_odometry/estimator.hpp"
#undef private
using namespace tram_odometry;
int main() {
  std::cout<<std::setprecision(17);
  std::cout<<"vehicle,hz,braking,noise,min_cov_determinant,nonfinite_cov,negative_cov,max_stop_residual,dropout_max_speed,dropout_distance,reset_residual,reset_cross\n";
  for(int vehicle:{30618,30639}) for(int hz:{5,10,50})
  for(bool braking:{false,true}) for(double noise:{.04,.054}) {
    auto c=EstimatorConfig::forVehicle(vehicle);Estimator e(c);e.reset(100,braking?6:0);
    double min_det=INFINITY,max_stop_residual=0;int nonfinite=0,negative=0;
    auto inspect=[&](bool stopped) {
#if HAS_KALMAN
      double det=e.velocity_variance_*e.kalman_aa_-e.kalman_va_*e.kalman_va_;
      min_det=std::min(min_det,det);nonfinite+=!std::isfinite(det);negative+=det < -1e-12;
      if(stopped) max_stop_residual=std::max(max_stop_residual,std::abs(e.kalman_residual_mps2_));
#endif
    };
    const double duration=braking?14:10;
    for(int i=1;i<=int(duration*hz);++i) {
      double elapsed=double(i)/hz,t=100+elapsed;
      double speed=braking?std::max(0.,6-std::max(0.,elapsed-5)):0;
      double measured=speed<.055?noise:speed;
      int notch=braking&&elapsed>=5&&elapsed<11?-4:0;
      e.submitDriverPosition(notch,t);inspect(false);
      e.submitFrontWheel(measured/c.wheel_scale/c.front_scale,t);inspect(false);
      e.submitRearWheel(measured/c.wheel_scale/c.rear_scale,t);inspect(elapsed>duration-1);
    }
    double distance=e.state().distance_m,max_speed=0;
    for(int i=1;i<=10*hz;++i){e.submitDriverPosition(0,100+duration+double(i)/hz);inspect(false);max_speed=std::max(max_speed,e.state().velocity_mps);}
    double added=e.state().distance_m-distance,reset_residual=0,reset_cross=0;
    e.reset(300,0);
#if HAS_KALMAN
    reset_residual=e.kalman_residual_mps2_;reset_cross=e.kalman_va_;
#else
    min_det=0;
#endif
    std::cout<<vehicle<<','<<hz<<','<<braking<<','<<noise<<','<<min_det<<','<<nonfinite<<','<<negative<<','<<max_stop_residual<<','<<max_speed<<','<<added<<','<<reset_residual<<','<<reset_cross<<'\n';
  }
}
