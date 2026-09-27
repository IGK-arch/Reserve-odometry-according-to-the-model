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
int main(){std::cout<<std::setprecision(15);for(double sigma:{.075,.085,.15,.3,.5})for(double dt:{.02,.1,.2,.3}){EstimatorConfig c;c.enable_adaptation=false;c.front_scale=c.rear_scale=1;c.wheel_sigma_mps=sigma;Estimator e(c);e.reset(100,0);double mindet=1;int failures=0;for(int i=1;i<=500;++i){double t=100+i*dt;e.submitDriverPosition(0,t);e.submitFrontWheel(.04*3.6,t);e.submitRearWheel(.04*3.6,t);double det=e.velocity_variance_*e.kalman_aa_-e.kalman_va_*e.kalman_va_;mindet=std::min(mindet,det);if(det < -1e-12)++failures;}std::cout<<"sigma="<<sigma<<" dt="<<dt<<" negative="<<failures<<" min_det="<<mindet<<"\n";}}
