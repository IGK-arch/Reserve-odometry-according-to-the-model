#include "tram_odometry/estimator.hpp"
#include <iostream>
#include <iomanip>
using namespace tram_odometry;
int main(){std::cout<<std::setprecision(15);EstimatorConfig c;c.enable_adaptation=false;c.front_scale=c.rear_scale=1;Estimator e(c);e.reset(100,0);
for(int i=1;i<=100;++i){double t=100+i*.1;e.submitDriverPosition(0,t);e.submitFrontWheel(.04*3.6,t);e.submitRearWheel(.04*3.6,t);}
auto a=e.state();std::cout<<"before dropout speed="<<a.velocity_mps<<" distance="<<a.distance_m<<"\n";double d0=a.distance_m;
for(int i=1;i<=100;++i){double t=110+i*.1;e.submitDriverPosition(0,t);if(i==10||i==20||i==50||i==100){auto s=e.state();std::cout<<"blackout_s="<<i*.1<<" speed="<<s.velocity_mps<<" extra_distance="<<s.distance_m-d0<<" model_accel="<<s.model_acceleration_mps2<<"\n";}}
}
