#include "approach_policy.hpp"
#include <cassert>
using namespace raicom_nav2_workspace;
int main() {
  Limits l; l.validate();
  SettledArrival arrival;
  assert(!arrival.reached(.01,.1,.1,.4,1.));
  assert(!arrival.reached(.01,.1,0.,0.,2.));
  assert(!arrival.reached(.01,.1,0.,0.,2.49));
  assert(arrival.reached(.01,.1,0.,0.,2.51));
  assert(!arrival.reached(.039,.1,0.,0.,2.6)); // observed post-arrival localization correction
  assert(!arrival.reached(.01,.1,0.,0.,3.));
  assert(arrival.reached(.01,.1,0.,0.,3.51));
  assert(!arrival.reached(.01,.44,0.,0.,3.6)); // observed 25-degree post-stop error
  assert(!arrival.reached(.01,.1,0.,.03,4.));
  assert(!arrival.reached(.01,.1,0.,0.,4.1));
  assert(arrival.reached(.01,.1,0.,0.,4.61));
  arrival.reset();assert(!arrival.reached(.01,.1,0.,0.,5.));
  BrakingHold hold;
  assert(hold.wait(10.,1.2));
  assert(hold.wait(10.4,.2)); // can decelerate without the old 0.3-second abort
  assert(!hold.wait(11.51,5.)); // cannot extend an existing deadline
  hold.reset();assert(hold.wait(20.,10.));assert(!hold.wait(23.51,10.));
  bool aligning=true;
  assert(stable_phase(1.,0.,.2,l,aligning)==Phase::path_turn);
  assert(stable_phase(1.,0.,.07,l,aligning)==Phase::tracking);
  for (double error : {.14,.16,.14,.17,.16})
    assert(stable_phase(1.,0.,error,l,aligning)==Phase::tracking);
  assert(stable_phase(1.,0.,.46,l,aligning)==Phase::path_turn);
  assert(stable_phase(1.,0.,.12,l,aligning)==Phase::path_turn);
  assert(stable_phase(.1,0.,.07,l,aligning)==Phase::fine_position);
  assert(stable_phase(.01,pi/18,1.,l,aligning)==Phase::complete);
  assert(stable_phase(.03,0.,.07,l,aligning)==Phase::fine_position);
  assert(safe_scale(.25,0.,[](double v,double){return .2*v+v*v<.06;})==.5);
  assert(safe_scale(.25,.35,[](double,double){return false;})==0.);
  assert(safe_scale(0.,.35,[](double,double w){return w<=.2;})==.5);
  assert(safe_scale(.25,.1,[](double,double){return true;})==1.);
  assert(phase(.02,pi/18,0,l)==Phase::complete);
  assert(phase(.0201,0,0,l)==Phase::fine_position);
  assert(phase(.01,pi/18+.001,0,l)==Phase::final_yaw);
  // Drifting out of the window must restore translation; no rotation latch.
  assert(phase(.025,pi/2,0,l)==Phase::fine_position);
  assert(phase(1.,0,pi,l)==Phase::path_turn);
  assert(phase(1.,pi,0,l)==Phase::tracking);
  assert(phase(.1,pi,0,l)==Phase::fine_position);
  assert(turn_rate(pi/18,pi/18,l)==0.);
  assert(turn_rate(-pi/2,pi/18,l)<0.);
  l.far=2.; l.near=1.; l.validate();
  assert(l.speed(3.)<=.25);  // User request never silently raises the verified cap.
  assert(l.speed(.03)<=.03);
  assert(l.speed(.02)==0.);
  assert(l.speed(.020001)==l.fine);
  assert(turn_rate(pi/18+.000001,pi/18,l)>=.06);
  l.far=.35; l.near=.15; l.validated_cap=.35; l.validate();
  assert(l.speed(3.)==.35);
  assert(l.speed(.15)==.15);
  assert(l.speed(.03)==.03);
  l.far=2.; l.near=1.; l.validated_cap=2.; l.turn=2.; l.validate();
  assert(l.speed(5.)==2.);
  assert(l.speed(.25)<=1.);
  assert(l.speed(.03)<=.03);
  assert(l.braking_horizon(2.,0.)>4.);
  assert(l.braking_horizon(0.,2.)>2.8);
  assert(turn_rate(pi,pi/18,l)<=2.);
  l.turn=.05; l.validate();
  assert(turn_rate(pi,pi/18,l)<=.05);
  l.far=2.;l.near=1.;l.fine=.5;l.near_distance=.1;l.validated_cap=2.;l.validate();
  assert(l.speed(.03)==.5); // operator-selected fine speed is not clamped to .08
  l.far=.25;l.near=1.2;l.fine=.5;l.validated_cap=1.2;l.validate();
  assert(l.speed(.03)==.5); // independent stages, cap is not the cruise value
  l.near_distance=.03;l.validate();
  assert(l.speed(.04)==.25); // fine phase cannot extend beyond chosen approach distance
  l.near_distance=0.;l.validate();
  assert(l.speed(.03)==.25);
  l.near_distance=3.;l.fine=.005;l.turn=.01;l.validate();
  assert(l.speed(.03)==.005);
  l.validated_cap=2.01;
  bool rejected=false; try {l.validate();} catch(const std::invalid_argument&) {rejected=true;}
  assert(rejected);
}
