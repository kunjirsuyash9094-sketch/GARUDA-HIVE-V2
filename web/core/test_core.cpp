#include "drone_core.hpp"
#include <cstdio>
#include <cmath>
using namespace dronesim;

static double run(double throttle, bool armed, double secs) {
    DroneCore d; d.reset(0, 2, 0);
    if (armed) { d.arm(); }
    const double dt = 1.0/400.0;
    int n = (int)(secs/dt);
    for (int i=0;i<n;i++){ if(armed) d.set_attitude_setpoint(0,0,0,throttle); d.step(dt); }
    return d.state().position.y;
}

int main(){
    printf("== SkySim standalone core (decoupled from Godot) ==\n");
    // 1. motors off -> falls to ground
    double y_off = run(0.0, false, 3.0);
    printf("disarmed 3s: alt=%.3f m  (expect ~ground 0.15)\n", y_off);

    // 2. throttle sweep after arming -> monotonic altitude response
    double lo = run(0.30, true, 4.0);
    double mid= run(0.55, true, 4.0);
    double hi = run(0.90, true, 4.0);
    printf("armed 4s: thr0.30 -> %.2f m | thr0.55 -> %.2f m | thr0.90 -> %.2f m\n", lo, mid, hi);

    // 3. trajectory sample at hover-ish throttle
    DroneCore d; d.reset(0,2,0); d.arm();
    const double dt=1.0/400.0;
    printf("hover-ish (thr 0.6) altitude trace:\n");
    for (int i=0;i<=2000;i++){
        d.set_attitude_setpoint(0,0,0,0.6); d.step(dt);
        if (i%400==0) printf("  t=%.1fs alt=%.2f vs=%+.2f thrust=%.1fN\n",
                             i*dt, d.state().position.y, d.telem().vertical_speed, d.telem().total_thrust);
    }
    // sanity
    bool nan = std::isnan(d.state().position.y) || std::isnan(d.telem().total_thrust);
    bool monotone = (lo <= mid+0.01) && (mid <= hi+0.01);
    printf("\nchecks: no_nan=%d  throttle_monotone=%d  landed_when_off=%d\n",
           !nan, monotone, (y_off < 0.5));
    printf("%s\n", (!nan && monotone && y_off<0.5) ? "CORE OK" : "CORE NEEDS REVIEW");
    return 0;
}
