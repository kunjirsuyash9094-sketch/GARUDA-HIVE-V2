#include "drone_core.hpp"
#ifdef __EMSCRIPTEN__
  #include <emscripten/emscripten.h>
  #define KA EMSCRIPTEN_KEEPALIVE
#else
  #define KA
#endif
using namespace dronesim;

extern "C" {
KA DroneCore* garuda_create()                         { return new DroneCore(); }
KA void garuda_destroy(DroneCore* d)                  { delete d; }
KA void garuda_reset(DroneCore* d,double x,double y,double z){ d->reset(x,y,z); }
KA void garuda_arm(DroneCore* d,int on)               { if(on) d->arm(); else d->disarm(); }
KA void garuda_set_attitude(DroneCore* d,double r,double p,double yr,double thr){ d->set_attitude_setpoint(r,p,yr,thr); }
KA void garuda_set_motors(DroneCore* d,double* t,int n){ d->set_motors(t,n); }
KA void garuda_set_wind(DroneCore* d,double x,double y,double z){ d->set_wind(x,y,z); }
KA void garuda_step(DroneCore* d,double dt)           { d->step(dt); }
KA void garuda_get_obs(DroneCore* d,double* out12)    { d->get_obs(out12); }
KA double garuda_altitude(DroneCore* d){ return d->telem().altitude; }
KA double garuda_vspeed(DroneCore* d)  { return d->telem().vertical_speed; }
KA double garuda_thrust(DroneCore* d)  { return d->telem().total_thrust; }
KA double garuda_roll(DroneCore* d)    { return d->telem().roll_deg; }
KA double garuda_pitch(DroneCore* d)   { return d->telem().pitch_deg; }
KA double garuda_yaw(DroneCore* d)     { return d->telem().yaw_deg; }
}
