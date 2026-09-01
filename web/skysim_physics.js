/**
 * SkySim Pure JS Physics Core
 * High-fidelity 6-DOF drone dynamics with Blade Element Theory (BET),
 * ISA Atmosphere, Dryden Turbulence, Cheeseman-Bennett Ground Effect,
 * Leishman Vortex Ring State (VRS), Cascade PID Flight Controller,
 * and Quad-X Mixer.
 */

(function(global) {
  'use strict';

  const PI = Math.PI;
  const TWO_PI = 2.0 * Math.PI;
  const DEG2RAD = PI / 180.0;
  const RAD2DEG = 180.0 / PI;

  function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  function lerp(a, b, t) {
    return a + t * (b - a);
  }

  // ---------------------------------------------------------------------------
  // Vector3 Math (Double precision)
  // ---------------------------------------------------------------------------
  class Vec3 {
    constructor(x = 0, y = 0, z = 0) {
      this.x = Number(x); this.y = Number(y); this.z = Number(z);
    }
    set(x, y, z) { this.x = x; this.y = y; this.z = z; return this; }
    clone() { return new Vec3(this.x, this.y, this.z); }
    copy(v) { this.x = v.x; this.y = v.y; this.z = v.z; return this; }
    add(v) { return new Vec3(this.x + v.x, this.y + v.y, this.z + v.z); }
    sub(v) { return new Vec3(this.x - v.x, this.y - v.y, this.z - v.z); }
    mul(s) { return new Vec3(this.x * s, this.y * s, this.z * s); }
    div(s) { const r = 1 / s; return new Vec3(this.x * r, this.y * r, this.z * r); }
    addSelf(v) { this.x += v.x; this.y += v.y; this.z += v.z; return this; }
    subSelf(v) { this.x -= v.x; this.y -= v.y; this.z -= v.z; return this; }
    mulSelf(s) { this.x *= s; this.y *= s; this.z *= s; return this; }
    dot(v) { return this.x * v.x + this.y * v.y + this.z * v.z; }
    norm2() { return this.dot(this); }
    norm() { return Math.sqrt(this.norm2()); }
    normalized() {
      const n = this.norm();
      return n > 1e-12 ? this.div(n) : new Vec3();
    }
    cross(v) {
      return new Vec3(
        this.y * v.z - this.z * v.y,
        this.z * v.x - this.x * v.z,
        this.x * v.y - this.y * v.x
      );
    }
  }

  // ---------------------------------------------------------------------------
  // Quaternion Math (Hamilton convention, w-last storage)
  // ---------------------------------------------------------------------------
  class Quat {
    constructor(x = 0, y = 0, z = 0, w = 1) {
      this.x = Number(x); this.y = Number(y); this.z = Number(z); this.w = Number(w);
    }
    set(x, y, z, w) { this.x = x; this.y = y; this.z = z; this.w = w; return this; }
    clone() { return new Quat(this.x, this.y, this.z, this.w); }
    copy(q) { this.x = q.x; this.y = q.y; this.z = q.z; this.w = q.w; return this; }
    conjugate() { return new Quat(-this.x, -this.y, -this.z, this.w); }
    mul(q) {
      return new Quat(
        this.w * q.x + this.x * q.w + this.y * q.z - this.z * q.y,
        this.w * q.y - this.x * q.z + this.y * q.w + this.z * q.x,
        this.w * q.z + this.x * q.y - this.y * q.x + this.z * q.w,
        this.w * q.w - this.x * q.x - this.y * q.y - this.z * q.z
      );
    }
    rotate(v) {
      const qv = new Vec3(this.x, this.y, this.z);
      const t = qv.cross(v).mul(2.0);
      return v.add(t.mul(this.w)).add(qv.cross(t));
    }
    normalize() {
      const n = Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z + this.w*this.w);
      if (n > 1e-12) {
        this.x /= n; this.y /= n; this.z /= n; this.w /= n;
      } else {
        this.x = 0; this.y = 0; this.z = 0; this.w = 1;
      }
      return this;
    }
    toEulerRPY() {
      // Roll (x), Pitch (y), Yaw (z) in ZYX extrinsic convention
      const sinr_cosp = 2 * (this.w * this.x + this.y * this.z);
      const cosr_cosp = 1 - 2 * (this.x * this.x + this.y * this.y);
      const roll = Math.atan2(sinr_cosp, cosr_cosp);

      const sinp = 2 * (this.w * this.y - this.z * this.x);
      let pitch;
      if (Math.abs(sinp) >= 1) pitch = (sinp >= 0 ? PI/2 : -PI/2);
      else pitch = Math.asin(sinp);

      const siny_cosp = 2 * (this.w * this.z + this.x * this.y);
      const cosy_cosp = 1 - 2 * (this.y * this.y + this.z * this.z);
      const yaw = Math.atan2(siny_cosp, cosy_cosp);
      return new Vec3(roll, pitch, yaw);
    }
  }

  // ---------------------------------------------------------------------------
  // Frame Conversions (Godot Y-up / -Z fwd to NED / FRD aerospace)
  // Matching include/core/frames.hpp
  // ---------------------------------------------------------------------------
  const Frames = {
    godotToNedVec(v) {
      // ned.x = -godot.z (North), ned.y = godot.x (East), ned.z = -godot.y (Down)
      return new Vec3(-v.z, v.x, -v.y);
    },
    nedToGodotVec(v) {
      return new Vec3(v.y, -v.z, -v.x);
    },
    godotToNedQuat(q) {
      // Permutation from frames.hpp: {-q.z, q.x, -q.y, q.w}
      return new Quat(-q.z, q.x, -q.y, q.w);
    },
    nedToGodotQuat(q) {
      return new Quat(q.y, -q.z, -q.x, q.w);
    }
  };

  // ---------------------------------------------------------------------------
  // ISA Atmosphere Model & Dryden Turbulence
  // ---------------------------------------------------------------------------
  class Atmosphere {
    constructor() {
      this.seaLevelP = 101325.0;
      this.seaLevelT = 288.15;
      this.seaLevelRho = 1.225;
      this.lapseRate = 0.0065;
      this.Rspecific = 287.058;
      this.gamma = 1.4;
      this.g0 = 9.80665;
      this.offsetM = 0.0;
      this.windGlobal = new Vec3();
      this.turbIntensity = 0.0; // m/s
      this.turbScale = 200.0;
      this.turbState = new Vec3();
    }

    setWind(x, y, z) {
      this.windGlobal.set(x, y, z);
    }

    setTurbulence(intensity, scale = 200.0) {
      this.turbIntensity = intensity;
      this.turbScale = scale;
    }

    atAltitude(altM) {
      const h = Math.max(altM + this.offsetM, 0.0);
      const T = this.seaLevelT - this.lapseRate * h;
      const ratio = T / this.seaLevelT;
      const exp = this.g0 / (this.Rspecific * this.lapseRate);
      const pressure = this.seaLevelP * Math.pow(ratio, exp);
      const density = pressure / (this.Rspecific * T);
      const speedOfSound = Math.sqrt(this.gamma * this.Rspecific * T);
      return { temperature: T, pressure, density, speedOfSound };
    }

    sampleWind(altM, dt) {
      if (this.turbIntensity < 1e-6) return this.windGlobal.clone();
      const V = Math.max(this.windGlobal.norm(), 1.0);
      const tau = this.turbScale / V;
      const alpha = Math.min(dt / (tau + dt), 0.5);
      const randVec = new Vec3(
        (Math.random() * 2 - 1) * this.turbIntensity,
        (Math.random() * 2 - 1) * this.turbIntensity * 0.5,
        (Math.random() * 2 - 1) * this.turbIntensity
      );
      this.turbState.x += alpha * (randVec.x - this.turbState.x);
      this.turbState.y += alpha * (randVec.y - this.turbState.y);
      this.turbState.z += alpha * (randVec.z - this.turbState.z);
      return this.windGlobal.add(this.turbState);
    }
  }

  // ---------------------------------------------------------------------------
  // Blade Element Theory Rotor Solver
  // ---------------------------------------------------------------------------
  class BladeElementSolver {
    constructor(cfg) {
      this.cfg = Object.assign({
        radius: 0.127,
        chord: 0.016,
        nBlades: 2,
        twistRootDeg: 14.0,
        twistTipDeg: 6.0,
        hubRadius: 0.015,
        clAlpha: 5.73,
        cd0: 0.012,
        cd2: 0.080,
        motorKv: 920.0,
        motorResistance: 0.12,
        motorInertia: 1.5e-5,
        maxVoltage: 14.8,
        escTau: 0.015,
        position: new Vec3(),
        spinDir: 1 // +1 CCW, -1 CW
      }, cfg);

      this.N_ANNULI = 24;
      this.state = {
        omega: 0.0,
        omegaCmd: 0.0,
        thrust: 0.0,
        torqueReaction: 0.0,
        power: 0.0
      };
      this.prevVi = 0.0;
    }

    setThrottle(throttleNorm) {
      const thr = clamp(throttleNorm, 0.0, 1.0);
      const maxOmega = this.cfg.motorKv * this.cfg.maxVoltage * (PI / 30.0);
      // Square-root mapping matching C++ RotorArray: thrust proportional to omega^2
      this.state.omegaCmd = maxOmega * Math.sqrt(Math.max(thr, 0.0));
    }

    solve(bodyVelWorld, bodyOmegaBF, orient, density, windWorld, dt) {
      // 1. First-order ESC motor lag
      const decay = Math.exp(-dt / this.cfg.escTau);
      this.state.omega = this.state.omega * decay + this.state.omegaCmd * (1.0 - decay);
      const omega = this.state.omega;

      if (omega < 1.0) {
        this.state.thrust = 0.0;
        this.state.torqueReaction = 0.0;
        this.state.power = 0.0;
        return {
          thrustWorld: new Vec3(),
          torqueBody: new Vec3(),
          power: 0.0
        };
      }

      const rho = density;
      const R = this.cfg.radius;
      const hub = this.cfg.hubRadius;
      const dr = (R - hub) / this.N_ANNULI;
      const diskArea = PI * R * R;

      // Rotor hub velocity relative to air
      const hubPosBF = this.cfg.position;
      const hubVelW = bodyVelWorld.add(orient.rotate(bodyOmegaBF.cross(hubPosBF)));
      const rotorUpW = orient.rotate(new Vec3(0, 1, 0));
      const v_axial = -(hubVelW.sub(windWorld)).dot(rotorUpW);

      // Rankine-Froude momentum inflow iteration
      let vi = this.prevVi;
      for (let iter = 0; iter < 3; ++iter) {
        const vi_new = Math.max(this.state.thrust, 0.0) / (2.0 * rho * diskArea * Math.max(Math.abs(v_axial + vi), 0.5));
        vi = lerp(vi, vi_new, 0.5);
      }

      // Blade element integration across annuli
      let totalThrust = 0.0;
      let totalTorque = 0.0;

      for (let i = 0; i < this.N_ANNULI; ++i) {
        const r = hub + (i + 0.5) * dr;
        const Vt = omega * r;
        const Va = v_axial + vi;
        const V2 = Vt * Vt + Va * Va;
        if (V2 < 1e-6) continue;

        const t_frac = (r - hub) / (R - hub);
        const theta = lerp(this.cfg.twistRootDeg, this.cfg.twistTipDeg, t_frac) * DEG2RAD;
        const phi = Math.atan2(Va, Vt);
        const alpha = theta - phi;

        const cl = this.cfg.clAlpha * alpha;
        const cd = this.cfg.cd0 + this.cfg.cd2 * cl * cl;
        const q = 0.5 * rho * V2;
        const dL = q * this.cfg.chord * cl * dr * this.cfg.nBlades;
        const dD = q * this.cfg.chord * cd * dr * this.cfg.nBlades;

        const dT = dL * Math.cos(phi) - dD * Math.sin(phi);
        const dQ = (dL * Math.sin(phi) + dD * Math.cos(phi)) * r;

        totalThrust += dT;
        totalTorque += dQ;
      }

      this.prevVi = vi;
      this.state.thrust = Math.max(totalThrust, 0.0);
      this.state.torqueReaction = totalTorque;
      this.state.power = totalTorque * omega;

      // Project into world frame
      const thrustW = rotorUpW.mul(this.state.thrust);
      // Reaction torque about +Y body axis
      const torqueB = new Vec3(0, -this.cfg.spinDir * totalTorque, 0);
      // Gyroscopic precession
      const gyroFactor = this.cfg.motorInertia * omega * this.cfg.spinDir;
      const gyroB = bodyOmegaBF.cross(new Vec3(0, gyroFactor, 0));
      const torqueBody = torqueB.add(gyroB);

      return {
        thrustWorld: thrustW,
        torqueBody,
        power: this.state.power
      };
    }
  }

  // ---------------------------------------------------------------------------
  // Aero Effects: Ground Effect (Cheeseman-Bennett) & VRS (Leishman)
  // ---------------------------------------------------------------------------
  class AeroEffects {
    constructor(rotorRadius = 0.127) {
      this.R = rotorRadius;
      this.vrsSeverity = 0.0;
      this.vrsActive = false;
    }

    groundEffectMultiplier(h_agl) {
      const h_R = h_agl / this.R;
      if (h_R > 3.0) return 1.0;
      const h = Math.max(h_agl, this.R * 0.25);
      const ratio = this.R / (4.0 * h);
      const denom = 1.0 - ratio * ratio;
      const raw = clamp(1.0 / Math.max(denom, 0.4), 1.0, 2.2);
      const blend = clamp(1.0 - (h_R - 0.25) / 2.75, 0.0, 1.0);
      return 1.0 + (raw - 1.0) * blend;
    }

    evaluateVRS(hoverInducedVel, descentRate, lateralSpeed, dt) {
      const Vc = Math.max(hoverInducedVel, 0.1);
      const mu_d = descentRate / Vc;
      const mu_l = lateralSpeed / Vc;

      let onset = 0.0;
      if (mu_d > 0.3 && mu_d < 1.5 && mu_l < 0.5) {
        const d_factor = 1.0 - Math.abs(mu_d - 0.9) / 0.6;
        const l_factor = 1.0 - mu_l / 0.5;
        onset = clamp(d_factor * l_factor, 0.0, 1.0);
      }

      const tau = (onset > this.vrsSeverity) ? 0.5 : 1.2;
      this.vrsSeverity += (onset - this.vrsSeverity) * Math.min(dt / (tau + dt), 1.0);
      this.vrsActive = this.vrsSeverity > 0.05;

      const thrustLoss = 0.30 * this.vrsSeverity;
      const buffeting = this.vrsActive ? (Math.sin(Date.now() * 0.015) * 0.08 * this.vrsSeverity) : 0;
      const thrustFactor = Math.max(1.0 - thrustLoss + buffeting, 0.5);

      return {
        active: this.vrsActive,
        severity: this.vrsSeverity,
        thrustFactor
      };
    }
  }

  // ---------------------------------------------------------------------------
  // Cascade PID Flight Controller & Quad-X Mixer
  // ---------------------------------------------------------------------------
  class PID {
    constructor(kp = 0, ki = 0, kd = 0, limit = 50.0) {
      this.kp = kp; this.ki = ki; this.kd = kd;
      this.limit = limit;
      this.integral = 0.0;
      this.prevErr = 0.0;
      this.first = true;
    }
    reset() {
      this.integral = 0.0; this.prevErr = 0.0; this.first = true;
    }
    update(err, dt) {
      this.integral += err * dt;
      this.integral = clamp(this.integral, -this.limit, this.limit);
      let deriv = 0.0;
      if (!this.first) deriv = (err - this.prevErr) / Math.max(dt, 1e-6);
      this.prevErr = err;
      this.first = false;
      return this.kp * err + this.ki * this.integral + this.kd * deriv;
    }
  }

  class FlightController {
    constructor() {
      // Outer attitude loop gains
      this.attRollP = 8.0;
      this.attPitchP = 8.0;
      this.maxRollRate = 8.0;
      this.maxPitchRate = 8.0;
      this.maxYawRate = 3.14;

      // Inner angular rate loop PIDs
      this.rateRollPID = new PID(0.15, 0.05, 0.003);
      this.ratePitchPID = new PID(0.15, 0.05, 0.003);
      this.rateYawPID = new PID(0.20, 0.10, 0.0);

      // Altitude hold PID
      this.altPID = new PID(0.35, 0.10, 0.20, 10.0);
      this.targetAlt = 3.0;
    }

    reset() {
      this.rateRollPID.reset();
      this.ratePitchPID.reset();
      this.rateYawPID.reset();
      this.altPID.reset();
    }

    update(setpoints, orientationFRD, angularVelFRD, dt) {
      const rpy = orientationFRD.toEulerRPY();

      // Outer attitude loop: attitude error -> rate setpoint
      let rollRateSP = this.attRollP * (setpoints.rollRad - rpy.x);
      let pitchRateSP = this.attPitchP * (setpoints.pitchRad - rpy.y);
      let yawRateSP = setpoints.yawRateRads;

      rollRateSP = clamp(rollRateSP, -this.maxRollRate, this.maxRollRate);
      pitchRateSP = clamp(pitchRateSP, -this.maxPitchRate, this.maxPitchRate);
      yawRateSP = clamp(yawRateSP, -this.maxYawRate, this.maxYawRate);

      // Inner rate loop: rate error -> torque demands
      const rollErr = rollRateSP - angularVelFRD.x;
      const pitchErr = pitchRateSP - angularVelFRD.y;
      const yawErr = yawRateSP - angularVelFRD.z;

      const tauRoll = this.rateRollPID.update(rollErr, dt);
      const tauPitch = this.ratePitchPID.update(pitchErr, dt);
      const tauYaw = this.rateYawPID.update(yawErr, dt);

      return [tauRoll, tauPitch, tauYaw, setpoints.thrustNorm];
    }
  }

  class MixerMatrix {
    static quadX() {
      // ArduPilot / PX4 Quad-X Layout matching C++ MixerMatrix::quad_x()
      // Demands are FRD frame: +roll right-down, +pitch nose-up, +yaw nose-right
      return [
        [ 1.0, -1.0,  1.0,  1.0 ], // M1 Front-Right CCW
        [ 1.0,  1.0, -1.0,  1.0 ], // M2 Back-Left   CCW
        [ 1.0,  1.0,  1.0, -1.0 ], // M3 Front-Left  CW
        [ 1.0, -1.0, -1.0, -1.0 ], // M4 Back-Right  CW
      ];
    }

    static mix(thrustNorm, rollTorque, pitchTorque, yawTorque) {
      const rows = MixerMatrix.quadX();
      const out = new Array(4);
      let maxVal = 0.0;
      for (let i = 0; i < 4; ++i) {
        out[i] = rows[i][0] * thrustNorm +
                 rows[i][1] * rollTorque +
                 rows[i][2] * pitchTorque +
                 rows[i][3] * yawTorque;
        maxVal = Math.max(maxVal, Math.abs(out[i]));
      }
      if (maxVal > 1.0) {
        for (let i = 0; i < 4; ++i) out[i] /= maxVal;
      }
      for (let i = 0; i < 4; ++i) out[i] = clamp(out[i], 0.0, 1.0);
      return out;
    }
  }

  // ---------------------------------------------------------------------------
  // SkySim Core (6-DOF Integrator & Full Drone Model)
  // ---------------------------------------------------------------------------
  class SkySimCore {
    constructor() {
      this.mass = 1.5; // kg
      this.rotorRadius = 0.127; // m
      this.motorKv = 920.0;
      this.maxVoltage = 14.8; // 4S LiPo
      this.batteryCapacityMah = 3000;
      this.batteryMahConsumed = 0;
      this.batteryVoltage = 16.8;

      this.groundRadius = 0.15; // m (landing gear clearance)
      this.groundHeight = 0.0;
      this.inertia = new Vec3(0.02, 0.035, 0.02); // Godot body axes

      this.position = new Vec3(0, 0.15, 0);
      this.velocity = new Vec3();
      this.orientation = new Quat(0, 0, 0, 1);
      this.angularVelocity = new Vec3();

      this.armed = false;
      this.directMotors = false;
      this.directThrottles = [0, 0, 0, 0];
      this.flightMode = 'ALT_HOLD'; // Default to stable Alt Hold

      this.setpoints = {
        rollRad: 0.0,
        pitchRad: 0.0,
        yawRateRads: 0.0,
        thrustNorm: 0.52
      };

      this.time = 0.0;
      this.atm = new Atmosphere();
      this.effects = new AeroEffects(this.rotorRadius);
      this.fc = new FlightController();

      // Quad-X Rotors Layout matching C++ DroneBody
      const arm = this.rotorRadius * 2.1;
      const layout = [
        { x:  arm, z: -arm, dir: +1 }, // M1 FR CCW
        { x: -arm, z:  arm, dir: +1 }, // M2 BL CCW
        { x: -arm, z: -arm, dir: -1 }, // M3 FL CW
        { x:  arm, z:  arm, dir: -1 }, // M4 BR CW
      ];

      this.rotors = layout.map(cfg => new BladeElementSolver({
        radius: this.rotorRadius,
        motorKv: this.motorKv,
        maxVoltage: this.maxVoltage,
        position: new Vec3(cfg.x, 0, cfg.z),
        spinDir: cfg.dir
      }));

      this.rotorThrottles = [0, 0, 0, 0];
      this.rotorRPMs = [0, 0, 0, 0];

      // Telemetry
      this.telemetry = {
        altitude: 0,
        verticalSpeed: 0,
        groundSpeed: 0,
        airspeed: 0,
        rollDeg: 0,
        pitchDeg: 0,
        yawDeg: 0,
        rollRate: 0,
        pitchRate: 0,
        yawRate: 0,
        totalThrust: 0,
        twr: 0,
        powerDraw: 0,
        currentDraw: 0,
        batteryVoltage: 16.8,
        batteryPercent: 100,
        batteryMahConsumed: 0,
        groundEffectFactor: 1.0,
        vrsActive: false,
        vrsSeverity: 0,
        airDensity: 1.225,
        windSpeed: 0,
        windDirectionDeg: 0,
        rpms: [0, 0, 0, 0],
        armed: false,
        flightMode: 'ALT_HOLD'
      };

      this.reset(0, 0.15, 0);
    }

    reset(x = 0, y = 0.15, z = 0) {
      this.position.set(x, y, z);
      this.velocity.set(0, 0, 0);
      this.orientation.set(0, 0, 0, 1);
      this.angularVelocity.set(0, 0, 0);
      this.armed = false;
      this.directMotors = false;
      this.setpoints = { rollRad: 0, pitchRad: 0, yawRateRads: 0, thrustNorm: 0.52 };
      this.time = 0.0;
      this.fc.reset();
      this.fc.targetAlt = 3.0;
      for (const r of this.rotors) {
        r.state.omega = 0;
        r.state.omegaCmd = 0;
        r.state.thrust = 0;
      }
      this.rotorThrottles = [0, 0, 0, 0];
      this.rotorRPMs = [0, 0, 0, 0];
      this.batteryMahConsumed = 0;
      this.updateTelemetry(new Vec3(), 0, 1.0, { active: false, severity: 0 }, { density: 1.225 }, new Vec3());
    }

    arm() {
      this.armed = true;
      this.directMotors = false;
      this.fc.reset();
    }

    disarm() {
      this.armed = false;
    }

    setFlightMode(mode) {
      this.flightMode = mode;
    }

    setAttitudeSetpoint(roll, pitch, yawRate, throttle) {
      this.setpoints.rollRad = clamp(roll, -0.785, 0.785); // +-45 deg
      this.setpoints.pitchRad = clamp(pitch, -0.785, 0.785);
      this.setpoints.yawRateRads = clamp(yawRate, -3.14, 3.14);
      this.setpoints.thrustNorm = clamp(throttle, 0.0, 1.0);
      this.directMotors = false;
    }

    setDirectMotors(throttles) {
      this.directThrottles = throttles.slice(0, 4);
      this.directMotors = true;
    }

    setWind(x, y, z) {
      this.atm.setWind(x, y, z);
    }

    setTurbulence(intensity) {
      this.atm.setTurbulence(intensity);
    }

    // Step physics with 4 sub-steps for extreme numerical stability at 400 Hz
    step(dt) {
      if (dt <= 0) return;
      const subSteps = 4;
      const subDt = dt / subSteps;

      for (let s = 0; s < subSteps; ++s) {
        this.stepInternal(subDt);
      }
    }

    stepInternal(dt) {
      this.time += dt;
      const alt = this.position.y;
      const windW = this.atm.sampleWind(alt, dt);
      const atmState = this.atm.atAltitude(alt);

      // 1. Flight control & Mixer in FRD Aerospace Frame
      let throttles = [0, 0, 0, 0];
      if (this.directMotors) {
        throttles = this.directThrottles;
      } else if (this.armed) {
        // Convert orientation & angular velocity to FRD frame
        const q_frd = Frames.godotToNedQuat(this.orientation);
        const w_frd = Frames.godotToNedVec(this.angularVelocity);

        let sp = Object.assign({}, this.setpoints);

        // Auto Altitude Hold mode adjustment
        if (this.flightMode === 'ALT_HOLD' || this.flightMode === 'WAYPOINT') {
          const altErr = this.fc.targetAlt - alt;
          const altHoverThr = 0.52;
          const altCorrection = this.fc.altPID.update(altErr, dt);
          sp.thrustNorm = clamp(altHoverThr + altCorrection, 0.1, 0.95);
        }

        const [tauR, tauP, tauY, thr] = this.fc.update(sp, q_frd, w_frd, dt);
        const torqueScale = 0.3;
        throttles = MixerMatrix.mix(thr, tauR * torqueScale, tauP * torqueScale, tauY * torqueScale);
      }

      this.rotorThrottles = throttles;

      // 2. Solve Blade Element Rotors
      let totalForceBody = new Vec3();
      let totalTorqueBody = new Vec3();
      let totalPower = 0.0;
      let primaryHoverInducedVel = 0.0;

      for (let i = 0; i < 4; ++i) {
        const rotor = this.rotors[i];
        rotor.setThrottle(this.armed ? throttles[i] : 0.0);
        const res = rotor.solve(
          this.velocity,
          this.angularVelocity,
          this.orientation,
          atmState.density,
          windW,
          dt
        );
        // Force in body frame
        const f_body = this.orientation.conjugate().rotate(res.thrustWorld);
        const arm_b = rotor.cfg.position;
        const tau = arm_b.cross(f_body).add(res.torqueBody);

        totalForceBody.addSelf(f_body);
        totalTorqueBody.addSelf(tau);
        totalPower += res.power;

        this.rotorRPMs[i] = (rotor.state.omega * 30.0) / PI;
        if (i === 0) {
          const diskA = PI * this.rotorRadius * this.rotorRadius;
          primaryHoverInducedVel = Math.sqrt(Math.max(rotor.state.thrust, 0.0) / (2.0 * atmState.density * diskA));
        }
      }

      // 3. Ground Effect
      const h_agl = Math.max(alt - this.groundHeight, 0.0);
      const ge = this.effects.groundEffectMultiplier(h_agl);
      totalForceBody.y *= ge;

      // 4. Vortex Ring State
      const descentRate = -this.velocity.y;
      const lateralSpeed = Math.sqrt(this.velocity.x * this.velocity.x + this.velocity.z * this.velocity.z);
      const vrs = this.effects.evaluateVRS(primaryHoverInducedVel, descentRate, lateralSpeed, dt);
      if (vrs.active) {
        totalForceBody.y *= vrs.thrustFactor;
      }

      // 5. Assemble World Forces: Thrust + Gravity + Aerodynamic Drag
      const forceWorld = this.orientation.rotate(totalForceBody);
      const g = 9.80665;
      forceWorld.y -= g * this.mass;

      const dragLin = 0.03;
      const dragQuad = 0.18;
      const velRel = this.velocity.sub(windW);
      const vRelNorm = velRel.norm();
      if (vRelNorm > 1e-4) {
        const dragMag = (dragLin + dragQuad * atmState.density * vRelNorm);
        forceWorld.subSelf(velRel.mul(dragMag));
      }

      // 6. Linear Integration (Semi-Implicit Euler)
      const accel = forceWorld.div(this.mass);
      this.velocity.addSelf(accel.mul(dt));
      this.position.addSelf(this.velocity.mul(dt));

      // 7. Angular Integration (Body Frame Rigid-Body Dynamics)
      const tau = totalTorqueBody;
      const w = this.angularVelocity;
      const Iw = new Vec3(this.inertia.x * w.x, this.inertia.y * w.y, this.inertia.z * w.z);
      const gyro = w.cross(Iw);
      const wdot = new Vec3(
        (tau.x - gyro.x) / this.inertia.x,
        (tau.y - gyro.y) / this.inertia.y,
        (tau.z - gyro.z) / this.inertia.z
      );
      this.angularVelocity.addSelf(wdot.mul(dt));

      // Quaternion Kinematics: q_dot = 0.5 * q * (w, 0)
      const wq = new Quat(w.x, w.y, w.z, 0.0);
      const qd = this.orientation.mul(wq);
      this.orientation.x += 0.5 * qd.x * dt;
      this.orientation.y += 0.5 * qd.y * dt;
      this.orientation.z += 0.5 * qd.z * dt;
      this.orientation.w += 0.5 * qd.w * dt;
      this.orientation.normalize();

      // 8. Ground Contact Collision & Landing Support
      if (this.position.y < this.groundRadius) {
        this.position.y = this.groundRadius;
        if (this.velocity.y < 0.0) this.velocity.y = 0.0;
        this.velocity.x *= 0.85;
        this.velocity.z *= 0.85;
        this.angularVelocity.mulSelf(0.70);

        // Ground uprighting damping when disarmed or idling on pad
        if (!this.armed || this.setpoints.thrustNorm < 0.2) {
          this.orientation.x *= 0.90;
          this.orientation.z *= 0.90;
          this.orientation.normalize();
        }
      }

      // 9. Battery Telemetry Integration
      const currentAmps = totalPower / Math.max(this.batteryVoltage, 10.0);
      const mahPerSecond = (currentAmps * 1000.0) / 3600.0;
      this.batteryMahConsumed += mahPerSecond * dt;
      this.batteryPercent = Math.max(0, 100 * (1.0 - this.batteryMahConsumed / this.batteryCapacityMah));
      this.batteryVoltage = Math.max(13.2, 16.8 - (3.6 * (this.batteryMahConsumed / this.batteryCapacityMah)) - (currentAmps * 0.04));

      // 10. Telemetry Update
      this.updateTelemetry(forceWorld, totalPower, ge, vrs, atmState, windW);
    }

    updateTelemetry(forceWorld, totalPower, ge, vrs, atmState, windW) {
      const q_frd = Frames.godotToNedQuat(this.orientation);
      const w_frd = Frames.godotToNedVec(this.angularVelocity);
      const rpy = q_frd.toEulerRPY();

      const alt = Math.max(0, this.position.y - this.groundRadius);
      const vs = this.velocity.y;
      const gs = Math.sqrt(this.velocity.x * this.velocity.x + this.velocity.z * this.velocity.z);
      const totalThrustN = Math.max(0, forceWorld.y + this.mass * 9.80665);
      const twr = totalThrustN / (this.mass * 9.80665);
      const currentAmps = totalPower / Math.max(this.batteryVoltage, 10.0);

      this.telemetry.altitude = alt;
      this.telemetry.verticalSpeed = vs;
      this.telemetry.groundSpeed = gs;
      this.telemetry.airspeed = (windW ? this.velocity.sub(windW).norm() : gs);
      this.telemetry.rollDeg = rpy.x * RAD2DEG;
      this.telemetry.pitchDeg = rpy.y * RAD2DEG;
      this.telemetry.yawDeg = ((rpy.z * RAD2DEG) % 360 + 360) % 360;
      this.telemetry.rollRate = w_frd.x * RAD2DEG;
      this.telemetry.pitchRate = w_frd.y * RAD2DEG;
      this.telemetry.yawRate = w_frd.z * RAD2DEG;
      this.telemetry.totalThrust = totalThrustN;
      this.telemetry.twr = twr;
      this.telemetry.powerDraw = totalPower;
      this.telemetry.currentDraw = currentAmps;
      this.telemetry.batteryVoltage = this.batteryVoltage;
      this.telemetry.batteryPercent = Math.max(0, 100 * (1.0 - this.batteryMahConsumed / this.batteryCapacityMah));
      this.telemetry.batteryMahConsumed = this.batteryMahConsumed;
      this.telemetry.groundEffectFactor = ge;
      this.telemetry.vrsActive = vrs.active;
      this.telemetry.vrsSeverity = vrs.severity;
      this.telemetry.airDensity = atmState.density;
      this.telemetry.rpms = this.rotorRPMs.slice();
      this.telemetry.armed = this.armed;
      this.telemetry.flightMode = this.flightMode;
    }

    getObservation() {
      const q_frd = Frames.godotToNedQuat(this.orientation);
      const rpy = q_frd.toEulerRPY();
      const w_frd = Frames.godotToNedVec(this.angularVelocity);
      return {
        pos: [this.position.x, this.position.y, this.position.z],
        vel: [this.velocity.x, this.velocity.y, this.velocity.z],
        euler: [rpy.x, rpy.y, rpy.z],
        rates: [w_frd.x, w_frd.y, w_frd.z],
        alt: Math.max(0, this.position.y - this.groundRadius),
        vs: this.velocity.y,
        telemetry: Object.assign({}, this.telemetry)
      };
    }
  }

  // Export to global scope
  global.SkySim = {
    Core: SkySimCore,
    Vec3,
    Quat,
    Frames,
    Atmosphere,
    BladeElementSolver,
    FlightController,
    MixerMatrix
  };

})(typeof window !== 'undefined' ? window : this);
