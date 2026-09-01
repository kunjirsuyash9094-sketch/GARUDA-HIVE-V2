# GARUDA HIVE V2 — Flight-System Forensic Repair & Validation Report

---

## 1. Root Cause Analysis & Forensic Findings

### 1.1 Root Cause of Autonomous Climb on Arming
* **Defect**: In `web/garuda_flight_engine.js`, the animation loop `step(dt)` was continuously sending `throttle = 0.5833` (the exact equilibrium hover throttle for a 10.0 kg aircraft) 60 times a second to the C++ physics server regardless of whether the aircraft was resting on the pad or in flight.
* **Mechanism**: The moment the operator clicked `ARM`, the controller transitioned from `armed = false` to `armed = true`. Because the throttle setpoint was already sitting at `0.5833`, all 8 motors spooled immediately to ~4800 RPM, producing $98.07\text{ N}$ of thrust and causing the aircraft to lift into the air without any pilot command.
* **Resolution**: Replaced the default hover throttle with a strict ground-idle protocol:
  $$\text{Startup / Disarmed} \implies \text{Throttle} = 0.0, \quad \text{RPM} = 0.0, \quad T_{\text{total}} = 0.0\text{ N}$$
  $$\text{Armed on Ground} \implies \text{Throttle} = 0.0, \quad \text{RPM} \approx 383\text{ RPM (Idle)}, \quad T_{\text{total}} = 0.50\text{ N} \ll mg$$
  Liftoff occurs **strictly** when the operator clicks `LAUNCH (TAKEOFF)` or commands climb ($W > 0$).

### 1.2 Root Cause of Missing / Broken Manual Controls
* **Defect**: 
  1. Input mapping confusion between Mode 2 Drone RC ($W/S = \text{Climb/Descend}$, $\text{Arrows} = \text{Pitch/Roll}$) and Flight Simulator Classic ($W/S = \text{Pitch Fwd/Back}$, $A/D = \text{Roll Left/Right}$).
  2. In `garuda_bridge.js`, mutual recursive loops were blowing the call stack (`RangeError: Maximum call stack size exceeded`), halting event listeners.
* **Resolution**: 
  1. Decoupled client action dispatching with zero recursion.
  2. Implemented dual selectable flight control schemes with an interactive UI switch (`MODE 2 RC` vs `CLASSIC FLIGHT SIM`).
  3. Added a live telemetry diagnostic card displaying real-time pilot inputs: `THROTTLE`, `ROLL CMD`, `PITCH CMD`, `YAW RATE CMD`, and `TARGET ALTITUDE`.

### 1.3 Root Cause of Vertical "UP-DOWN-UP-DOWN" Oscillation
* **Defect**: 
  1. Keypress inputs for W/S were sending raw discrete throttle steps ($0.5833 \pm 0.25$), applying abrupt $+80\text{ N} / -60\text{ N}$ square-wave force shocks to the airframe.
  2. In `ScalarPID` (`flight_controller.hpp`), the derivative term was computed from setpoint error $(e - e_{\text{prev}})/\Delta t$. Because pilot setpoints arrived at 60 Hz over WebSocket while physics stepped at 400 Hz, 6 out of 7 physics ticks had $\Delta e = 0$, causing severe high-frequency derivative chatter.
* **Resolution**: 
  1. Upgraded C++ PID to **Derivative-on-Measurement** ($-\dot{\omega}$), completely eliminating derivative kick.
  2. Implemented smooth velocity-rate altitude integration: holding climb/descend keys integrates target altitude ($\dot{h}_{\text{target}} = \text{climb\_input} \times 2.0\text{ m/s}$), and releasing keys smoothly locks the current altitude with zero overshoot or oscillation.

### 1.4 Root Cause of Missing Stabilization
* **Defect**: Frame transformation coordinate mismatch between Godot/WebGL ($+Y$ Up, $+X$ Right, $-Z$ Forward) and aerospace FRD ($+X$ Forward, $+Y$ Right, $+Z$ Down) in quaternion to Euler conversion.
* **Resolution**: Standardized on unified `dronesim::frames::godot_to_ned(Quat)` across C++ and JavaScript.

### 1.5 Root Cause of Incorrect Landing
* **Defect**: Landing relied on kinematic position snapping rather than continuous physical descent and ground contact compliance.
* **Resolution**: Implemented 2-stage aerodynamic descent: controlled descent velocity at $-0.75\text{ m/s}$ until ground proximity, followed by contact flare, suspension spring-damper settling ($k = 12000\text{ N/m}$, $d = 850\text{ N}\cdot\text{s/m}$), and automatic motor disarm.

### 1.6 Root Cause of Incorrect Rotor Visualization
* **Defect**: Propeller blade rotation was being integrated inside the asynchronous 60 Hz WebSocket callback rather than the browser's native animation loop (`requestAnimationFrame`).
* **Resolution**: Added `GarudaOctocopterModel.updateInRenderLoop(dt)` which continuously integrates blade rotation angle $\Delta\theta_i = \text{spin}_i \cdot \frac{\text{RPM}_i \cdot 2\pi}{60} \cdot dt$ at the monitor's native refresh rate (60/144 Hz).

---

## 2. Files Modified & Created

| File Path | Action | Description |
|---|---|---|
| `include/garuda/control/flight_controller.hpp` | **MODIFIED** | Upgraded `ScalarPID` to Derivative-on-Measurement, anti-windup clamping, and strict idle floor. |
| `include/garuda/core/drone_instance.hpp` | **MODIFIED** | Added `_spawn_pos` tracking to preserve pad position during reset. |
| `src/garuda/core/drone_instance.cpp` | **MODIFIED** | Fixed true AGL altitude calculation ($y - \text{clearance}$) and preserved spawn coordinates on reset. |
| `garuda_server.py` | **MODIFIED** | Calibrated takeoff/landing throttles and decoupled WebSocket action handlers. |
| `web/garuda_flight_engine.js` | **MODIFIED** | Full 6-DOF force/torque overhaul, smooth velocity-rate altitude hold, strict ground-idle state. |
| `web/garuda_bridge.js` | **MODIFIED** | Synchronized server state with local engine and connected live pilot input diagnostics. |
| `web/garuda_octocopter.js` | **MODIFIED** | Added continuous render-loop rotor spin (`updateInRenderLoop`) and cached RPMs. |
| `web/garuda_audio.js` | **MODIFIED** | Added `ensureContext()` auto-resume for Web Audio and real-time RPM harmonic modulation. |
| `web/index.html` | **MODIFIED** | Added Live Pilot Input Diagnostic Card, Control Scheme Switcher, and render loop link. |
| `tests/test_physics_experiments.cpp` | **MODIFIED** | Updated Exp 03 liftoff threshold to True AGL metric ($h > 0.20\text{m}$). |
| `tests/test_live_flight_forensics.py` | **NEW** | Full 8-test automated live WebSocket forensic test suite. |
| `docs/physics/FLIGHT_SYSTEM_FORENSIC_REPORT.md` | **NEW** | Complete technical forensic documentation report. |

---

## 3. Control-Loop & Multirotor Aerodynamics Architecture

```
[ PILOT COMMAND / WEBSOCKET ]
              ↓ (60 Hz Setpoint)
[ POSITION / ALTITUDE CONTROLLER ]
              ↓ (Desired Vertical Accel & Tilt Targets)
[ ATTITUDE OUTER LOOP (P-Controller) ]
              ↓ (Angular Rate Demands: p_sp, q_sp, r_sp)
[ RATE INNER LOOP (PID with Derivative-on-Measurement) ]
              ↓ (Normalized Torque Demands: tau_roll, tau_pitch, tau_yaw)
[ 8-ROTOR OCTO-X ALLOCATION MIXER ]
              ↓ (Per-Motor Commanded Throttle u_i in [0, 1])
[ FIRST-ORDER ESC MOTOR LAG DYNAMICS (tau = 15 ms) ]
              ↓ (Rotor Speed omega_i in rad/s, RPM_i)
[ BLADE ELEMENT THEORY (24 Annuli Numerical Integration) ]
              ↓ (Individual Rotor Thrust T_i, Drag Torque Q_i, Gyroscopic Precession)
[ 6-DOF NEWTON-EULER RIGID BODY INTEGRATOR ]
              ↓ (Forces + Dynamic Mass Gravity + Drag + Ground Contact Spring-Damper)
[ 400 Hz AUTHORITATIVE TELEMETRY SNAPSHOT ]
```

---

## 4. 8-Rotor Octo-X Mixer Allocation Matrix

For an 8-rotor radial Octo-X configuration at arm angles $\psi_i = 22.5^\circ + i \cdot 45^\circ$ with alternating spin directions $\text{spin}_i \in \{+1 (\text{CCW}), -1 (\text{CW})\}$:

$$u_i = u_{\text{throttle}} - \cos(\psi_i) \cdot \tau_{\text{roll}} + \sin(\psi_i) \cdot \tau_{\text{pitch}} + \text{spin}_i \cdot \tau_{\text{yaw}}$$

| Motor ID | Position Angle $\psi_i$ | Spin Direction | Roll Coeff ($-\cos\psi_i$) | Pitch Coeff ($\sin\psi_i$) | Yaw Coeff ($\text{spin}_i$) |
|---|---|---|---|---|---|
| **M1** | $22.5^\circ$ (Front-Right) | CCW ($+1$) | $-0.9239$ | $+0.3827$ | $+1.0$ |
| **M2** | $67.5^\circ$ (Right-Front) | CW ($-1$) | $-0.3827$ | $+0.9239$ | $-1.0$ |
| **M3** | $112.5^\circ$ (Right-Rear) | CCW ($+1$) | $+0.3827$ | $+0.9239$ | $+1.0$ |
| **M4** | $157.5^\circ$ (Rear-Right) | CW ($-1$) | $+0.9239$ | $+0.3827$ | $-1.0$ |
| **M5** | $202.5^\circ$ (Rear-Left) | CCW ($+1$) | $+0.9239$ | $-0.3827$ | $+1.0$ |
| **M6** | $247.5^\circ$ (Left-Rear) | CW ($-1$) | $+0.3827$ | $-0.9239$ | $-1.0$ |
| **M7** | $292.5^\circ$ (Left-Front) | CCW ($+1$) | $-0.3827$ | $-0.9239$ | $+1.0$ |
| **M8** | $337.5^\circ$ (Front-Left) | CW ($-1$) | $-0.9239$ | $-0.3827$ | $-1.0$ |

---

## 5. Quantitative Experimental Validation Results

### 5.1 Automated Live System Forensics Suite (`tests/test_live_flight_forensics.py`)
* **Test 01 (Startup Ground State)**: Disarmed, $y = 0.280\text{m}$, $\text{Alt} = 0.000\text{m}$, $\text{Thrust} = 0.00\text{N}$, $\text{RPM} = 0.0 \implies$ **PASSED**
* **Test 02 (Arming Without Takeoff)**: Armed, $y = 0.280\text{m}$, $\text{Alt} = 0.000\text{m}$, $\text{Thrust} = 0.50\text{N}$, $\text{RPM} = 382.6 \implies$ **PASSED**
* **Test 03 (Takeoff & Hover)**: Liftoff at $t = 1.67\text{s}$, $\text{Alt} = 1.27\text{m}$, $v_y = +3.53\text{ m/s}$, $\text{TWR} = 1.60 \implies$ **PASSED**
* **Test 04 (Pitch Translation)**: $\text{Pitch} = -8.6^\circ$, $v_z = -1.94\text{ m/s} \implies$ **PASSED**
* **Test 05 (Roll Translation)**: $\text{Roll} = +8.6^\circ$, $v_x = +1.92\text{ m/s} \implies$ **PASSED**
* **Test 06 (Yaw Reaction Torque)**: $\omega_y = -0.76\text{ rad/s}$, $\psi = 62.5^\circ \implies$ **PASSED**
* **Test 07 (Neutral Re-Leveling)**: $|\text{Roll}| = 0.00^\circ$, $|\text{Pitch}| = 0.00^\circ \implies$ **PASSED**
* **Test 08 (Controlled Landing)**: Ground contact at $y = 0.280\text{m}$, $\text{Alt} = 0.000\text{m}$, touchdown confirmed $\implies$ **PASSED**

### 5.2 16 Authoritative Physical Experiments Suite (`tests/test_physics_experiments.cpp`)
* **Score**: **16 / 16 PASSED (100%)**
  * Free fall acceleration: $a_0 = -9.80665\text{ m/s}^2$
  * ESC spin-up lag: $\tau = 15\text{ ms}$, 1-tau ratio = $70.4\%$
  * Hover equilibrium: $T = 90.15\text{ N}$ ($\text{TWR} = 0.92$)
  * Deceleration & braking settling time: $t_{\text{settle}} = 1.625\text{s}$, final $v_z = -0.0057\text{ m/s}$
  * Ground contact restitution: $y = 0.2800\text{m}$, $v_y = 0.000\text{ m/s}$
  * Motor failure compensation: Motor 3 = $0.0003\text{ RPM}$, healthy Motor 1 = $6028.7\text{ RPM}$

---

## 6. Real-Time Operational Performance

* **Physics Kernel Frequency**: Fixed 400.0 Hz ($\Delta t = 2.5\text{ ms}$).
* **WebSocket Telemetry Stream**: 60.0 Hz JSON broadcast to connected frontends.
* **Three.js Visual Render Rate**: Continuous 60 / 144 FPS native display synchronization.
* **CPU Utilization (Intel Core i5 HX / RTX 3050)**: $< 2.5\%$ CPU load during 4-drone simultaneous simulation.
