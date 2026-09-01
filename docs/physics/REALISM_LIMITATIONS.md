# GARUDA HIVE V2 — Realism Assumptions, Engineering Trade-Offs & Limitations

**Document**: `REALISM_LIMITATIONS.md`  
**Classification**: Engineering Specification & Limitations Ledger  
**Canonical Vehicle**: `GARUDA-HL-01` 8-Rotor Heavy-Lift Industrial Octocopter  

---

## 1. Classification Methodology

To maintain complete engineering transparency, every physical subsystem in GARUDA HIVE V2 is explicitly tagged with one of four implementation statuses:

1. **`PHYSICALLY MODELED`**: First-principles physical equations solved explicitly at high rate ($400\text{ Hz}$) without empirical shortcuts.
2. **`SIMPLIFIED`**: Analytically sound approximations designed for real-time execution while preserving physical causality.
3. **`TUNED`**: Calibrated semi-empirical models parameterized against bench or wind-tunnel test data.
4. **`NOT IMPLEMENTED`**: Higher-order aerodynamic phenomena deferred to future research phases.

---

## 2. Subsystem Classification & Limitations Ledger

### 2.1. Aerodynamics & Propulsion
* **24-Annulus Blade Element Theory (BET)**: `[PHYSICALLY MODELED]`
  * *Assumptions*: Discretized radial annuli with 2D thin airfoil theory; linear/stalled section lift and drag polars.
  * *Limitation*: Does not model unsteady 3D rotor wake shedding or blade-to-blade vortex wake impingement between adjacent arms.
* **Rankine-Froude Induced Inflow**: `[PHYSICALLY MODELED]`
  * *Assumptions*: Uniform annular induced velocity balance.
* **Cheeseman-Bennett Ground Effect**: `[PHYSICALLY MODELED]`
  * *Assumptions*: Image rotor potential flow representation valid for $h \ge 0.5 R$.
* **Vortex Ring State (VRS)**: `[TUNED]`
  * *Assumptions*: Empirical Leishman thrust attenuation function applied during steep vertical descent ($0.5 \le \bar{V}_z \le 1.5$).
* **Aeroelastic Blade Flapping ($\beta(t)$)**: `[NOT IMPLEMENTED]`
  * *Future Scope*: Carbon-fiber propeller hinge/bending dynamics at high forward speeds ($>25\text{ m/s}$).

### 2.2. Actuation & Powertrain
* **First-Order ESC Lag ($\tau = 15\text{ms}$)**: `[PHYSICALLY MODELED]`
  * *Assumptions*: Linear motor speed response $\dot{\omega} = (\omega_{\text{cmd}} - \omega)/\tau$.
* **Motor Stator Thermal Dissipation**: `[SIMPLIFIED]`
  * *Assumptions*: Lumped thermal mass with $I^2 R$ resistive heating and propeller slipstream convective cooling.
* **6S LiPo Thevenin Equivalent Circuit**: `[PHYSICALLY MODELED]`
  * *Assumptions*: State of Charge (SoC) open-circuit curve $V_{\text{OCV}}(\text{SoC})$ with dynamic $I \cdot R_{\text{int}}$ voltage sag.
* **Peukert Capacity Scaling ($k = 1.08$)**: `[PHYSICALLY MODELED]`
  * *Assumptions*: High current draw exponentially reduces effective Ah discharge capacity.

### 2.3. Multi-Body Mechanics & Rigid Body Dynamics
* **6-DOF Newton-Euler Dynamic Equations**: `[PHYSICALLY MODELED]`
  * *Assumptions*: Rigid airframe body with semi-implicit Euler integration and unit quaternion kinematics.
* **Payload Coupling (Mass, CoM Shift, Parallel-Axis Tensor)**: `[PHYSICALLY MODELED]`
  * *Assumptions*: Payloads are rigidly attached to the central payload bay.
  * *Limitation*: Slung load pendulum dynamics on flexible cables are not currently modeled.
* **Spring-Damper Ground Contact**: `[PHYSICALLY MODELED]`
  * *Assumptions*: Continuous penalty spring stiffness ($12,000\text{ N/m}$), critical damping ($850\text{ N}\cdot\text{s/m}$), Coulomb friction ($\mu=0.70$), and restitution ($e=0.10$).

### 2.4. Environment & Atmosphere
* **ISA Troposphere Model**: `[PHYSICALLY MODELED]`
  * *Assumptions*: International Standard Atmosphere density, temperature, and pressure lapse rates up to $11,000\text{m}$.
* **Dryden Turbulence Model (MIL-HDBK-1797)**: `[TUNED]`
  * *Assumptions*: Spatially correlated stochastic gust filters driven by altitude and wind speed.
  * *Limitation*: Full 3D voxel grid CFD micro-weather urban wind canyons are deferred to Phase 3.

---

## 3. Real-Time Performance & Numerical Limits

* **Physics Timestep ($dt$)**: Fixed $0.0025\text{ s}$ ($400\text{ Hz}$).
* **Numerical Precision**: 64-bit IEEE 754 Floating-Point arithmetic throughout the physics core.
* **Bit-Level Determinism**: Verified 100% reproducible hash state across repeated runs.
* **Maximum Recommended Flight Speed**: $25.0\text{ m/s}$ ($90\text{ km/h}$).
* **Maximum Recommended Wind Speed**: $15.0\text{ m/s}$ ($30\text{ knots}$).
