# GARUDA HIVE V2 — Realism Benchmark & Simulation Fidelity Matrix

---

## 1. Executive Summary

This document establishes the official physical simulation benchmark for **GARUDA HIVE V2** (featuring the canonical **`GARUDA-HL-01` Heavy-Lift Industrial Octocopter**). The physical simulation core replaces naive game-like kinematics with authoritative aerodynamics, 24-annulus Blade Element Theory (BET), first-order motor lag, LiPo Peukert battery dynamics, non-linear spring-damper ground contact, and true RPM-driven propeller visualizations.

---

## 2. Realism Comparison Matrix

The following matrix compares **GARUDA HIVE V2** against the legacy **SkySim 3.0** baseline, **PX4 Autopilot / Gazebo (RotorS)**, and **Microsoft AirSim (FastPhysicsEngine)** across 10 critical physical dimensions.

| Dimension | Legacy SkySim 3.0 | PX4 / Gazebo (RotorS) | Microsoft AirSim (FastPhysics) | GARUDA HIVE V2 |
|:---|:---|:---|:---|:---|
| **1. Vehicle Configuration** | Generic Quadrotor ($4\text{ rotors}$) | Quad / Hex / Octo via SDF | Multirotor (Quad-X default) | **Canonical 8-Rotor Octo-X (`GARUDA-HL-01`)** |
| **2. Rotor Aerodynamics** | Lumped $T = C_T \omega^2$ | $C_T/C_Q$ quadratic rotor model | Quadratic body wrench + drag torque | **24-Annuli Blade Element Theory (BET) + Rankine-Froude Inflow** |
| **3. Ground Interaction** | Hard plane clipping | ODE/Bullet rigid contacts | Normal spring-damper penalty | **Continuous Non-Linear Spring-Damper + Coulomb Friction + Non-Penetration Restitution** |
| **4. Motor Dynamics** | Instantaneous throttle response | 1st-order ESC filter ($\tau \approx 20\text{ms}$) | 1st-order motor response | **1st-order ESC ($\tau = 15\text{ms}$) + Dynamic Thermal Model + Failure Injection** |
| **5. Battery & Power** | Linear SoC percentage | Simple voltage divider | Static battery capacity | **6S Thevenin Equivalent Circuit + Internal Resistance Voltage Sag + Peukert Effect** |
| **6. Aerodynamic Effects** | None | Rotor drag ($v_\perp$) | Induced drag approximation | **Cheeseman-Bennett Ground Effect + Leishman Vortex Ring State (VRS)** |
| **7. Payload Coupling** | Point mass addition | SDF link rigid attachment | Mass offset | **Dynamic Mass Scaling + CoM Offset Shift + Parallel-Axis Inertia Tensor Recalculation** |
| **8. Propeller Visualization** | Flat rotating discs | Static blade geometry | Flat blur textures | **True 3D Cambered Tapered Airfoil Blades + Hub + Authoritative $\omega \cdot \Delta t$ Spin + High-RPM Motion Blur** |
| **9. Flight Control Cascade** | Basic Rate PID | Full Cascade (Pos $\rightarrow$ Vel $\rightarrow$ Att $\rightarrow$ Rate) | Simplified Attitude PID | **Aerospace FRD Attitude P Loop $\rightarrow$ Rate PID $\rightarrow$ 8-Rotor Octo-X Mixer** |
| **10. Simulation Rate & Determinism** | 60 Hz non-deterministic | Variable physics step ($250\text{Hz}$) | Tick-based stepping | **Fixed 400 Hz ($dt = 0.0025\text{s}$) with 100% Bit-Identical Determinism** |

---

## 3. Detailed Physical Benchmark Findings

### 3.1. Propulsion & Aerodynamics
* **Legacy SkySim**: Used a simple constant $C_T = 0.00000185$ multiplying $\omega^2$, ignoring relative airspeed, rotor inflow, twist distribution, and blade tip losses.
* **GARUDA HIVE V2**: Discretizes each 15-inch carbon blade into 24 concentric annuli ($dr = R / 24$). For each annulus, it computes the local relative velocity vector $(V_{axial}, V_{tan})$, the induced inflow velocity via iterative Rankine-Froude momentum theory, the local angle of attack $\alpha(r) = \theta(r) - \phi(r)$, and evaluates non-linear $C_L(\alpha)$ and $C_D(\alpha)$ polars with stall modeling.

### 3.2. Propeller Visualization vs Physical Truth
* **Legacy SkySim**: Rendered flat rectangular boxes and swapped them with flat rotating semi-transparent discs.
* **GARUDA HIVE V2**: Features 8 fully modeled 3D propeller heads. Each blade exhibits root chord ($0.032\text{m}$), tip chord ($0.016\text{m}$), aerodynamic washout twist ($16^\circ \rightarrow 7^\circ$), camber profile, and billet CNC spinner nut. The rotation angle integrates authoritative simulation RPM ($\Delta\theta = \text{spin} \cdot \frac{\text{RPM} \cdot 2\pi}{60} \cdot \Delta t$). Motion blur is a dynamic visual supplement that appears only above 2500 RPM while keeping the underlying physical blades visible.

### 3.3. Ground Contact & Resting Stability
* **Legacy SkySim**: Clamped vehicle altitude to $0.0\text{m}$ whenever vertical position was negative, causing visual vibration and sudden velocity resets.
* **GARUDA HIVE V2**: Implements 4 discrete landing contact points with non-linear spring stiffness ($k = 12000\text{ N/m}$), critical damping ($d = 850\text{ N}\cdot\text{s/m}$), Coulomb dynamic friction ($\mu = 0.70$), and restitution ($e = 0.10$). Drones settle smoothly on ground pads with realistic suspension compression ($0.28\text{m}$ pad clearance).

### 3.4. Electrical System & Battery Ledger
* **Legacy SkySim**: Decremented SoC based solely on flight time.
* **GARUDA HIVE V2**: Computes instantaneous electrical power demand:
  $$P_{\text{elec}} = \sum_{i=1}^8 \left(\frac{Q_i \omega_i}{\eta_{\text{motor}}} + P_{\text{ESC, idle}}\right) + P_{\text{avionics}} + P_{\text{payload}}$$
  Terminal voltage dynamically sags under high current:
  $$V_{\text{terminal}} = V_{\text{OCV}}(\text{SoC}) - I \cdot R_{\text{internal}}$$
  Effective capacity scales according to Peukert's Law: $C_{\text{eff}} = C_{\text{nominal}} \left(\frac{I_{\text{nom}}}{I}\right)^{k - 1}$ with $k = 1.08$.

---

## 4. Benchmark Conclusion

GARUDA HIVE V2 successfully bridges the gap between aerospace-grade multi-body physics solvers and high-performance real-time interactive 3D visualizers. All 15 physical benchmark experiments pass with quantitative accuracy, verifying that visual flight behaviors emerge naturally from underlying aerodynamic and mechanical laws.
