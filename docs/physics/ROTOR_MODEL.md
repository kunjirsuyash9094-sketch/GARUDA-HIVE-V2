# GARUDA HIVE V2 — Aerodynamic Rotor & Blade Element Theory (BET) Model

**Document**: `ROTOR_MODEL.md`  
**Classification**: Engineering Specification  
**Canonical Vehicle**: `GARUDA-HL-01` 8-Rotor Heavy-Lift Industrial Octocopter  

---

## 1. Overview & Aerodynamic Methodology

Propulsion forces and torques for each of the 8 rotors on `GARUDA-HL-01` are computed using **24-annulus Blade Element Theory (BET)** combined with **Rankine-Froude Momentum Theory Induced Inflow**.

* **Rotor Diameter**: $15\text{ inches}$ ($0.381\text{ m}$) $\implies$ Radius $R = 0.1905\text{ m}$
* **Hub Root Cutout ($r_0$)**: $0.025\text{ m}$
* **Blades per Propeller ($B$)**: 2 blades
* **Mean Blade Chord ($c$)**: $0.032\text{ m}$ (Root) $\rightarrow 0.016\text{ m}$ (Tip)
* **Blade Washout Twist**: Geometric twist $\theta(r) = 16.0^\circ - \left(\frac{r - r_0}{R - r_0}\right) 9.0^\circ$ ($16.0^\circ \rightarrow 7.0^\circ$)
* **Radial Annular Discretization ($N$)**: 24 elements ($\Delta r = \frac{R - r_0}{24} = 0.0069\text{ m}$)

---

## 2. Mathematical Formulation `[PHYSICALLY MODELED]`

For each annular blade element $j$ at radial distance $r_j$:

### 2.1. Local Relative Flow Velocity
$$V_{\text{axial}, j} = V_{z, \text{rel}} + v_{\text{induced}, j}$$
$$V_{\text{tan}, j} = \omega r_j - V_{x, \text{rel}}$$
$$V_{\text{total}, j} = \sqrt{V_{\text{axial}, j}^2 + V_{\text{tan}, j}^2}$$
$$\phi_j = \arctan\left(\frac{V_{\text{axial}, j}}{V_{\text{tan}, j}}\right)$$

### 2.2. Inflow Velocity via Momentum Balance
The induced velocity $v_{\text{induced}}$ is solved iteratively across the annular disk area $\Delta A_j = 2\pi r_j \Delta r$:
$$v_{\text{induced}, j} = \sqrt{-\frac{V_{\text{axial}, j}^2}{4} + \sqrt{\left(\frac{V_{\text{axial}, j}^2}{4}\right)^2 + \left(\frac{\Delta T_j}{2 \rho \Delta A_j}\right)^2}} - \frac{V_{\text{axial}, j}}{2}$$

### 2.3. Aerodynamic Section Lift and Drag
The local angle of attack is:
$$\alpha_j = \theta(r_j) - \phi_j$$

Lift and drag coefficients from thin-airfoil theory with stall modeling:
$$C_L(\alpha_j) = \begin{cases} 2\pi \cdot (\alpha_j - \alpha_0) & \text{if } |\alpha_j| \le \alpha_{\text{stall}} \\ C_{L, \text{max}} \cdot \text{sgn}(\alpha_j) \cos(\alpha_j) & \text{if } |\alpha_j| > \alpha_{\text{stall}} \end{cases}$$
$$C_D(\alpha_j) = C_{D, 0} + \frac{C_L^2(\alpha_j)}{\pi \text{AR} e}$$

Elemental dynamic pressure:
$$q_j = \frac{1}{2} \rho V_{\text{total}, j}^2$$
$$\Delta L_j = q_j c(r_j) C_L(\alpha_j) \Delta r \cdot B$$
$$\Delta D_j = q_j c(r_j) C_D(\alpha_j) \Delta r \cdot B$$

### 2.4. Total Rotor Thrust, Torque & Mechanical Power
$$\Delta T_j = \Delta L_j \cos\phi_j - \Delta D_j \sin\phi_j$$
$$\Delta Q_j = (\Delta L_j \sin\phi_j + \Delta D_j \cos\phi_j) r_j$$

$$T_i = \sum_{j=1}^{24} \Delta T_j, \quad Q_i = \sum_{j=1}^{24} \Delta Q_j, \quad P_{\text{mech}, i} = Q_i \omega_i$$

---

## 3. Gyroscopic Precession Moments `[PHYSICALLY MODELED]`

Because spinning rotors possess angular momentum $L_{\text{rotor}} = I_{\text{prop}} \omega_i \vec{k}$, vehicle body rotation $\vec{\omega}_b$ induces gyroscopic precession torque:

$$\vec{\tau}_{\text{gyro}, i} = \vec{\omega}_b \times \begin{bmatrix} 0 \\ \text{spin}_i I_{\text{prop}} \omega_i \\ 0 \end{bmatrix}$$

For 8 counter-rotating rotors (4 CCW, 4 CW), net gyroscopic moments cancel in symmetric hover and manifest only during asymmetric motor maneuvers or single motor failure.

---

## 4. Vortex Ring State (VRS) Formulation `[PHYSICALLY MODELED]`

During steep descent, the empirical Leishman model attenuates thrust when descending into induced downwash:

$$\bar{V}_z = \frac{-v_{y, \text{world}}}{v_{\text{hover\_induced}}}, \quad v_{\text{hover\_induced}} = \sqrt{\frac{T_{\text{total}}}{2 \rho A_{\text{total}}}}$$

$$\text{If } 0.5 \le \bar{V}_z \le 1.5 \implies k_{\text{VRS}} = 1.0 - 0.35 \sin\left(\pi \frac{\bar{V}_z - 0.5}{1.0}\right)$$

$$T_{\text{effective}, i} = k_{\text{VRS}} \cdot T_i$$

---

## 5. Visual RPM-Driven Rendering `[PHYSICALLY MODELED]`

In the 3D WebGL client (`web/garuda_octocopter.js`), visual propeller rotation integrates the authoritative simulation angular velocity:

$$\Delta \theta_i = \text{spin}_i \cdot \left(\frac{\text{RPM}_i \cdot 2\pi}{60}\right) \cdot \Delta t$$

* High-RPM motion blur discs fade in smoothly above $2500\text{ RPM}$ as a visual supplement without replacing the underlying 3D cambered blade structure.
