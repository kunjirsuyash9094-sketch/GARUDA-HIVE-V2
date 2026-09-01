# GARUDA HIVE V2 — Multirotor Flight Dynamics Mathematical Formulation

**Document**: `DRONE_FLIGHT_DYNAMICS.md`  
**Classification**: Engineering Specification  
**Canonical Vehicle**: `GARUDA-HL-01` 8-Rotor Heavy-Lift Industrial Octocopter  

---

## 1. Coordinate Systems & Reference Frames

GARUDA HIVE V2 uses standard aerospace conventions for internal dynamic calculations:

1. **Earth-Centered Inertial / World Frame ($\mathcal{W}$)**:
   * Cartesian coordinates $(x_w, y_w, z_w)$ with $+Y_w = \text{Up}$ (Y-up convention for 3D visualization and Godot compatibility) or NED $(\text{North}, \text{East}, \text{Down})$.
2. **Body-Fixed Frame ($\mathcal{B}$)**:
   * Origin fixed at the vehicle Center of Mass (CoM).
   * $+X_b = \text{Forward}$, $+Y_b = \text{Right}$, $+Z_b = \text{Down}$ (FRD Aerospace Standard).
   * Transformed from Godot body frame via permutation matrix $P = \begin{bmatrix} 0 & 0 & -1 \\ 1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix}$.

---

## 2. 6-DOF Newton-Euler Equations of Motion

### 2.1. Translational Kinematics & Dynamics `[PHYSICALLY MODELED]`

$$\dot{\vec{p}}_w = \vec{v}_w$$

$$m_{\text{eff}} \dot{\vec{v}}_w = \vec{F}_{\text{total}, w} = R(q) \vec{F}_{\text{body}} + \vec{F}_{\text{gravity}, w} + \vec{F}_{\text{aero\_drag}, w} + \vec{F}_{\text{ground}, w}$$

Where:
* $m_{\text{eff}} = m_{\text{dry}} + m_{\text{payload}}$ is the total mass ($10.0\text{ kg}$ nominal, $15.0\text{ kg}$ MTOW).
* $\vec{F}_{\text{gravity}, w} = \begin{bmatrix} 0 \\ -m_{\text{eff}} g \\ 0 \end{bmatrix}$ with $g = 9.80665\text{ m/s}^2$.
* $\vec{F}_{\text{body}} = \begin{bmatrix} 0 \\ \sum_{i=1}^8 T_i \\ 0 \end{bmatrix}$ in body frame ($+Y$ up).
* $\vec{F}_{\text{aero\_drag}, w} = -\frac{1}{2} \rho C_D A \|\vec{v}_{\text{rel}}\| \vec{v}_{\text{rel}} - C_{\text{linear}} \vec{v}_{\text{rel}}$, with relative airflow $\vec{v}_{\text{rel}} = \vec{v}_w - \vec{v}_{\text{wind}, w}$.

### 2.2. Rotational Kinematics & Dynamics `[PHYSICALLY MODELED]`

$$\dot{\vec{\omega}}_b = I_{\text{eff}}^{-1} \left( \vec{\tau}_{\text{body}} - \vec{\omega}_b \times (I_{\text{eff}} \vec{\omega}_b) \right)$$

$$\dot{q} = \frac{1}{2} q \otimes \begin{bmatrix} 0 \\ \vec{\omega}_b \end{bmatrix} = \frac{1}{2} \begin{bmatrix} -\vec{q}_v \cdot \vec{\omega}_b \\ q_w \vec{\omega}_b + \vec{q}_v \times \vec{\omega}_b \end{bmatrix}$$

Where:
* $I_{\text{eff}} = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ is the effective inertia tensor adjusted for payload via the Parallel-Axis Theorem.
* $\vec{\tau}_{\text{body}} = \vec{\tau}_{\text{thrust\_arms}} + \vec{\tau}_{\text{reaction\_drag}} + \vec{\tau}_{\text{gyroscopic}} + \vec{\tau}_{\text{ground}}$.

---

## 3. Octo-X Geometry & Propulsion Allocation `[PHYSICALLY MODELED]`

For 8 rotors positioned radially at angles $\psi_i = 22.5^\circ + i \cdot 45.0^\circ$ with arm radius $R_{\text{arm}} = 0.55\text{ m}$:

| Rotor # | Angle $\psi_i$ | Position $(X_b, Y_b)$ (FRD) | Spin Direction | Pitch Moment Coeff $k_P$ | Roll Moment Coeff $k_R$ | Yaw Moment Coeff $k_Y$ |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **M1** | $22.5^\circ$ | $(+0.210\text{m}, +0.508\text{m})$ | CCW (+1) | $+\sin(22.5^\circ) = +0.383$ | $-\cos(22.5^\circ) = -0.924$ | $+1.0$ |
| **M2** | $67.5^\circ$ | $(+0.508\text{m}, +0.210\text{m})$ | CW (-1) | $+\sin(67.5^\circ) = +0.924$ | $-\cos(67.5^\circ) = -0.383$ | $-1.0$ |
| **M3** | $112.5^\circ$ | $(+0.508\text{m}, -0.210\text{m})$ | CCW (+1) | $+\sin(112.5^\circ) = +0.924$ | $-\cos(112.5^\circ) = +0.383$ | $+1.0$ |
| **M4** | $157.5^\circ$ | $(+0.210\text{m}, -0.508\text{m})$ | CW (-1) | $+\sin(157.5^\circ) = +0.383$ | $-\cos(157.5^\circ) = +0.924$ | $-1.0$ |
| **M5** | $202.5^\circ$ | $(-0.210\text{m}, -0.508\text{m})$ | CCW (+1) | $+\sin(202.5^\circ) = -0.383$ | $-\cos(202.5^\circ) = +0.924$ | $+1.0$ |
| **M6** | $247.5^\circ$ | $(-0.508\text{m}, -0.210\text{m})$ | CW (-1) | $+\sin(247.5^\circ) = -0.924$ | $-\cos(247.5^\circ) = +0.383$ | $-1.0$ |
| **M7** | $292.5^\circ$ | $(-0.508\text{m}, +0.210\text{m})$ | CCW (+1) | $+\sin(292.5^\circ) = -0.924$ | $-\cos(292.5^\circ) = -0.383$ | $+1.0$ |
| **M8** | $337.5^\circ$ | $(-0.210\text{m}, +0.508\text{m})$ | CW (-1) | $+\sin(337.5^\circ) = -0.383$ | $-\cos(337.5^\circ) = -0.924$ | $-1.0$ |

$$\vec{\tau}_{\text{arms}} = \begin{bmatrix} \sum_{i=1}^8 -R_{\text{arm}} \cos(\psi_i) T_i \\ \sum_{i=1}^8 R_{\text{arm}} \sin(\psi_i) T_i \\ 0 \end{bmatrix}, \quad \vec{\tau}_{\text{reaction}} = \begin{bmatrix} 0 \\ 0 \\ \sum_{i=1}^8 \text{spin}_i Q_i \end{bmatrix}$$

---

## 4. Motor Dynamics & ESC Response `[PHYSICALLY MODELED]`

$$\frac{d\omega_i}{dt} = \frac{\omega_{\text{command}, i} - \omega_i}{\tau_{\text{ESC}}}, \quad \tau_{\text{ESC}} = 0.015\text{ s}$$

$$\omega_i(t + \Delta t) = \omega_i(t) + (1 - e^{-\Delta t / \tau_{\text{ESC}}}) (\omega_{\text{command}, i} - \omega_i(t))$$

Motor limits:
* $\omega_{\text{idle}} = 1100\text{ RPM} = 115.19\text{ rad/s}$
* $\omega_{\text{max}} = 6200\text{ RPM} = 649.26\text{ rad/s}$

---

## 5. Control Cascade Architecture `[PHYSICALLY MODELED]`

1. **Outer Attitude Loop (P)**:
   $$\vec{\omega}_{\text{demand}} = \begin{bmatrix} K_{p, \text{att}} (\phi_{\text{target}} - \phi) \\ K_{p, \text{att}} (\theta_{\text{target}} - \theta) \\ \dot{\psi}_{\text{target}} \end{bmatrix}$$
2. **Inner Angular Rate Loop (PID)**:
   $$\vec{\tau}_{\text{demand}} = K_p (\vec{\omega}_{\text{demand}} - \vec{\omega}_b) + K_i \int (\vec{\omega}_{\text{demand}} - \vec{\omega}_b) dt + K_d \frac{d(\vec{\omega}_{\text{demand}} - \vec{\omega}_b)}{dt}$$
3. **Mixer Matrix Output**:
   $$u_i = \text{clamp}\left( \max\left(u_{\text{throttle}} + k_{R, i} \tau_{\text{roll}} + k_{P, i} \tau_{\text{pitch}} + k_{Y, i} \tau_{\text{yaw}}, u_{\text{idle}}\right), 0.0, 1.0 \right)$$
