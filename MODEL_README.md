# GARUDA-HL-01 Mechanical 3D Asset Documentation

## 1. Engineering Specifications & Dimensions
- **Vehicle Type**: Heavy-Lift Reconnaissance & Tactical Utility UAV
- **Airframe Configuration**: 8-Rotor Octo-X (45.0° equidistant radial separation)
- **Target Motor-to-Motor Span**: **1.10 m** (authoritative engineering scale)
- **Arm Length**: **0.55 m** (chassis socket to motor shaft center)
- **Fuselage Dimensions**: 0.49 m (Length) × 0.33 m (Width) × 0.17 m (Height)
- **Rotor Diameter**: 0.41 m (16.2-inch folding carbon aerofoil blades)
- **Propeller Tip Clearance**: **+12.6 cm** between adjacent blades (zero collision/overlap)
- **Landing Gear Height**: 0.36 m (Inverted A-frame carbon ground skids)
- **Vehicle Origin**: `(0.0, 0.0, 0.0)` at Vehicle Reference Frame / Center of Mass (CoM)

---

## 2. Rotor Angles & Alternating CW / CCW Conventions
Every rotor origin is placed **exactly at the motor shaft center** with local $+Y$ defined as the rotor shaft axis.

| Rotor Node | Angle | Position $(X, Y, Z)$ | Rotation Direction | Local Axis | Sign |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `ROTOR_01` | $22.5^\circ$ | $(+0.210, +0.073, +0.508)$ | **CW** (Clockwise) | $+Y$ | $-1.0$ |
| `ROTOR_02` | $67.5^\circ$ | $(+0.508, +0.073, +0.210)$ | **CCW** (Counter-Clockwise) | $+Y$ | $+1.0$ |
| `ROTOR_03` | $112.5^\circ$ | $(+0.508, +0.073, -0.210)$ | **CW** (Clockwise) | $+Y$ | $-1.0$ |
| `ROTOR_04` | $157.5^\circ$ | $(+0.210, +0.073, -0.508)$ | **CCW** (Counter-Clockwise) | $+Y$ | $+1.0$ |
| `ROTOR_05` | $202.5^\circ$ | $(-0.210, +0.073, -0.508)$ | **CW** (Clockwise) | $+Y$ | $-1.0$ |
| `ROTOR_06` | $247.5^\circ$ | $(-0.508, +0.073, -0.210)$ | **CCW** (Counter-Clockwise) | $+Y$ | $+1.0$ |
| `ROTOR_07` | $292.5^\circ$ | $(-0.508, +0.073, +0.210)$ | **CW** (Clockwise) | $+Y$ | $-1.0$ |
| `ROTOR_08` | $337.5^\circ$ | $(-0.210, +0.073, +0.508)$ | **CCW** (Counter-Clockwise) | $+Y$ | $+1.0$ |

---

## 3. Node Hierarchy Architecture

```
GARUDA_HL_01
│
├── AIRFRAME
│   └── BODY_CORE (Faceted Stealth Diamond Canopy & Avionics Deck)
│
├── ARMS
│   ├── ARM_01 (Root Socket + Clamp Collar + Carbon Tube + Motor Flange)
│   ├── ARM_02
│   ├── ARM_03
│   ├── ARM_04
│   ├── ARM_05
│   ├── ARM_06
│   ├── ARM_07
│   └── ARM_08
│
├── PROPULSION
│   ├── MOTOR_01
│   │   ├── MOTOR_CAN_01
│   │   ├── MOTOR_RING_01 (CNC Red Anodized Heat Sink)
│   │   ├── NAV_LED_01
│   │   └── ROTOR_01 (Origin at shaft center, local +Y axis)
│   │       ├── ROTOR_HUB_01
│   │       ├── BLADE_A (Cambered Aerofoil + White Tip Stripe)
│   │       └── BLADE_B (180° Y-Opposed Cambered Aerofoil)
│   │
│   ├── MOTOR_02 .. MOTOR_08 (Independent controllable nodes)
│
├── LANDING_GEAR
│   └── LANDING_GEAR_MAIN (Symmetrical Inverted A-Frame Carbon Skids)
│
├── PAYLOAD
│   ├── PAYLOAD_MOUNT (Vibration Damper Plate & Silicone Isolators)
│   └── GIMBAL_YAW (Origin at Yaw axis)
│       └── GIMBAL_PITCH (Origin at Pitch axis)
│           └── GIMBAL_ROLL (Origin at Roll axis)
│               └── CAMERA_BODY (Quad-Lens EO/IR Turret)
│                   └── CAMERA_LENSES
│
├── ANTENNAS
│   └── ANTENNA_SYSTEM (4x Multi-Band Whip Antennas)
│
└── LIGHTS
    ├── LIGHTS_CYAN_GUIDES (Front Snout & Shoulder Accents)
    ├── NAV_LED_01 .. NAV_LED_08 (Starboard Green / Port Red)
    └── BEACON_BASE -> LIGHTS_BEACON (Pulsing Red Top Strobe)
```

---

## 4. Reusable PBR Material Library
1. `MAT_STEALTH_CARBON`: Radar absorbent matte black composite ($RGB=0.055, 0.060, 0.075$, Roughness $0.28$, Metallic $0.90$).
2. `MAT_CARBON_TUBE`: Structural twill carbon fiber ($RGB=0.035, 0.040, 0.050$, Roughness $0.20$, Metallic $0.85$).
3. `MAT_CNC_RED_ALUMINUM`: High-contrast red anodized aluminum cooling rings ($RGB=0.95, 0.05, 0.05$, Metallic $0.98$).
4. `MAT_CYAN_LIGHTGUIDE`: Cyberpunk glowing cyan HUD lightguides ($RGB=0.0, 0.95, 1.0$, Emissive).
5. `MAT_RED_STROBE`: Top anti-collision beacon ($RGB=1.0, 0.05, 0.05$, Emissive).
6. `MAT_OPTICAL_GLASS`: Anti-reflective optical sensor glass ($RGB=0.02, 0.08, 0.16$, Roughness $0.02$).
7. `MAT_TACTICAL_WHITE`: High-visibility propeller tip stripes ($RGB=0.98, 0.98, 0.98$).
8. `MAT_PROPELLER_CARBON`: Lightweight carbon aerofoil blades ($RGB=0.08, 0.09, 0.11$).
9. `MAT_NAV_GREEN` / `MAT_NAV_RED`: Aviation standard starboard/port navigation markers.

---

## 5. Model Validation & Testing
Launch the validation scene in Godot 4:
```bash
godot.exe --path . res://scenes/GARUDA_MODEL_TEST.tscn
```
- **Click & Drag**: Orbit 360° around the model.
- **Scroll Wheel**: Macro zoom from $0.45\text{m}$ to $6.0\text{m}$.
- **Master RPM Slider**: Spin all 8 rotors in alternating CW/CCW directions.
- **Gimbal Sliders**: Test Yaw, Pitch, and Roll articulation.
- **8 Orthographic View Buttons**: Jump to 01 FRONT, 02 BACK, 03 LEFT, 04 RIGHT, 05 TOP, 06 BOTTOM, 07 FRONT 45°, 08 REAR 45°.
