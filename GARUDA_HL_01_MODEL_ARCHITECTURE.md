# GARUDA-HL-01 Model Architecture & Asset Pipeline Specification

## 1. Authoritative Architecture Summary

```
                      C++20 ENGINE
        garuda::model::GarudaVehicleSpecification
               (Single Source of Truth)
                          │
         ┌────────────────┴────────────────┐
         ▼                                 ▼
 C++ FLIGHT PHYSICS               C++ MANIFEST EXPORT
 (Rigid Body, BET, ESC)   (build/generated/GARUDA_HL_01_MODEL_SPEC.json)
         │                                 │
         │                                 ▼
         │                      PYTHON OFFLINE COMPILER
         │                     (Reads Manifest -> Builds GLB)
         │                                 │
         │                                 ▼
         │                            GLTF / GLB
         │                                 │
         └────────────────┬────────────────┘
                          ▼
                   GODOT 4 RUNTIME
          (VisualBridge maps Telemetry -> Transforms)
```

---

## 2. Authoritative Dimensions & Coordinate System

### 2.1 Coordinate Conventions
- **Forward Axis**: $-Z$ (Aviation nose heading matching Godot 4 camera forward)
- **Starboard / Right Axis**: $+X$
- **Up / Altitude Normal**: $+Y$
- **Rotor Shaft Axis**: Local $+Y$
- **Vehicle Origin**: Center of Mass / Airframe Datum at $(0, 0, 0)$
- **Motor / Rotor Origins**: Exact center of physical motor shaft at $(x_i, y_i, z_i)$

### 2.2 Numerical Specifications
- **Configuration**: 8-Rotor Octo-X
- **Motor-to-Motor Span**: **$1.100\text{ m}$**
- **Arm Length**: **$0.550\text{ m}$**
- **Angular Separation**: **$45.0^\circ$**
- **Initial Angle**: **$22.5^\circ$**
- **Radial Angles**:
  * Rotor 01: $22.5^\circ$ | Position: $(+0.2105, +0.0150, +0.5081)\text{ m}$ | **CW** ($-1$)
  * Rotor 02: $67.5^\circ$ | Position: $(+0.5081, +0.0150, +0.2105)\text{ m}$ | **CCW** ($+1$)
  * Rotor 03: $112.5^\circ$ | Position: $(+0.5081, +0.0150, -0.2105)\text{ m}$ | **CW** ($-1$)
  * Rotor 04: $157.5^\circ$ | Position: $(+0.2105, +0.0150, -0.5081)\text{ m}$ | **CCW** ($+1$)
  * Rotor 05: $202.5^\circ$ | Position: $(-0.2105, +0.0150, -0.5081)\text{ m}$ | **CW** ($-1$)
  * Rotor 06: $247.5^\circ$ | Position: $(-0.5081, +0.0150, -0.2105)\text{ m}$ | **CCW** ($+1$)
  * Rotor 07: $292.5^\circ$ | Position: $(-0.5081, +0.0150, +0.2105)\text{ m}$ | **CW** ($-1$)
  * Rotor 08: $337.5^\circ$ | Position: $(-0.2105, +0.0150, +0.5081)\text{ m}$ | **CCW** ($+1$)
- **Propeller Diameter**: 16-inch ($r = 0.2032\text{ m}$)
- **Adjacent Shaft Distance**: $0.421\text{ m}$
- **Propeller Tip Clearance**: $+1.5\text{ cm}$ (physical clearance verified mathematically)
- **Ground Clearance**: $0.360\text{ m}$

---

## 3. Node Hierarchy & Component Mapping

```
GARUDA_HL_01 (Root Datum (0,0,0))
│
├── AIRFRAME
│   ├── BODY_MAIN (9-Slice Faceted Stealth Diamond Fuselage & Upper Deck)
│   ├── AIRFRAME_SOCKETS (8x CNC Machined Arm Root Sockets & Quick-Release Rails)
│   └── AIRFRAME_CYAN_LIGHTS (Recessed Cyan Status Lightguides)
│
├── ARMS
│   ├── ARM_01 (0.55m Continuous Carbon Tube + Clamp Collar + Motor Flange)
│   ├── ARM_02
│   ├── ...
│   └── ARM_08
│
├── PROPULSION
│   ├── MOTOR_01
│   │   ├── MOTOR_CAN_01 (6215 High-Torque Brushless Rotor Bell & Stator)
│   │   ├── MOTOR_RING_01 (12-Vane Centrifugal CNC Red Anodized Cooling Ring)
│   │   ├── NAV_LED_01 (Navigation LED Aperture)
│   │   └── ROTOR_01 (Local offset: (0, 0.058, 0), Local axis: +Y)
│   │       ├── ROTOR_HUB_01 (Machined Aluminum Folding Hub)
│   │       ├── BLADE_A_01 (3D Cambered Aerofoil + Tactical White Tip Stripe)
│   │       └── BLADE_B_01 (180° Y-Opposed 3D Cambered Aerofoil Blade)
│   ├── ...
│   └── MOTOR_08
│
├── LANDING_GEAR
│   └── LANDING_GEAR_MAIN (Symmetrical Inverted A-Frame Struts & 0.62m Ground Skids)
│
├── PAYLOAD
│   └── PAYLOAD_MOUNT (Quick-Release Base & 4x Silicone Vibration Isolators)
│       └── GIMBAL_YAW (Continuous Pan Motor, Offset: [0, -0.023, 0])
│           └── GIMBAL_PITCH (CNC Aluminum U-Yoke Arm, Offset: [0, -0.017, 0])
│               └── GIMBAL_ROLL (Roll Bracket & Trunnions)
│                   └── CAMERA_BODY (Quad Optical Turret: 4K Day + FLIR + Laser + SWIR)
│
├── AVIONICS
│   ├── ANTENNA_SYSTEM (4x Multi-Band Whip Antennas & GNSS Receiver Domes)
│   └── BEACON_BASE -> LIGHTS_BEACON (Pulsing High-Intensity Red Warning Strobe)
│
└── LIGHTS
```

---

## 4. Reusable PBR Materials & Textures

| Material ID | Category | Appearance / Physical Property |
| :--- | :--- | :--- |
| `MAT_STEALTH_CARBON` | Matte Carbon Composite | Controlled roughness (0.42), subtle micro-weave, stealth black |
| `MAT_CARBON_TUBE` | Structural Carbon Boom | High-modulus unidirectional carbon tube with satin sheen (0.35) |
| `MAT_CNC_RED_ALUMINUM` | Anodized Aluminum | Metallic (0.92), deep industrial red (#C41818), roughness (0.24) |
| `MAT_DARK_METAL` | Machined Hardened Alloy | Metallic (0.88), dark titanium/tungsten (#22252A), roughness (0.30) |
| `MAT_OPTICAL_GLASS` | Multi-Coated Optical Lens | Transmissive glass, roughness (0.05), anti-reflective emerald sheen |
| `MAT_TACTICAL_WHITE` | High-Visibility Tip Stripe | High-reflectance matte white stripe on blade tips (#EEEEEE) |
| `MAT_CYAN_STATUS` | Emissive Lightguide | Cyan status illumination (#00E5FF), emission energy = 2.5 |
| `MAT_RED_STATUS` | Anti-Collision Beacon | Pulsing aviation red strobe (#FF1122), emission energy = 4.0 |

---

## 5. Verification & Test Suite

- **C++ Architectural Validator**: `test_model_architecture.exe` executes 20 rigorous mathematical checks.
- **Manifest Generator**: `generate_model_manifest.exe` produces deterministic manifest JSON.
- **Pipeline Runner**: `python scripts/pipeline/run_full_pipeline.py` builds and validates all standalone modules and master assets.
- **Post-Build Asset Validator**: `python scripts/pipeline/validate_built_asset.py` performs 39 structural checks across GLB node graphs.
- **Godot Test Suite**:
  * Complete Aircraft: [`scenes/GARUDA_MODEL_TEST.tscn`](file:///c:/GARUDA-HIVE-V2/scenes/GARUDA_MODEL_TEST.tscn)
  * Airframe: [`scenes/component_tests/BODY_TEST.tscn`](file:///c:/GARUDA-HIVE-V2/scenes/component_tests/BODY_TEST.tscn)
  * Arm: [`scenes/component_tests/ARM_TEST.tscn`](file:///c:/GARUDA-HIVE-V2/scenes/component_tests/ARM_TEST.tscn)
  * Motor: [`scenes/component_tests/MOTOR_TEST.tscn`](file:///c:/GARUDA-HIVE-V2/scenes/component_tests/MOTOR_TEST.tscn)
  * Rotor: [`scenes/component_tests/ROTOR_TEST.tscn`](file:///c:/GARUDA-HIVE-V2/scenes/component_tests/ROTOR_TEST.tscn)
  * Gimbal: [`scenes/component_tests/GIMBAL_TEST.tscn`](file:///c:/GARUDA-HIVE-V2/scenes/component_tests/GIMBAL_TEST.tscn)
