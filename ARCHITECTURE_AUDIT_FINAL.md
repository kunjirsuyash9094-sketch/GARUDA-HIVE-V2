# GARUDA-HL-01 Architecture Audit & Manifest Migration Plan (FINAL)

## 1. Current Source of Truth Analysis

| Subsystem | Current Source of Truth | Intended Architectural Authority | Status / Issue |
| :--- | :--- | :--- | :--- |
| **Physics Dynamics & Flight Control** | `include/garuda/config/quadrotor_config.hpp` & `include/garuda/physics/` | **C++20** (`garuda::QuadrotorConfig`) | Authoritative (Preserve 100%) |
| **Motor Positions & Layout** | `include/garuda/physics/motor_system.hpp` (`init_positions()`) | **C++20** (`garuda::model::VehicleSpecification`) | Duplicated in Python & GDScript |
| **Mechanical Component Hierarchy** | Python (`scripts/pipeline/assemble_garuda_master.py`) | **C++20** (`garuda::model::VehicleSpecification`) | Must move to C++20 |
| **Visual Geometry Generation** | Python (`scripts/pipeline/build_*.py`) | **Python Offline Asset Compiler (Manifest-Driven)** | Refactor to consume C++ manifest |
| **Compiled 3D Visual Asset** | `models/GARUDA_HL_01/00_MASTER/GARUDA_HL_01_MASTER.glb` | **GLTF 2.0 Binary Representation** | Recompiled via manifest pipeline |
| **Runtime Rendering & Visualization** | Godot 4 (`scenes/GARUDA_MODEL_TEST.tscn`) | **Godot 4 Visual Adapter** | Connect to C++ `VisualBridge` |

---

## 2. Duplication & Conflict Map

### 2.1 Arm Length & Motor Span (`0.55m` / `1.10m`)
- **Authoritative Definition**: `include/garuda/config/quadrotor_config.hpp` (Line 33: `double arm_length_m{0.550};`)
- **Duplicated in Python**:
  * `scripts/pipeline/assemble_garuda_master.py` (Line 109: `arm_len = 0.55`)
  * `scripts/pipeline/build_02_arm_master.py` (Line 27: `arm_len = 0.55`)
  * `scripts/build_master_garuda_model.py` (Line 609: `arm_len = 0.55`)
- **Duplicated in GDScript / Scenes**:
  * `scripts/drone_physics_engine.gd` (Line 13: `@export var arm_length: float = 0.55`)
  * `scripts/garuda_model_test.gd` (Line 74: `lbl_arm.text = "ARM LENGTH: 0.55 m..."`)
  * `scenes/GARUDA_MODEL_TEST.tscn` (Line 88)
- **Resolution**: C++ manifest generator exports `arm_length_m: 0.550` and `motor_span_m: 1.100` to `build/generated/GARUDA_HL_01_MODEL_SPEC.json`. Python reads from JSON manifest; Godot displays metadata driven by C++.

### 2.2 Rotor Radial Angles (`22.5°`, `67.5°`, `112.5°`, `157.5°`, `202.5°`, `247.5°`, `292.5°`, `337.5°`)
- **Authoritative Definition**: `include/garuda/physics/motor_system.hpp` (Line 50: `22.5 + i * 45.0`) & `include/garuda/control/flight_controller.hpp` (Line 140)
- **Duplicated in Python**:
  * `scripts/pipeline/assemble_garuda_master.py` (Line 106)
  * `scripts/build_master_garuda_model.py` (Line 606)
- **Duplicated in GDScript**:
  * `scripts/drone_physics_engine.gd` (Line 23)
- **Resolution**: C++ calculates all 8 radial angles and 3D positions in `garuda::model::VehicleSpecification`, exported directly into the manifest.

### 2.3 Rotor Directions (CW vs CCW) & Local Axes
- **Authoritative Definition**: `include/garuda/physics/motor_system.hpp` (Line 52: `spin_dir = (i % 2 == 0) ? 1 : -1`)
- **Duplicated in GDScript**:
  * `scripts/garuda_model_test.gd` (Line 20: `const ROTOR_DIRS = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0]`)
- **Resolution**: Formally declare `enum class RotorDirection { CW, CCW };` in `include/garuda/model/component_types.hpp` with documented local $+Y$ shaft axis and coordinate mapping.

### 2.4 Gimbal Hierarchy & Offsets
- **Previous State**: Gimbal offset $[0, -0.088, -0.12]$ hard-coded in Python scripts.
- **Resolution**: Define `GarudaGimbal` and `GarudaPayloadMount` in C++ with joint origins (Yaw, Pitch, Roll) and angular limits ($-180^\circ..+180^\circ$, $-90^\circ..+30^\circ$, $-45^\circ..+45^\circ$).

---

## 3. Node Hierarchy & Coordinate System Definition

### 3.1 Authoritative C++ Coordinate System
- **Forward Axis**: $-Z$ (Aviation body frame convention matching Godot 4 camera forward)
- **Right Axis**: $+X$ (Starboard)
- **Up Axis**: $+Y$ (Vertical altitude normal)
- **Rotor Shaft Axis**: Local $+Y$
- **Gimbal Joint Axes**:
  * Yaw: Local $Y$-axis
  * Pitch: Local $X$-axis
  * Roll: Local $Z$-axis

### 3.2 Target GLB & Godot Node Hierarchy
```
GARUDA_HL_01 (Vehicle Reference Frame / CoM at (0,0,0))
│
├── AIRFRAME
│   └── BODY_MAIN (Faceted Aerodynamic Diamond Fuselage & Upper Deck)
│
├── ARMS
│   ├── ARM_01 (Root Socket + Clamp Collar + 0.55m Carbon Tube + Motor Flange)
│   ├── ARM_02
│   ├── ...
│   └── ARM_08
│
├── PROPULSION
│   ├── MOTOR_01
│   │   ├── MOTOR_CAN_01
│   │   ├── MOTOR_RING_01 (CNC Red Anodized Heat Sink)
│   │   ├── NAV_LED_01
│   │   └── ROTOR_01 (Origin at motor shaft center, local +Y axis)
│   │       ├── ROTOR_HUB_01
│   │       ├── BLADE_A (3D Cambered Aerofoil + White Tip Stripe)
│   │       └── BLADE_B (180° Y-Opposed 3D Cambered Aerofoil)
│   ├── ...
│   └── MOTOR_08
│
├── LANDING_GEAR
│   └── LANDING_GEAR_MAIN (Symmetrical Inverted A-Frame Carbon Struts & Ground Skids)
│
├── PAYLOAD
│   ├── PAYLOAD_MOUNT (Vibration Damper Plate & Silicone Isolators)
│   └── GIMBAL_YAW (Origin on Yaw axis)
│       └── GIMBAL_PITCH (Origin on Pitch axis)
│           └── GIMBAL_ROLL (Origin on Roll axis)
│               └── CAMERA_BODY (Quad-Lens Turret: 4K Day + FLIR + Laser + SWIR)
│
├── AVIONICS
│   └── ANTENNA_SYSTEM (4x Multi-Band Whip Antennas)
│
└── LIGHTS
    ├── AIRFRAME_CYAN_LIGHTS (Front Snout & Shoulder Accents)
    └── BEACON_BASE -> LIGHTS_BEACON (Pulsing Red Top Strobe)
```

---

## 4. File Classification

### 4.1 Files to Keep (Untouched / Preserved)
- `include/garuda/physics/*` (`battery_model.hpp`, `environment.hpp`, `ground_contact.hpp`, `motor_system.hpp`, `propulsion_system.hpp`, `rigid_body.hpp`)
- `include/garuda/config/quadrotor_config.hpp`
- `include/garuda/payload/*` (`payload_system.hpp`, `inspection_camera.hpp`, `payload_catalogue.hpp`)
- `include/garuda/control/*` (`flight_controller.hpp`)
- `include/garuda/capi.h`
- `src/garuda/capi.cpp`
- `src/garuda_sim.cpp`

### 4.2 Files to Create / Migrate
- **C++20 Model Specification**:
  * `include/garuda/model/component_types.hpp` [NEW]
  * `include/garuda/model/transform_3d.hpp` [NEW]
  * `include/garuda/model/vehicle_components.hpp` [NEW]
  * `include/garuda/model/vehicle_specification.hpp` [NEW]
  * `include/garuda/model/vehicle_model.hpp` [NEW]
  * `include/garuda/model/model_validator.hpp` [NEW]
  * `include/garuda/model/visual_bridge.hpp` [NEW]
- **C++20 Implementations**:
  * `src/garuda/model/vehicle_specification.cpp` [NEW]
  * `src/garuda/model/vehicle_model.cpp` [NEW]
  * `src/garuda/model/model_validator.cpp` [NEW]
  * `src/garuda/model/visual_bridge.cpp` [NEW]
- **C++ Manifest Generator & Validator Test Executable**:
  * `tools/generate_model_manifest.cpp` [NEW]
  * `tests/test_model_architecture.cpp` [NEW]
- **Generated Manifest**:
  * `build/generated/GARUDA_HL_01_MODEL_SPEC.json` [GENERATED]
- **Python Manifest-Driven Asset Compilers**:
  * `scripts/pipeline/geo_utils.py` [UPDATE: Manifest validation loader]
  * `scripts/pipeline/build_01_airframe.py` [UPDATE: Read dimensions from manifest]
  * `scripts/pipeline/build_02_arm_master.py` [UPDATE: Read arm length from manifest]
  * `scripts/pipeline/build_03_motor_master.py` [UPDATE: Read motor specs from manifest]
  * `scripts/pipeline/build_04_rotor_master.py` [UPDATE: Read rotor radius & directions from manifest]
  * `scripts/pipeline/build_05_landing_gear.py` [UPDATE: Read gear dimensions from manifest]
  * `scripts/pipeline/build_06_payload_gimbal.py` [UPDATE: Read gimbal limits from manifest]
  * `scripts/pipeline/assemble_garuda_master.py` [UPDATE: Driven 100% by manifest transforms]
  * `scripts/pipeline/validate_built_asset.py` [NEW: Post-build GLB verification against manifest]

### 4.3 Redundant / Stale Files (Eventually Deprecated)
- `scripts/build_realistic_garuda_model.py` (Replaced by modular manifest pipeline)
- `scripts/build_master_garuda_model.py` (Replaced by modular manifest pipeline)
- `scripts/generate_garuda_hl01_model.py` (Legacy procedural primitive script)

---

## 5. Migration Execution Order

```
PHASE A: C++ Authoritative Model Specification (`include/garuda/model/`)
   │
   ▼
PHASE B: C++ Mathematical Model Validator & Unit Tests (`tests/test_model_architecture.cpp`)
   │
   ▼
PHASE C: C++ Manifest Generator (`tools/generate_model_manifest.cpp` -> `GARUDA_HL_01_MODEL_SPEC.json`)
   │
   ▼
PHASE D: Refactor Python Pipeline to be 100% Manifest-Driven (`scripts/pipeline/`)
   │
   ▼
PHASE E: Asset Verification Suite (`validate_built_asset.py` checking GLB vs C++ Manifest)
   │
   ▼
PHASE F: Recompile Master & Component GLB Assets (`models/GARUDA_HL_01/`)
   │
   ▼
PHASE G: C++ Visual Bridge Implementation (`visual_bridge.hpp` / `visual_bridge.cpp`)
   │
   ▼
PHASE H: Godot Component & Model Test Validation (`GARUDA_MODEL_TEST.tscn`, `ROTOR_TEST.tscn`, etc.)
   │
   ▼
PHASE I: Final Regression & Architecture Documentation (`GARUDA_HL_01_MODEL_ARCHITECTURE.md`)
```

---

## 6. Validation & Regression Strategy

1. **C++20 Unit Tests (`test_model_architecture.exe`)**:
   - Verify motor span = $1.10\text{ m} \pm 10^{-4}$
   - Verify arm length = $0.55\text{ m} \pm 10^{-4}$
   - Verify rotor spacing = $45.0^\circ \pm 10^{-4}$
   - Verify first rotor angle = $22.5^\circ \pm 10^{-4}$
   - Verify 180° pairwise symmetry ($M_1 \leftrightarrow M_5$, $M_2 \leftrightarrow M_6$, $M_3 \leftrightarrow M_7$, $M_4 \leftrightarrow M_8$)
   - Verify alternating CW/CCW signs
   - Verify propeller clearance $> 10\text{ cm}$
2. **Schema & Asset Integrity Checks (`validate_built_asset.py`)**:
   - Validates that every node in `GARUDA_HL_01.glb` exactly matches the C++ manifest component IDs, origins, and positions.
3. **Interactive Visual & Animation Verification**:
   - Godot test scenes demonstrate smooth 0..3000 RPM rotor rotation around shaft local $+Y$, 3-axis gimbal articulation, and camera orbit.
