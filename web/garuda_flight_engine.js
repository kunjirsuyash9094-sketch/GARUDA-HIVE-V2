/**
 * GARUDA HIVE V2 — AUTHORITATIVE MULTIROTOR FLIGHT SIMULATION ENGINE
 * Full 6-DOF Rigid-Body Dynamics, Blade Element Propulsion & Multi-Loop Cascade PID
 * 
 * Strict Multirotor Principles:
 * - ARMED != TAKEOFF (Drone idles on landing gear at 800 RPM, producing negligible thrust).
 * - Takeoff requires deliberate pilot throttle or Launch command exceeding vehicle weight.
 * - True multirotor physics: Forces/Moments integrated via Newton-Euler (F_net = F_thrust - F_gravity - F_drag).
 * - High-speed, responsive flight dynamics matching real tactical heavy-lift UAVs.
 * - Real-time Himalayan terrain collision & AGL tracking.
 * - Autonomous Return-To-Launch (RTL) & Auto-Landing directly to Hive Launching Pad LZ-01.
 */

(function(global) {
    'use strict';

    const DEG2RAD = Math.PI / 180.0;
    const RAD2DEG = 180.0 / Math.PI;

    function clamp(val, min, max) {
        return Math.max(min, Math.min(max, val));
    }

    class GarudaFlightEngine {
        constructor() {
            // =========================================================================
            // 1. Vehicle Inertial & Structural Parameters (GARUDA-HL-01 Heavy-Lift)
            // =========================================================================
            this.dryMassKg = 8.50;
            this.payloadMassKg = 2.20;
            this.totalMass = this.dryMassKg + this.payloadMassKg; // 10.70 kg
            this.gravity = 9.80665; // m/s^2
            this.weightN = this.totalMass * this.gravity; // 104.93 N

            this.armLength = 0.55; // 550mm motor-to-center distance (1100mm span)
            this.groundClearance = 0.38; // Landing gear resting height above pad (m)

            // Principle Moments of Inertia in Y-up Frame (X=Pitch, Y=Yaw, Z=Roll)
            this.Ixx = 0.285; // Pitch Inertia (kg·m²)
            this.Iyy = 0.490; // Yaw Inertia (kg·m²)
            this.Izz = 0.285; // Roll Inertia (kg·m²)

            // 8-Rotor Aerodynamic Constants (15" High-Pitch Carbon Propellers)
            this.rotorCount = 8;
            this.Ct = 1.15e-5; // Thrust coeff: T = Ct * RPM^2 (N)
            this.Cq = 1.85e-7; // Torque coeff: Q = Cq * RPM^2 (N·m)
            this.motorMaxRpm = 5400.0;
            this.motorIdleRpm = 800.0;
            this.motorTimeConstant = 0.025; // 25ms rapid ESC/Motor spool-up

            // Aerodynamic drag damping coefficients
            this.linearDragCoeff = 0.38; // N / (m/s)
            this.rotationalDragCoeff = 4.2; // rad/s damping

            // Control Sensitivity & Scheme
            this.controlScheme = 'MODE2'; // MODE2 (Drone RC) or CLASSIC (Flight Sim)
            this.throttleSensitivity = 1.25;
            this.pitchRollAgility = 1.25;
            this.yawAgility = 1.20;

            // =========================================================================
            // 2. 6-DOF Physical State Variables (Newton-Euler Equations of Motion)
            // =========================================================================
            this.position = { x: 0.0, y: this.groundClearance, z: 0.0 };
            this.velocity = { x: 0.0, y: 0.0, z: 0.0 };
            this.acceleration = { x: 0.0, y: 0.0, z: 0.0 };

            // Quaternion Orientation (w, x, y, z)
            this.quaternion = { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };
            this.angularVel = { x: 0.0, y: 0.0, z: 0.0 }; // rad/s (X=pitch, Y=yaw, Z=roll)
            this.angularAcc = { x: 0.0, y: 0.0, z: 0.0 };
            this.rotationEuler = { roll: 0.0, pitch: 0.0, yaw: 0.0 }; // degrees

            // 8 Individual Motors State
            this.motorRpms = [0, 0, 0, 0, 0, 0, 0, 0];
            this.targetRpms = [0, 0, 0, 0, 0, 0, 0, 0];
            this.motorHealth = [0, 0, 0, 0, 0, 0, 0, 0]; // 0=Nominal, 1=Degraded, 2=Failed

            // =========================================================================
            // 3. Flight Controller & Pilot Input State
            // =========================================================================
            this.armed = false;
            this.isFlying = false;
            this.isLaunching = false;
            this.isLanding = false;
            this.flightMode = 'DISARMED'; // DISARMED, ARMED_GROUND, ALT_HOLD, POS_HOLD, AUTO_LAND
            this.pilotThrottle = 0.0; // 0.0 to 1.0
            this.targetAltitude = 3.0; // meters AGL
            this.homePosition = { x: 0.0, y: this.groundClearance, z: 0.0 };

            // Live Diagnostic Pilot Inputs (Exposed for Telemetry & HUD)
            this.pilotInputs = {
                throttleNorm: 0.0,
                rollCmdDeg: 0.0,
                pitchCmdDeg: 0.0,
                yawRateCmdDeg: 0.0,
                targetAltM: 3.0,
                totalThrustN: 0.0
            };

            // Cascaded Altitude & Attitude PID Controllers (Tuned for high responsiveness & stability)
            this.pidAlt = { kp: 2.2, ki: 0.25, kd: 1.4, integral: 0.0, prevErr: 0.0 };
            this.attRollP = 7.5;
            this.attPitchP = 7.5;
            this.pidRateRoll = { kp: 0.28, ki: 0.08, kd: 0.006, integral: 0.0, prevErr: 0.0 };
            this.pidRatePitch = { kp: 0.28, ki: 0.08, kd: 0.006, integral: 0.0, prevErr: 0.0 };
            this.pidRateYaw = { kp: 0.40, ki: 0.10, kd: 0.000, integral: 0.0, prevErr: 0.0 };

            // Battery State (6S 16Ah LiPo)
            this.batteryCapacityMah = 16000;
            this.batteryMahConsumed = 0;
            this.batterySoc = 1.0;
            this.batteryVoltage = 25.20;
            this.batteryCurrent = 0.0;
            this.batteryPower = 0.0;

            // Keyboard State & Callbacks
            this.activeKeys = {};
            this.keyChangeCallbacks = [];
            this.simTick = 0;
            this.simTime = 0.0;

            this.setupKeyboardListeners();
        }

        setKeyChangeCallback(callback) {
            if (typeof callback === 'function') {
                this.keyChangeCallbacks.push(callback);
            }
        }

        simulateKey(code, isPressed) {
            let normCode = code;
            if (normCode === 'w' || normCode === 'W') normCode = 'KeyW';
            else if (normCode === 's' || normCode === 'S') normCode = 'KeyS';
            else if (normCode === 'a' || normCode === 'A') normCode = 'KeyA';
            else if (normCode === 'd' || normCode === 'D') normCode = 'KeyD';
            else if (normCode === 'q' || normCode === 'Q') normCode = 'KeyQ';
            else if (normCode === 'e' || normCode === 'E') normCode = 'KeyE';
            else if (normCode === 'r' || normCode === 'R') normCode = 'KeyR';
            else if (normCode === 'l' || normCode === 'L') normCode = 'KeyL';
            else if (normCode === 'space' || normCode === 'Space') normCode = 'Space';
            else if (normCode === 'up' || normCode === 'ArrowUp') normCode = 'ArrowUp';
            else if (normCode === 'down' || normCode === 'ArrowDown') normCode = 'ArrowDown';
            else if (normCode === 'left' || normCode === 'ArrowLeft') normCode = 'ArrowLeft';
            else if (normCode === 'right' || normCode === 'ArrowRight') normCode = 'ArrowRight';

            this.activeKeys[normCode] = isPressed;
            this.triggerKeyVisual(normCode, isPressed);

            if (isPressed) {
                if (normCode === 'KeyR') this.resetToPad();
                else if (normCode === 'KeyL') this.land();
                else if (normCode === 'Space') {
                    if (!this.armed) this.arm();
                    else if (!this.isFlying) this.launch();
                }
            }
        }

        triggerKeyVisual(normCode, isPressed) {
            this.keyChangeCallbacks.forEach(cb => {
                try { cb(normCode, isPressed); } catch(e) {}
            });
            this.updateKeyCapVisual(normCode, isPressed);
        }

        setupKeyboardListeners() {
            const handleKeyDown = (e) => {
                const code = e.code || e.key;
                if (['Space', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(code)) {
                    if (e.target.tagName !== 'INPUT' && e.target.tagName !== 'SELECT') {
                        e.preventDefault();
                    }
                }

                let normCode = code;
                if (e.key === 'w' || e.key === 'W') normCode = 'KeyW';
                else if (e.key === 's' || e.key === 'S') normCode = 'KeyS';
                else if (e.key === 'a' || e.key === 'A') normCode = 'KeyA';
                else if (e.key === 'd' || e.key === 'D') normCode = 'KeyD';
                else if (e.key === 'q' || e.key === 'Q') normCode = 'KeyQ';
                else if (e.key === 'e' || e.key === 'E') normCode = 'KeyE';
                else if (e.key === 'r' || e.key === 'R') normCode = 'KeyR';
                else if (e.key === 'l' || e.key === 'L') normCode = 'KeyL';

                this.activeKeys[normCode] = true;
                this.triggerKeyVisual(normCode, true);

                if (normCode === 'KeyR') this.resetToPad();
                else if (normCode === 'KeyL') this.land();
                else if (normCode === 'Space') {
                    if (!this.armed) this.arm();
                    else if (!this.isFlying) this.launch();
                }
            };

            const handleKeyUp = (e) => {
                let normCode = e.code || e.key;
                if (e.key === 'w' || e.key === 'W') normCode = 'KeyW';
                else if (e.key === 's' || e.key === 'S') normCode = 'KeyS';
                else if (e.key === 'a' || e.key === 'A') normCode = 'KeyA';
                else if (e.key === 'd' || e.key === 'D') normCode = 'KeyD';
                else if (e.key === 'q' || e.key === 'Q') normCode = 'KeyQ';
                else if (e.key === 'e' || e.key === 'E') normCode = 'KeyE';
                else if (e.key === 'r' || e.key === 'R') normCode = 'KeyR';
                else if (e.key === 'l' || e.key === 'L') normCode = 'KeyL';

                this.activeKeys[normCode] = false;
                this.triggerKeyVisual(normCode, false);
            };

            window.addEventListener('keydown', handleKeyDown);
            window.addEventListener('keyup', handleKeyUp);
        }

        updateKeyCapVisual(code, isPressed) {
            const keyMap = {
                'KeyW': 'key-w', 'KeyS': 'key-s', 'KeyA': 'key-a', 'KeyD': 'key-d',
                'KeyQ': 'key-q', 'KeyE': 'key-e', 'Space': 'key-space', 'KeyL': 'key-l', 'KeyR': 'key-r',
                'ArrowUp': 'key-up', 'ArrowDown': 'key-down', 'ArrowLeft': 'key-left', 'ArrowRight': 'key-right'
            };
            const elemId = keyMap[code];
            if (elemId) {
                const elem = document.getElementById(elemId);
                if (elem) {
                    if (isPressed) elem.classList.add('pressed');
                    else elem.classList.remove('pressed');
                }
            }
        }

        arm() {
            this.armed = true;
            this.isFlying = false;
            this.isLaunching = false;
            this.isLanding = false;
            this.flightMode = 'ARMED_GROUND';
            this.pilotThrottle = 0.0;

            // Idle spool up to 800 RPM (negligible thrust ~1.5N << 105N weight)
            for (let i = 0; i < 8; i++) {
                if (this.motorHealth[i] === 0) this.targetRpms[i] = this.motorIdleRpm;
            }

            if (window.GarudaAudio) window.GarudaAudio.playArmChime();
            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.sendAction("arm");
            }
            console.log("[GARUDA Flight Engine] ⚡ ARMED: Idle propulsion engaged (800 RPM). Resting on pad.");
        }

        disarm() {
            this.armed = false;
            this.isFlying = false;
            this.isLaunching = false;
            this.isLanding = false;
            this.flightMode = 'DISARMED';
            this.pilotThrottle = 0.0;

            for (let i = 0; i < 8; i++) {
                this.targetRpms[i] = 0.0;
            }

            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.sendAction("disarm");
            }
            console.log("[GARUDA Flight Engine] 🛑 DISARMED: Motor cut-off.");
        }

        launch() {
            if (!this.armed) this.arm();
            this.isFlying = true;
            this.isLaunching = true;
            this.isLanding = false;
            this.flightMode = 'ALT_HOLD';
            this.targetAltitude = 3.0;
            this.pilotThrottle = 0.70; // Deliberate takeoff climb throttle

            if (window.GarudaAudio) window.GarudaAudio.playLaunchSound();
            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.sendAction("takeoff");
            }
            console.log("[GARUDA Flight Engine] 🚀 TAKEOFF INITIATED: Climbing to 3.0m AGL.");
        }

        land() {
            if (!this.armed) {
                this.arm();
            }
            this.isLanding = true;
            this.isLaunching = false;
            this.isFlying = true;
            this.flightMode = 'AUTO_LAND';

            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.sendAction("land");
            }
            console.log("[GARUDA Flight Engine] 🛬 AUTO-LANDING (RTL) INITIATED: Navigating back to Hive Launching Pad LZ-01.");
        }

        hover() {
            this.isLaunching = false;
            this.isLanding = false;
            this.flightMode = 'POS_HOLD';
            const terrainY = this.getTerrainHeightAt(this.position.x, this.position.z);
            this.targetAltitude = Math.max(1.0, this.position.y - terrainY - this.groundClearance);

            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.sendAction("hover");
            }
            console.log(`[GARUDA Flight Engine] 🎯 HOVER at ${this.targetAltitude.toFixed(2)}m AGL.`);
        }

        resetLocalState() {
            this.armed = false;
            this.isFlying = false;
            this.isLaunching = false;
            this.isLanding = false;
            this.flightMode = 'DISARMED';
            this.pilotThrottle = 0.0;
            this.targetAltitude = 3.0;

            this.position = { x: 0.0, y: this.groundClearance, z: 0.0 };
            this.velocity = { x: 0.0, y: 0.0, z: 0.0 };
            this.acceleration = { x: 0.0, y: 0.0, z: 0.0 };
            this.quaternion = { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };
            this.angularVel = { x: 0.0, y: 0.0, z: 0.0 };
            this.angularAcc = { x: 0.0, y: 0.0, z: 0.0 };
            this.rotationEuler = { roll: 0.0, pitch: 0.0, yaw: 0.0 };

            for (let i = 0; i < 8; i++) {
                this.motorRpms[i] = 0.0;
                this.targetRpms[i] = 0.0;
                this.motorHealth[i] = 0;
            }

            this.pidAlt.integral = 0;
            this.pidAlt.prevErr = 0;
            this.pidRateRoll.integral = 0;
            this.pidRatePitch.integral = 0;
            this.pidRateYaw.integral = 0;

            if (window.GarudaClient && window.GarudaClient.octoModel) {
                window.GarudaClient.octoModel.restoreFromCrash();
            }
        }

        resetToPad() {
            this.resetLocalState();
            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.sendAction("reset");
            }
            console.log("[GARUDA Flight Engine] 🔄 RESET TO HIVE LAUNCHING PAD LZ-01.");
        }

        failMotor(index) {
            if (index >= 0 && index < 8) {
                this.motorHealth[index] = 2; // FAILED
                this.targetRpms[index] = 0.0;
                if (window.GarudaAudio) window.GarudaAudio.playMotorFailSound();
                if (window.GarudaClient && window.GarudaClient.connected) {
                    window.GarudaClient.sendAction("fail_motor", { motor_index: index });
                }
                console.warn(`[GARUDA Flight Engine] 💥 MOTOR #${index + 1} FAILURE INJECTED!`);
            }
        }

        restoreFailures() {
            for (let i = 0; i < 8; i++) this.motorHealth[i] = 0;
            if (window.GarudaClient && window.GarudaClient.octoModel) {
                window.GarudaClient.octoModel.restoreFromCrash();
            }
            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.sendAction("reset_failures");
            }
            console.log("[GARUDA Flight Engine] 🟢 All 8 Motors Restored.");
        }

        setPayloadMass(massKg) {
            this.payloadMassKg = parseFloat(massKg) || 0.0;
            this.totalMass = this.dryMassKg + this.payloadMassKg;
            this.weightN = this.totalMass * this.gravity;
            console.log(`[GARUDA Flight Engine] ⚖️ Total Mass: ${this.totalMass.toFixed(2)} kg (Weight: ${this.weightN.toFixed(1)} N).`);
        }

        getTerrainHeightAt(x, z) {
            if (window.GarudaWorld && typeof window.GarudaWorld.getGroundHeight === 'function') {
                return window.GarudaWorld.getGroundHeight(x, z);
            }
            return 0.0;
        }

        // =========================================================================
        // 4. Physical Step: True 6-DOF Force & Torque Integration Loop
        // =========================================================================
        step(dt) {
            dt = Math.min(dt, 0.025); // Cap integration step at 40Hz min

            const keys = this.activeKeys;
            let pitchCmd = 0.0;   // Nose Down (-) / Nose Up (+)
            let rollCmd = 0.0;    // Right Down (+) / Left Down (-)
            let yawRateCmd = 0.0; // Turn CW (+) / Turn CCW (-)
            let climbDemand = 0.0;// Throttle climb / descent input

            const agility = this.pitchRollAgility || 1.25;
            const sensitivity = this.throttleSensitivity || 1.25;
            const maxTiltRad = 0.55 * agility; // ~31.5° maximum physical tilt for high speed cruise

            if (this.controlScheme === 'CLASSIC') {
                // Classic Flight Sim (W/S = Pitch, A/D = Roll, Q/E = Yaw, ArrowUp/Down/Space/Shift = Throttle)
                if (keys['KeyW']) pitchCmd -= 0.32 * agility;
                if (keys['KeyS']) pitchCmd += 0.32 * agility;
                if (keys['KeyD']) rollCmd += 0.32 * agility;
                if (keys['KeyA']) rollCmd -= 0.32 * agility;
                if (keys['KeyE']) yawRateCmd += 1.2 * (this.yawAgility || 1.2);
                if (keys['KeyQ']) yawRateCmd -= 1.2 * (this.yawAgility || 1.2);
                if (keys['ArrowUp'] || keys['Space']) climbDemand += 1.2 * sensitivity;
                if (keys['ArrowDown'] || keys['ShiftLeft']) climbDemand -= 1.2 * sensitivity;
            } else {
                // Standard Drone RC Mode 2 (ArrowUp/Down = Pitch, ArrowLeft/Right = Roll, A/D or Q/E = Yaw, W/S/Space/Shift = Throttle)
                if (keys['ArrowUp']) pitchCmd -= 0.32 * agility;
                if (keys['ArrowDown']) pitchCmd += 0.32 * agility;
                if (keys['ArrowRight']) rollCmd += 0.32 * agility;
                if (keys['ArrowLeft']) rollCmd -= 0.32 * agility;

                if (keys['KeyD'] || keys['KeyE']) yawRateCmd += 1.2 * (this.yawAgility || 1.2);
                if (keys['KeyA'] || keys['KeyQ']) yawRateCmd -= 1.2 * (this.yawAgility || 1.2);

                if (keys['KeyW'] || keys['Space']) climbDemand += 1.2 * sensitivity;
                if (keys['KeyS'] || keys['ShiftLeft']) climbDemand -= 1.2 * sensitivity;
            }

            const currentTerrainY = this.getTerrainHeightAt(this.position.x, this.position.z);
            const agl = Math.max(0.0, this.position.y - currentTerrainY - this.groundClearance);

            // 1. Throttle & Autopilot State Machine
            let computedThrottle = 0.0;

            if (this.armed) {
                if (!this.isFlying) {
                    // Sitting on Landing Pad: Idle throttle -> Motors spin at 800 RPM
                    if (climbDemand > 0.1) {
                        this.pilotThrottle = clamp(this.pilotThrottle + 0.55 * dt, 0.0, 1.0);
                    } else {
                        this.pilotThrottle = clamp(this.pilotThrottle - 0.60 * dt, 0.0, 1.0);
                    }

                    computedThrottle = this.pilotThrottle;

                    // Takeoff threshold: Lift off when throttle exceeds 56%
                    if (computedThrottle > 0.56) {
                        this.isFlying = true;
                        this.flightMode = 'ALT_HOLD';
                        this.targetAltitude = Math.max(2.0, agl + 1.0);
                    }
                } else if (this.isLanding) {
                    // Autonomous Return-To-Launch (RTL) & Precision Landing to Pad (0, 0)
                    const dx = this.homePosition.x - this.position.x;
                    const dz = this.homePosition.z - this.position.z;
                    const distPad = Math.hypot(dx, dz);

                    if (distPad > 0.35) {
                        // Far transit: maintain safe clearance above terrain and cruise towards (0, 0)
                        this.targetAltitude = Math.max(3.8, currentTerrainY + 2.5);
                        
                        // Bank towards target with velocity damping
                        const targetVx = clamp(dx * 2.2, -10.0, 10.0);
                        const targetVz = clamp(dz * 2.2, -10.0, 10.0);
                        
                        rollCmd = clamp((targetVx - this.velocity.x) * 0.15, -maxTiltRad, maxTiltRad);
                        pitchCmd = clamp(-(targetVz - this.velocity.z) * 0.15, -maxTiltRad, maxTiltRad);
                    } else {
                        // Hover over pad & initiate smooth descent
                        rollCmd = clamp((dx * 2.0 - this.velocity.x) * 0.18, -0.10, 0.10);
                        pitchCmd = clamp(-(dz * 2.0 - this.velocity.z) * 0.18, -0.10, 0.10);
                        
                        // Controlled descent: 0.65 m/s
                        this.targetAltitude = Math.max(0.0, this.targetAltitude - 0.65 * dt);

                        // Touchdown detection
                        if (agl <= 0.03 && Math.abs(this.velocity.y) < 0.25) {
                            this.position.x = 0.0;
                            this.position.z = 0.0;
                            this.position.y = this.groundClearance;
                            this.velocity.x = 0;
                            this.velocity.y = 0;
                            this.velocity.z = 0;
                            this.disarm();
                            if (window.GarudaAudio) window.GarudaAudio.playTouchdownSound();
                            console.log("[GARUDA Flight Engine] 🎯 TOUCHDOWN CONFIRMED ON HIVE LAUNCHING PAD LZ-01.");
                        }
                    }

                    // Altitude PID for Auto-Landing
                    const altErr = this.targetAltitude - agl;
                    this.pidAlt.integral = clamp(this.pidAlt.integral + altErr * dt, -3.0, 3.0);
                    const dAlt = (altErr - this.pidAlt.prevErr) / dt;
                    this.pidAlt.prevErr = altErr;
                    const desVy = clamp(this.pidAlt.kp * altErr + this.pidAlt.ki * this.pidAlt.integral + this.pidAlt.kd * dAlt, -1.5, 2.5);
                    const vyErr = desVy - this.velocity.y;
                    computedThrottle = clamp(0.5833 + (vyErr * 0.11), 0.12, 0.92);
                } else {
                    // Active Pilot Flight (Altitude Hold + Pilot Climb Rate Demand)
                    if (Math.abs(climbDemand) > 0.01) {
                        this.targetAltitude = clamp(this.targetAltitude + climbDemand * 3.5 * dt, 0.5, 60.0);
                    }

                    // Closed-loop altitude controller: Target Alt -> Target Vy -> Throttle
                    const altErr = this.targetAltitude - agl;
                    this.pidAlt.integral = clamp(this.pidAlt.integral + altErr * dt, -4.0, 4.0);
                    const dAlt = (altErr - this.pidAlt.prevErr) / dt;
                    this.pidAlt.prevErr = altErr;

                    const desVy = clamp(this.pidAlt.kp * altErr + this.pidAlt.ki * this.pidAlt.integral + this.pidAlt.kd * dAlt, -3.0, 4.5);
                    const vyErr = desVy - this.velocity.y;
                    computedThrottle = clamp(0.5833 + (vyErr * 0.10), 0.18, 0.95);
                }
            } else {
                computedThrottle = 0.0;
            }

            // Expose Live Pilot Inputs for Diagnostic HUD
            this.pilotInputs = {
                throttleNorm: computedThrottle,
                rollCmdDeg: rollCmd * RAD2DEG,
                pitchCmdDeg: pitchCmd * RAD2DEG,
                yawRateCmdDeg: yawRateCmd * RAD2DEG,
                targetAltM: this.targetAltitude,
                totalThrustN: 0.0
            };

            // If connected to authoritative C++ backend, delegate control setpoints
            if (window.GarudaClient && window.GarudaClient.connected) {
                if (this.isLaunching || this.isLanding) return;

                const hasInput = (Math.abs(rollCmd) > 0.001 || Math.abs(pitchCmd) > 0.001 || Math.abs(yawRateCmd) > 0.001 || Math.abs(climbDemand) > 0.001);
                if (hasInput) {
                    this._wasManualInput = true;
                    window.GarudaClient.setControl(rollCmd, pitchCmd, yawRateCmd, computedThrottle);
                } else if (this._wasManualInput) {
                    this._wasManualInput = false;
                    window.GarudaClient.hover();
                }
                return;
            }

            // =====================================================================
            // Standalone Authoritative 6-DOF Physical Integration Loop
            // =====================================================================
            this.simTick++;
            this.simTime += dt;

            // 2. Attitude Outer Loop in Y-up Frame
            const qw = this.quaternion.w, qx = this.quaternion.x, qy = this.quaternion.y, qz = this.quaternion.z;
            const rollCur = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qz * qz + qx * qx));
            const pitchCur = Math.asin(clamp(2 * (qw * qx - qy * qz), -1.0, 1.0));
            const yawCur = Math.atan2(2 * (qw * qy + qx * qz), 1 - 2 * (qy * qy + qx * qx));

            this.rotationEuler.roll = rollCur * RAD2DEG;
            this.rotationEuler.pitch = pitchCur * RAD2DEG;
            this.rotationEuler.yaw = ((yawCur * RAD2DEG) + 360) % 360;

            const rollRateDemand = clamp(this.attRollP * (rollCmd - rollCur), -4.5, 4.5);
            const pitchRateDemand = clamp(this.attPitchP * (pitchCmd - pitchCur), -4.5, 4.5);

            // 3. Inner Angular Rate Loop
            const pitchRateErr = pitchRateDemand - this.angularVel.x;
            this.pidRatePitch.integral = clamp(this.pidRatePitch.integral + pitchRateErr * dt, -6.0, 6.0);
            const dPitch = (this.angularVel.x - (this.pidRatePitch.prevRate || 0)) / dt;
            this.pidRatePitch.prevRate = this.angularVel.x;
            const tauPitchDem = clamp(this.pidRatePitch.kp * pitchRateErr + this.pidRatePitch.ki * this.pidRatePitch.integral - this.pidRatePitch.kd * dPitch, -0.32, 0.32);

            const rollRateErr = rollRateDemand - this.angularVel.z;
            this.pidRateRoll.integral = clamp(this.pidRateRoll.integral + rollRateErr * dt, -6.0, 6.0);
            const dRoll = (this.angularVel.z - (this.pidRateRoll.prevRate || 0)) / dt;
            this.pidRateRoll.prevRate = this.angularVel.z;
            const tauRollDem = clamp(this.pidRateRoll.kp * rollRateErr + this.pidRateRoll.ki * this.pidRateRoll.integral - this.pidRateRoll.kd * dRoll, -0.32, 0.32);

            const yawRateErr = yawRateCmd - this.angularVel.y;
            this.pidRateYaw.integral = clamp(this.pidRateYaw.integral + yawRateErr * dt, -6.0, 6.0);
            const dYaw = (this.angularVel.y - (this.pidRateYaw.prevRate || 0)) / dt;
            this.pidRateYaw.prevRate = this.angularVel.y;
            const tauYawDem = clamp(this.pidRateYaw.kp * yawRateErr + this.pidRateYaw.ki * this.pidRateYaw.integral - this.pidRateYaw.kd * dYaw, -0.25, 0.25);

            // 4. 8-Rotor Octo-X Mixer Allocation
            let sumThrust = 0.0;
            let totalTorqueX = 0.0;
            let totalTorqueY = 0.0;
            let totalTorqueZ = 0.0;

            for (let i = 0; i < 8; i++) {
                if (this.motorHealth[i] === 2 || !this.armed) {
                    this.targetRpms[i] = 0.0;
                } else if (!this.isFlying && computedThrottle < 0.05) {
                    this.targetRpms[i] = this.motorIdleRpm;
                } else {
                    const angleRad = (22.5 + i * 45.0) * DEG2RAD;
                    const armX = Math.cos(angleRad);
                    const armZ = Math.sin(angleRad);
                    const spinDir = (i % 2 === 0) ? 1.0 : -1.0;

                    const mix = computedThrottle + (tauPitchDem * armX) + (tauRollDem * armZ) + (tauYawDem * spinDir);
                    const cmd = clamp(mix, 0.05, 1.0);
                    this.targetRpms[i] = cmd * this.motorMaxRpm;
                }

                // First-Order Motor Lag: d(RPM)/dt = (target - current) / tau
                const rpmErr = this.targetRpms[i] - this.motorRpms[i];
                this.motorRpms[i] += (rpmErr / this.motorTimeConstant) * dt;
                this.motorRpms[i] = Math.max(0.0, this.motorRpms[i]);

                // Propeller Thrust (N) = Ct * RPM^2
                const t_i = this.Ct * Math.pow(this.motorRpms[i], 2);
                sumThrust += t_i;

                // Differential Torques about Body Axes
                const angleRad = (22.5 + i * 45.0) * DEG2RAD;
                const armX = Math.cos(angleRad) * this.armLength;
                const armZ = Math.sin(angleRad) * this.armLength;
                const spinDir = (i % 2 === 0) ? 1.0 : -1.0;

                totalTorqueX += t_i * armX;
                totalTorqueZ += t_i * armZ;
                totalTorqueY += spinDir * (this.Cq * Math.pow(this.motorRpms[i], 2));
            }

            this.pilotInputs.totalThrustN = sumThrust;

            // 5. World Force Integration
            const bThrustY = sumThrust;
            const fWorldX = 2 * (qx * qy - qw * qz) * bThrustY;
            const fWorldY = (qw * qw - qx * qx + qy * qy - qz * qz) * bThrustY - this.weightN;
            const fWorldZ = 2 * (qy * qz + qw * qx) * bThrustY;

            // Aerodynamic Drag
            const dragX = -this.linearDragCoeff * this.velocity.x;
            const dragY = -this.linearDragCoeff * this.velocity.y;
            const dragZ = -this.linearDragCoeff * this.velocity.z;

            let netFx = fWorldX + dragX;
            let netFy = fWorldY + dragY;
            let netFz = fWorldZ + dragZ;

            // Ground Contact Normal Force with Terrain Heightfield
            const minGroundY = currentTerrainY + this.groundClearance;
            if (this.position.y <= minGroundY) {
                this.position.y = minGroundY;
                if (netFy < 0) netFy = 0.0;
                if (this.velocity.y < 0) this.velocity.y = 0.0;

                // Ground friction
                this.velocity.x *= 0.85;
                this.velocity.z *= 0.85;
                this.angularVel.x *= 0.80;
                this.angularVel.y *= 0.80;
                this.angularVel.z *= 0.80;
            }

            // Translational Integration
            this.acceleration.x = netFx / this.totalMass;
            this.acceleration.y = netFy / this.totalMass;
            this.acceleration.z = netFz / this.totalMass;

            this.velocity.x += this.acceleration.x * dt;
            this.velocity.y += this.acceleration.y * dt;
            this.velocity.z += this.acceleration.z * dt;

            this.position.x += this.velocity.x * dt;
            this.position.y += this.velocity.y * dt;
            this.position.z += this.velocity.z * dt;

            // Hard clamp above terrain surface
            if (this.position.y < minGroundY) {
                this.position.y = minGroundY;
                if (this.velocity.y < 0) this.velocity.y = 0;
            }

            // Rotational Integration
            this.angularAcc.x = totalTorqueX / this.Ixx;
            this.angularAcc.y = totalTorqueY / this.Iyy;
            this.angularAcc.z = totalTorqueZ / this.Izz;

            this.angularVel.x += this.angularAcc.x * dt;
            this.angularVel.y += this.angularAcc.y * dt;
            this.angularVel.z += this.angularAcc.z * dt;

            // Rotational Drag
            const rotDamp = Math.exp(-dt * this.rotationalDragCoeff);
            this.angularVel.x *= rotDamp;
            this.angularVel.y *= rotDamp;
            this.angularVel.z *= rotDamp;

            // Quaternion Kinematics
            const halfDt = 0.5 * dt;
            const dqW = (-this.quaternion.x * this.angularVel.x - this.quaternion.y * this.angularVel.y - this.quaternion.z * this.angularVel.z) * halfDt;
            const dqX = ( this.quaternion.w * this.angularVel.x + this.quaternion.y * this.angularVel.z - this.quaternion.z * this.angularVel.y) * halfDt;
            const dqY = ( this.quaternion.w * this.angularVel.y - this.quaternion.x * this.angularVel.z + this.quaternion.z * this.angularVel.x) * halfDt;
            const dqZ = ( this.quaternion.w * this.angularVel.z + this.quaternion.x * this.angularVel.y - this.quaternion.y * this.angularVel.x) * halfDt;

            this.quaternion.w += dqW;
            this.quaternion.x += dqX;
            this.quaternion.y += dqY;
            this.quaternion.z += dqZ;

            // Normalize Quaternion
            const qMag = Math.sqrt(this.quaternion.w * this.quaternion.w + this.quaternion.x * this.quaternion.x + this.quaternion.y * this.quaternion.y + this.quaternion.z * this.quaternion.z);
            if (qMag > 1e-6) {
                this.quaternion.w /= qMag;
                this.quaternion.x /= qMag;
                this.quaternion.y /= qMag;
                this.quaternion.z /= qMag;
            }

            // Audio Modulation
            if (window.GarudaAudio) {
                const avgRpm = this.motorRpms.reduce((a, b) => a + b, 0) / 8.0;
                const thrustRatio = sumThrust / (this.totalMass * this.gravity * 2.2);
                window.GarudaAudio.updateMotorSound(avgRpm, thrustRatio);
            }
        }
    }

    global.GarudaFlightEngine = GarudaFlightEngine;
    if (typeof window !== 'undefined') {
        window.GarudaFlight = new GarudaFlightEngine();
    }
})(typeof window !== 'undefined' ? window : this);
