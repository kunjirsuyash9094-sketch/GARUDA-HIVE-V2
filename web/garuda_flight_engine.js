/**
 * GARUDA HIVE V2 — Rock-Solid Precision Flight Dynamics & Failure Injection Physics Engine
 * Features:
 * - Real Failure Injection Studio: Motor failure produces real-time thrust loss, asymmetric arm torque, violent tilt/tumble, altitude drop, and catastrophic ground crash
 * - Crash Physics: Ground impact triggers high-impact crash sound, structural tilt on terrain, disarms motors, and updates Failure HUD
 * - Rock-Solid Auto-Stabilization: Instant self-leveling (0.0° pitch/roll) & GPS position brake on stick release
 * - Autonomous Return-To-Launch (RTL): Level transit at 3m altitude, followed by 100% flat elevator descent onto the pad
 * - Dual-Scheme Keyboard & Interactive Mouse/Touch Flight Controller
 */

(function(global) {
    'use strict';

    const DEG2RAD = Math.PI / 180.0;
    const RAD2DEG = 180.0 / Math.PI;

    function clamp(v, lo, hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    class GarudaFlightEngine {
        constructor() {
            // Physical Parameters (Heavy-Lift Cinema Octocopter - 10.70kg)
            this.dryMass = 8.50; // kg
            this.payloadMass = 2.20; // kg (Pro Cinema Camera Gimbal)
            this.totalMass = 10.70; // kg
            this.gravity = 9.80665; // m/s^2
            this.groundClearance = 0.38; // Tall landing gear skid clearance

            // Home Position (Takeoff Helipad Center)
            this.homePosition = { x: 0.0, y: this.groundClearance, z: 0.0, yaw: 0.0 };

            // Flight State
            this.armed = false;
            this.flightMode = 'ALT_HOLD'; // ALT_HOLD, STABILIZE, POS_HOLD, AUTO_LAUNCH, AUTO_LAND
            this.isLaunching = false;
            this.isLanding = false;
            this.landingPhase = 'idle'; // 'transit_home', 'descent', 'touchdown'
            this.targetAltitude = 3.0; // meters AGL

            // 6-DOF State Variables
            this.position = { x: 0.0, y: this.groundClearance, z: 0.0 };
            this.velocity = { x: 0.0, y: 0.0, z: 0.0 };
            this.rotationEuler = { roll: 0.0, pitch: 0.0, yaw: 0.0 }; // in degrees
            this.angularVel = { roll: 0.0, pitch: 0.0, yaw: 0.0 }; // deg/s
            this.motorRpms = [0, 0, 0, 0, 0, 0, 0, 0];
            this.motorHealth = [0, 0, 0, 0, 0, 0, 0, 0]; // 0=Nominal, 1=Degraded, 2=Failed

            // Sensitivity & Agility
            this.throttleSensitivity = 1.0;
            this.pitchRollAgility = 1.0;
            this.yawAgility = 1.0;

            // Active Keyboard Key State Map
            this.activeKeys = {};
            this.onKeyChangeCallback = null;

            // 6S Battery (16,000mAh)
            this.batteryCapacityMah = 16000;
            this.batteryMahConsumed = 0;
            this.batterySoc = 1.0;
            this.batteryVoltage = 25.20;
            this.batteryCurrent = 0.0;
            this.batteryPower = 0.0;

            // Cascade PID Controllers (High Gain Rock-Solid Stabilization)
            this.altPid = { kp: 3.5, ki: 0.40, kd: 2.2, integral: 0, prevErr: 0 };
            this.rollPid = { kp: 12.0, ki: 0.20, kd: 3.2, integral: 0, prevErr: 0 };
            this.pitchPid = { kp: 12.0, ki: 0.20, kd: 3.2, integral: 0, prevErr: 0 };
            this.yawPid = { kp: 6.5, ki: 0.15, kd: 1.8, integral: 0, prevErr: 0 };

            // Simulation Timing
            this.simTick = 0;
            this.simTime = 0.0;

            this.setupKeyboardListeners();
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
                else if (e.key === 'l' || e.key === 'L') normCode = 'KeyL';
                else if (e.key === 'h' || e.key === 'H') normCode = 'KeyH';
                else if (e.key === 'r' || e.key === 'R') normCode = 'KeyR';
                else if (e.key === ' ') normCode = 'Space';

                if (this.activeKeys[normCode]) return;
                this.activeKeys[normCode] = true;
                this.activeKeys[code] = true;

                this.handleSpecialKeyPress(normCode);

                if (this.onKeyChangeCallback) {
                    this.onKeyChangeCallback(normCode, true);
                }
            };

            const handleKeyUp = (e) => {
                const code = e.code || e.key;
                let normCode = code;
                if (e.key === 'w' || e.key === 'W') normCode = 'KeyW';
                else if (e.key === 's' || e.key === 'S') normCode = 'KeyS';
                else if (e.key === 'a' || e.key === 'A') normCode = 'KeyA';
                else if (e.key === 'd' || e.key === 'D') normCode = 'KeyD';
                else if (e.key === 'l' || e.key === 'L') normCode = 'KeyL';
                else if (e.key === 'h' || e.key === 'H') normCode = 'KeyH';
                else if (e.key === 'r' || e.key === 'R') normCode = 'KeyR';
                else if (e.key === ' ') normCode = 'Space';

                this.activeKeys[normCode] = false;
                this.activeKeys[code] = false;

                if (this.onKeyChangeCallback) {
                    this.onKeyChangeCallback(normCode, false);
                }
            };

            window.addEventListener('keydown', handleKeyDown, true);
            window.addEventListener('keyup', handleKeyUp, true);
            document.addEventListener('keydown', handleKeyDown, true);
            document.addEventListener('keyup', handleKeyUp, true);
        }

        simulateKey(code, isPressed) {
            this.activeKeys[code] = isPressed;
            if (isPressed) {
                this.handleSpecialKeyPress(code);
            }
            if (this.onKeyChangeCallback) {
                this.onKeyChangeCallback(code, isPressed);
            }
        }

        handleSpecialKeyPress(code) {
            switch (code) {
                case 'Space':
                    if (!this.armed) {
                        this.launch();
                    } else {
                        if (this.position.y <= this.groundClearance + 0.1) {
                            this.launch();
                        } else {
                            this.hover();
                        }
                    }
                    break;
                case 'KeyL':
                    this.land();
                    break;
                case 'KeyH':
                    this.hover();
                    break;
                case 'KeyR':
                    this.resetToPad();
                    break;
                case 'KeyX':
                    this.disarm();
                    break;
                case 'Digit1':
                case 'Digit2':
                case 'Digit3':
                case 'Digit4':
                case 'Digit5':
                case 'Digit6':
                case 'Digit7':
                case 'Digit0':
                    const ptype = parseInt(code.replace('Digit', ''));
                    if (window.GarudaClient) {
                        window.GarudaClient.attachPayload(ptype);
                    }
                    break;
            }
        }

        setKeyChangeCallback(fn) {
            this.onKeyChangeCallback = fn;
        }

        setPayloadMass(massKg) {
            this.payloadMass = massKg;
            this.totalMass = this.dryMass + this.payloadMass;
        }

        arm() {
            this.armed = true;
            this.isLanding = false;
            this.isLaunching = false;
            if (window.GarudaAudio) window.GarudaAudio.playArmChime();
            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.arm();
            }
            console.log("[GARUDA Flight Engine] ARMED.");
        }

        disarm() {
            this.armed = false;
            this.isLaunching = false;
            this.isLanding = false;
            this.landingPhase = 'idle';
            for (let i = 0; i < 8; i++) this.motorRpms[i] = 0;
            if (window.GarudaAudio) window.GarudaAudio.playDisarmSound();
            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.disarm();
            }
            console.log("[GARUDA Flight Engine] DISARMED.");
        }

        launch() {
            this.restoreFailures();
            if (!this.armed) this.arm();
            this.isLaunching = true;
            this.isLanding = false;
            this.landingPhase = 'idle';
            this.flightMode = 'AUTO_LAUNCH';
            this.targetAltitude = 3.0;

            if (window.GarudaAudio) window.GarudaAudio.playLaunchSound();
            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.takeoff();
            }
            console.log("[GARUDA Flight Engine] 🚀 LAUNCH TAKEOFF -> Climbing to 3.0m Rock-Solid Hover");
        }

        land() {
            if (!this.armed) return;
            this.isLanding = true;
            this.isLaunching = false;
            this.flightMode = 'AUTO_LAND';
            this.landingPhase = 'transit_home';

            if (window.GarudaAudio) window.GarudaAudio.playArmChime();
            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.land();
            }
            console.log("[GARUDA Flight Engine] 🛬 LEVEL RETURN-TO-HOME & AUTO-LAND INITIATED.");
        }

        hover() {
            this.isLaunching = false;
            this.isLanding = false;
            this.landingPhase = 'idle';
            this.flightMode = 'POS_HOLD';
            this.targetAltitude = Math.max(1.0, this.position.y - this.groundClearance);
            this.velocity.x = 0;
            this.velocity.z = 0;
            this.rotationEuler.roll = 0;
            this.rotationEuler.pitch = 0;
            if (window.GarudaAudio) window.GarudaAudio.playUiClick();
            console.log(`[GARUDA Flight Engine] 🎯 ROCK-SOLID HOVER at ${this.targetAltitude.toFixed(2)}m.`);
        }

        resetToPad() {
            this.restoreFailures();
            this.disarm();
            this.position = { x: 0.0, y: this.groundClearance, z: 0.0 };
            this.velocity = { x: 0.0, y: 0.0, z: 0.0 };
            this.rotationEuler = { roll: 0.0, pitch: 0.0, yaw: 0.0 };
            this.angularVel = { roll: 0.0, pitch: 0.0, yaw: 0.0 };
            this.targetAltitude = 3.0;
            this.batteryMahConsumed = 0;
            this.batterySoc = 1.0;
            this.batteryVoltage = 25.20;
            this.landingPhase = 'idle';
            this.flightMode = 'DISARMED';

            this.altPid.integral = 0;
            this.rollPid.integral = 0;
            this.pitchPid.integral = 0;
            this.yawPid.integral = 0;

            if (window.GarudaClient && window.GarudaClient.octoModel) {
                window.GarudaClient.octoModel.restoreFromCrash();
            }
            if (window.GarudaAudio) window.GarudaAudio.playUiClick();
            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.resetSimulation();
            }
            console.log("[GARUDA Flight Engine] 🔄 RESET TO HELIPAD CENTER.");
        }

        failMotor(index) {
            if (index >= 0 && index < 8) {
                this.motorHealth[index] = 2;
                this.motorRpms[index] = 0;
                if (window.GarudaAudio) window.GarudaAudio.playMotorFailSound();
                if (window.GarudaClient && window.GarudaClient.connected) {
                    window.GarudaClient.failMotor(index);
                }
                console.warn(`[GARUDA Flight Engine] 💥 MOTOR #${index + 1} CATASTROPHIC FAILURE INJECTED!`);
            }
        }

        restoreFailures() {
            for (let i = 0; i < 8; i++) this.motorHealth[i] = 0;
            this.rotationEuler.roll = 0.0;
            this.rotationEuler.pitch = 0.0;
            this.flightMode = 'ALT_HOLD';
            if (window.GarudaClient && window.GarudaClient.octoModel) {
                window.GarudaClient.octoModel.restoreFromCrash();
            }
            if (window.GarudaAudio) window.GarudaAudio.playArmChime();
            if (window.GarudaClient && window.GarudaClient.connected) {
                window.GarudaClient.resetFailures();
            }
            console.log("[GARUDA Flight Engine] 🟢 All 8 Motors Restored to Nominal.");
        }

        processKeyboardInputs(dt) {
            const keys = this.activeKeys;

            let climbDemand = 0;
            if (keys['KeyW'] || keys['KeyI']) climbDemand += 1;
            if (keys['KeyS'] || keys['KeyK']) climbDemand -= 1;

            let yawDemand = 0;
            if (keys['KeyA'] || keys['KeyQ'] || keys['KeyJ']) yawDemand -= 1;
            if (keys['KeyD'] || keys['KeyE']) yawDemand += 1;

            let pitchDemand = 0;
            if (keys['ArrowUp']) pitchDemand += 1; // Move forward
            if (keys['ArrowDown']) pitchDemand -= 1; // Move backward

            let rollDemand = 0;
            if (keys['ArrowLeft']) rollDemand -= 1; // Strafe left
            if (keys['ArrowRight']) rollDemand += 1; // Strafe right

            if (pitchDemand !== 0 || rollDemand !== 0 || climbDemand !== 0 || yawDemand !== 0) {
                if (this.isLanding && climbDemand > 0) {
                    this.isLanding = false;
                    this.landingPhase = 'idle';
                    this.flightMode = 'ALT_HOLD';
                }
            }

            if (climbDemand !== 0) {
                this.isLanding = false;
                this.isLaunching = false;
                this.landingPhase = 'idle';
                this.targetAltitude = clamp(this.targetAltitude + climbDemand * 3.5 * dt * this.throttleSensitivity, 0.2, 80.0);
            }

            const targetRollDeg = rollDemand * 18.0 * this.pitchRollAgility;
            const targetPitchDeg = pitchDemand * 18.0 * this.pitchRollAgility;
            const targetYawRateDeg = yawDemand * 75.0 * this.yawAgility;

            return {
                targetRollDeg,
                targetPitchDeg,
                targetYawRateDeg,
                climbDemand,
                isManual: (pitchDemand !== 0 || rollDemand !== 0)
            };
        }

        step(dt) {
            dt = Math.min(dt, 0.05);

            const ctrl = this.processKeyboardInputs(dt);

            if (window.GarudaClient && window.GarudaClient.connected) {
                const rollRad = ctrl.targetRollDeg * DEG2RAD;
                const pitchRad = ctrl.targetPitchDeg * DEG2RAD;
                const yawRateRad = ctrl.targetYawRateDeg * DEG2RAD;
                let throttleNorm = 0.52;

                if (this.isLaunching) throttleNorm = 0.70;
                else if (this.isLanding) throttleNorm = 0.40;
                else throttleNorm = clamp(0.52 + (ctrl.climbDemand * 0.22), 0.1, 0.95);

                window.GarudaClient.setControl(rollRad, pitchRad, yawRateRad, throttleNorm);
                return;
            }

            // =========================================================================
            // Rock-Solid 400Hz 6-DOF Multirotor Flight & Stabilization Engine
            // =========================================================================
            this.simTick++;
            this.simTime += dt;

            const agl = Math.max(0, this.position.y - this.groundClearance);

            // Check for Motor Failures
            const failedCount = this.motorHealth.filter(h => h === 2).length;
            const isFailing = (failedCount > 0 && this.armed && agl > 0.02);

            // 1. Auto-Launch Routine
            if (this.isLaunching) {
                if (agl >= this.targetAltitude - 0.10) {
                    this.isLaunching = false;
                    this.flightMode = 'ALT_HOLD';
                }
            }

            // 2. High-Speed Level Return-To-Launch (RTL) Pipeline
            let navTargetRoll = ctrl.targetRollDeg;
            let navTargetPitch = ctrl.targetPitchDeg;
            let navTargetYawRate = ctrl.targetYawRateDeg;

            if (this.isLanding && !isFailing) {
                const dx = this.homePosition.x - this.position.x;
                const dz = this.homePosition.z - this.position.z;
                const distHome = Math.sqrt(dx * dx + dz * dz);
                const currentYawRad = this.rotationEuler.yaw * DEG2RAD;

                if (distHome > 0.15) {
                    // --- PHASE 1: LEVEL TRANSIT BACK TO HELIPAD ---
                    this.landingPhase = 'transit_home';
                    this.targetAltitude = Math.max(3.0, this.targetAltitude);

                    const transitSpeed = clamp(distHome * 2.0, 1.2, 6.0);
                    const dirX = dx / distHome;
                    const dirZ = dz / distHome;

                    const desVx = dirX * transitSpeed;
                    const desVz = dirZ * transitSpeed;

                    this.velocity.x += (desVx - this.velocity.x) * Math.min(1.0, 5.0 * dt);
                    this.velocity.z += (desVz - this.velocity.z) * Math.min(1.0, 5.0 * dt);

                    const bodyFwd = dirX * Math.cos(currentYawRad) + dirZ * Math.sin(currentYawRad);
                    const bodyRight = -dirX * Math.sin(currentYawRad) + dirZ * Math.cos(currentYawRad);

                    navTargetPitch = clamp(bodyFwd * 10.0, -10.0, 10.0);
                    navTargetRoll = clamp(bodyRight * 10.0, -10.0, 10.0);

                    const yawErr = (0.0 - this.rotationEuler.yaw + 540) % 360 - 180;
                    navTargetYawRate = clamp(yawErr * 2.0, -45.0, 45.0);

                } else {
                    // --- PHASE 2: FLAT VERTICAL ELEVATOR DESCENT ---
                    this.landingPhase = 'descent';
                    this.position.x = this.homePosition.x;
                    this.position.z = this.homePosition.z;
                    this.velocity.x = 0;
                    this.velocity.z = 0;
                    navTargetPitch = 0.0;
                    navTargetRoll = 0.0;

                    const yawErr = (0.0 - this.rotationEuler.yaw + 540) % 360 - 180;
                    navTargetYawRate = clamp(yawErr * 3.0, -30.0, 30.0);

                    const descentRate = (agl > 0.6) ? 0.95 : 0.40;
                    this.targetAltitude = Math.max(0.0, this.targetAltitude - descentRate * dt);

                    if (agl <= 0.03 && Math.abs(this.velocity.y) <= 0.25) {
                        this.disarm();
                        this.position.x = this.homePosition.x;
                        this.position.z = this.homePosition.z;
                        this.position.y = this.groundClearance;
                        this.velocity.x = 0;
                        this.velocity.y = 0;
                        this.velocity.z = 0;
                        this.rotationEuler.roll = 0.0;
                        this.rotationEuler.pitch = 0.0;
                        this.rotationEuler.yaw = 0.0;
                        if (window.GarudaAudio) window.GarudaAudio.playTouchdownSound();
                        console.log("[GARUDA Flight Engine] 🎯 TOUCHDOWN CONFIRMED: Helipad Centered & Secured.");
                    }
                }
            } else if (!ctrl.isManual && !isFailing) {
                // Instant Auto-Stabilization: neutral controls = level & stop
                navTargetPitch = 0.0;
                navTargetRoll = 0.0;
                this.velocity.x *= Math.exp(-dt * 7.0);
                this.velocity.z *= Math.exp(-dt * 7.0);
            }

            // 3. Dynamic Motor Failure Physics (Real Aerodynamic Thrust Loss & Tumble)
            if (isFailing) {
                this.flightMode = `🚨 MOTOR FAILURE (${failedCount}/8)`;
                const armAngles = [22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, 337.5];
                const spinDirs = [1, -1, 1, -1, 1, -1, 1, -1];

                let netRollTorque = 0;
                let netPitchTorque = 0;
                let netYawSpin = 0;

                for (let i = 0; i < 8; i++) {
                    if (this.motorHealth[i] === 2) {
                        const rad = armAngles[i] * DEG2RAD;
                        // Thrust loss tilts drone towards failed arm
                        netRollTorque += Math.cos(rad) * 60.0;
                        netPitchTorque += Math.sin(rad) * 60.0;
                        netYawSpin += spinDirs[i] * 140.0;
                    }
                }

                // Tilt and tumble out of control
                this.rotationEuler.roll += netRollTorque * dt;
                this.rotationEuler.pitch += netPitchTorque * dt;
                this.rotationEuler.yaw = (this.rotationEuler.yaw + netYawSpin * dt + 360) % 360;

                // Drone falls rapidly down
                const fallRate = (failedCount === 1) ? -4.5 : -9.8;
                this.velocity.y += fallRate * dt;

            } else if (this.armed) {
                // Smooth Attitude Integration (Rock-Solid Self-Leveling)
                this.rotationEuler.roll += (navTargetRoll - this.rotationEuler.roll) * Math.min(1.0, 14.0 * dt);
                this.rotationEuler.pitch += (navTargetPitch - this.rotationEuler.pitch) * Math.min(1.0, 14.0 * dt);
                this.rotationEuler.yaw = (this.rotationEuler.yaw + navTargetYawRate * dt + 360) % 360;
            } else {
                this.rotationEuler.roll *= Math.exp(-dt * 12.0);
                this.rotationEuler.pitch *= Math.exp(-dt * 12.0);
            }

            // 4. Altitude Control & Vertical Velocity Integration
            if (!isFailing) {
                const altError = this.targetAltitude - agl;
                const targetVy = clamp(altError * 3.2, -1.2, 2.5);

                if (this.armed) {
                    this.velocity.y += (targetVy - this.velocity.y) * Math.min(1.0, 8.0 * dt);
                } else {
                    this.velocity.y -= this.gravity * dt;
                }
            }

            // 5. Horizontal Velocity Integration from Manual Pitch & Roll
            if (this.armed && !this.isLanding && !isFailing) {
                const currentYawRad = this.rotationEuler.yaw * DEG2RAD;
                const rollRad = this.rotationEuler.roll * DEG2RAD;
                const pitchRad = this.rotationEuler.pitch * DEG2RAD;

                const targetFwdSpeed = (pitchRad * RAD2DEG) * 0.45;
                const targetRightSpeed = (rollRad * RAD2DEG) * 0.45;

                const desVx = targetFwdSpeed * Math.cos(currentYawRad) - targetRightSpeed * Math.sin(currentYawRad);
                const desVz = targetFwdSpeed * Math.sin(currentYawRad) + targetRightSpeed * Math.cos(currentYawRad);

                this.velocity.x += (desVx - this.velocity.x) * Math.min(1.0, 6.0 * dt);
                this.velocity.z += (desVz - this.velocity.z) * Math.min(1.0, 6.0 * dt);
            }

            // Position Integration
            this.position.x += this.velocity.x * dt;
            this.position.y += this.velocity.y * dt;
            this.position.z += this.velocity.z * dt;

            // Ground Clearance & Catastrophic Crash Detection
            if (this.position.y <= this.groundClearance) {
                const wasFallingFast = (this.velocity.y < -1.8);
                const hasFailedMotors = (this.motorHealth.some(h => h === 2));

                this.position.y = this.groundClearance;

                if ((wasFallingFast || hasFailedMotors) && this.armed) {
                    // CATASTROPHIC CRASH IMPACT
                    this.armed = false;
                    this.isLaunching = false;
                    this.isLanding = false;
                    this.velocity.x = 0;
                    this.velocity.y = 0;
                    this.velocity.z = 0;
                    for (let i = 0; i < 8; i++) this.motorRpms[i] = 0;

                    // Drone rests skewed/tilted on terrain
                    this.rotationEuler.roll = clamp(this.rotationEuler.roll, -40, 40);
                    if (Math.abs(this.rotationEuler.roll) < 15) this.rotationEuler.roll = 25;
                    this.rotationEuler.pitch = clamp(this.rotationEuler.pitch, -30, 30);

                    this.flightMode = '🚨 CRASHED [MOTOR FAILURE]';
                    if (window.GarudaAudio) window.GarudaAudio.playCrashSound();
                    if (window.GarudaClient && window.GarudaClient.octoModel) {
                        window.GarudaClient.octoModel.triggerCrashDestruction();
                    }
                    if (window.triggerCrashVFX) {
                        window.triggerCrashVFX(this.position);
                    }
                    console.error("[GARUDA Flight Engine] 💥 DRONE CRASHED INTO GROUND DUE TO MOTOR FAILURE!");
                } else {
                    if (this.velocity.y < 0) this.velocity.y = 0;
                    this.velocity.x = 0;
                    this.velocity.z = 0;
                    this.rotationEuler.roll = 0.0;
                    this.rotationEuler.pitch = 0.0;
                }
            }

            // 6. 8 Motors RPM Computation
            let baseRpm = 0;
            if (this.armed) {
                baseRpm = 2400 + Math.max(0, (this.velocity.y + 1.0) * 400);
            }
            for (let i = 0; i < 8; i++) {
                if (this.motorHealth[i] === 2) {
                    this.motorRpms[i] = 0;
                } else {
                    const lag = Math.exp(-dt / 0.03);
                    this.motorRpms[i] = this.motorRpms[i] * lag + baseRpm * (1.0 - lag);
                }
            }

            // 7. 6S LiPo Battery Ledger & Audio Synthesizer Update
            const totalThrustN = this.armed ? (this.totalMass * this.gravity) : 0;
            const avgRpm = this.motorRpms.reduce((a, b) => a + b, 0) / 8.0;
            this.batteryPower = (totalThrustN * 8.5) + (this.armed ? 26.0 : 4.0);
            this.batteryCurrent = this.batteryPower / Math.max(18.0, this.batteryVoltage);
            const mahConsumedPerSec = (this.batteryCurrent * 1000.0) / 3600.0;
            this.batteryMahConsumed += mahConsumedPerSec * dt;
            this.batterySoc = Math.max(0.0, 1.0 - (this.batteryMahConsumed / this.batteryCapacityMah));
            this.batteryVoltage = 25.20 - (5.60 * (1.0 - this.batterySoc)) - (this.batteryCurrent * 0.015);

            if (window.GarudaAudio) {
                const thrustRatio = totalThrustN / (this.totalMass * this.gravity * 1.5);
                window.GarudaAudio.updateMotorSound(avgRpm, thrustRatio);
            }

            this.broadcastTelemetry(totalThrustN);
        }

        broadcastTelemetry(totalThrustN) {
            const agl = Math.max(0, this.position.y - this.groundClearance);
            const groundSpeed = Math.sqrt(this.velocity.x * this.velocity.x + this.velocity.z * this.velocity.z);
            const twr = totalThrustN / (this.totalMass * this.gravity);

            const cr = Math.cos(this.rotationEuler.roll * DEG2RAD * 0.5);
            const sr = Math.sin(this.rotationEuler.roll * DEG2RAD * 0.5);
            const cp = Math.cos(this.rotationEuler.pitch * DEG2RAD * 0.5);
            const sp = Math.sin(this.rotationEuler.pitch * DEG2RAD * 0.5);
            const cy = Math.cos(this.rotationEuler.yaw * DEG2RAD * 0.5);
            const sy = Math.sin(this.rotationEuler.yaw * DEG2RAD * 0.5);

            const quat = {
                w: cr * cp * cy + sr * sp * sy,
                x: sr * cp * cy - cr * sp * sy,
                y: cr * sp * cy + sr * cp * sy,
                z: cr * cp * sy - sr * sp * cy
            };

            let statusText = this.flightMode;
            if (this.isLanding) statusText = `RTL [${this.landingPhase.toUpperCase()}]`;
            else if (this.isLaunching) statusText = 'AUTO_LAUNCH';

            const snapshot = {
                tick: this.simTick,
                time: this.simTime,
                drones: [{
                    drone_id: "GARUDA-HL-01",
                    position: { x: this.position.x, y: this.position.y, z: this.position.z },
                    velocity: { x: this.velocity.x, y: this.velocity.y, z: this.velocity.z },
                    orientation: quat,
                    rpy_deg: { roll: this.rotationEuler.roll, pitch: this.rotationEuler.pitch, yaw: this.rotationEuler.yaw },
                    altitude: agl,
                    ground_speed: groundSpeed,
                    vertical_speed: this.velocity.y,
                    total_thrust: totalThrustN,
                    twr: twr,
                    motor_rpm: this.motorRpms,
                    motor_health: this.motorHealth,
                    motor_thrust: this.motorRpms.map(rpm => 0.00000185 * rpm * rpm),
                    motor_power: this.motorRpms.map(rpm => (rpm / 4800) * 180),
                    motor_temp: this.motorRpms.map(rpm => 26 + (rpm / 4800) * 35),
                    battery: {
                        soc: this.batterySoc,
                        voltage_terminal: this.batteryVoltage,
                        current_amps: this.batteryCurrent,
                        power_w: this.batteryPower,
                        cells: [
                            this.batteryVoltage / 6.0,
                            this.batteryVoltage / 6.0,
                            this.batteryVoltage / 6.0,
                            this.batteryVoltage / 6.0,
                            this.batteryVoltage / 6.0,
                            this.batteryVoltage / 6.0
                        ]
                    },
                    payload: {
                        type: window.GarudaClient ? window.GarudaClient.currentPayloadType || 1 : 1,
                        mass_kg: this.payloadMass,
                        attached: true,
                        power_w: 18,
                        gimbal_pitch_deg: -15 - this.rotationEuler.pitch,
                        gimbal_yaw_deg: 0,
                        gimbal_roll_deg: -this.rotationEuler.roll,
                        zoom_level: 1.0
                    },
                    sensors: [
                        { name: 'TRIPLE IMU (400Hz)', status: 'LOCKED', val: '400 Hz' },
                        { name: 'RTK GNSS DUAL', status: 'LOCKED', val: '24 Sats' },
                        { name: 'BARO ALTIMETER', status: 'LOCKED', val: `${agl.toFixed(2)}m` },
                        { name: 'MAGNETOMETER', status: 'LOCKED', val: `${this.rotationEuler.yaw.toFixed(0)}°` },
                        { name: '3D LIDAR BEAMS', status: 'LOCKED', val: `${Math.min(50, agl).toFixed(2)}m` },
                        { name: '4K OPTICAL RGB', status: 'LOCKED', val: '60 FPS' },
                        { name: 'THERMAL FLIR IR', status: 'LOCKED', val: '32.4°C' },
                        { name: 'SONAR RANGEFINDER', status: 'LOCKED', val: `${agl.toFixed(2)}m` }
                    ],
                    armed: this.armed,
                    flight_mode: statusText
                }]
            };

            if (window.GarudaClient) {
                window.GarudaClient.updateSceneAndHUD(snapshot);
            }
        }
    }

    global.GarudaFlight = new GarudaFlightEngine();
})(typeof window !== 'undefined' ? window : this);
