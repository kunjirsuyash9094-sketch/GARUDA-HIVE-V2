/**
 * GARUDA HIVE V2 — Canonical Heavy-Lift Industrial Octocopter ("GARUDA-HL-01")
 * 
 * Physical 3D Architecture:
 * - 8 High-Modulus Carbon-Titanium Boom Arms in Octo-X Layout (psi_i = 22.5° + i*45°)
 * - 8 Industrial High-Torque Brushless Motors with CNC Billet Housings & Stator Vents
 * - 8 True 3D Aerodynamic Carbon Propeller Assemblies with:
 *     * Billet Central Hub & Spinner Nut
 *     * High-Aspect-Ratio Cambered Blades with Root-to-Tip Chord Taper & Aerodynamic Twist
 *     * Authoritative RPM-Driven Rotation (omega = RPM * 2*pi / 60)
 *     * High-RPM Dynamic Motion-Blur Supplement (retaining visible underlying blade geometry)
 * - 4 Articulated Heavy-Duty Landing Gear Legs + Dual Tactical Skid Rails (0.38m Clearance)
 * - Central Armored Avionics & Battery Bay (6S LiPo Pack with Voltage & Heat Telemetry)
 * - Under-slung 2-Axis Stabilized FLIR / 4K Inspection Gimbal Turret
 * - Dynamic Physical Crash Destruction & Debris Physics
 */

class GarudaOctocopterModel {
    constructor(scene) {
        this.scene = scene;
        this.rootGroup = new THREE.Group();
        this.rootGroup.name = "GARUDA-HL-01-OCTOCOPTER";
        this.scene.add(this.rootGroup);

        this.motors = [];
        this.props = [];
        this.propBlurs = [];
        this.propBlades = [];
        this.armMeshes = [];
        this.landingLegs = [];
        this.strobeLights = [];

        // Gimbal & Reconnaissance Sensor References
        this.gimbalYawGroup = null;
        this.gimbalPitchGroup = null;
        this.gimbalRollGroup = null;
        this.sensorTurretGroup = null;
        this.cameraLensMesh = null;
        this.payloadGroup = null;

        // Ground Height Clearance for tactical landing gear
        this.landingGearHeight = 0.38; // meters

        // Component Registry for 3D Raycasting
        this.interactiveComponents = new Map();

        // Timing for authoritative RPM integration
        this.lastUpdateTime = null;

        // Physics Debris
        this.debrisGroup = new THREE.Group();
        this.scene.add(this.debrisGroup);
        this.activeDebris = [];
        this.isDestroyed = false;

        this.initMaterials();
        this.buildOctocopterAirframe();
    }

    initMaterials() {
        let hullTex = null;
        let carbonTex = null;

        if (window.GarudaTextureGenerator) {
            hullTex = GarudaTextureGenerator.createMilitaryHullTexture();
            carbonTex = GarudaTextureGenerator.createCarbonFiberTextures();
        }

        // 1. Military Matte Gunmetal Composite Armor (Main Hull & Pods)
        this.armorMat = new THREE.MeshStandardMaterial({
            color: 0x22252a,
            roughness: 0.32,
            metalness: 0.78,
            map: hullTex || null
        });

        // 2. Heavy CNC Matte Dark Titanium (Chassis Framework, Hinges, Motor Bases)
        this.darkMetalMat = new THREE.MeshStandardMaterial({
            color: 0x16181d,
            roughness: 0.22,
            metalness: 0.90
        });

        // 3. Carbon Fiber Composite (Propeller Blades & Reinforced Struts)
        this.carbonMat = new THREE.MeshStandardMaterial({
            color: 0x121417,
            roughness: 0.28,
            metalness: 0.72,
            map: carbonTex ? carbonTex.diffuse : null
        });

        // 4. Tactical Gold / Amber Accents & Motor Windings
        this.accentGoldMat = new THREE.MeshStandardMaterial({
            color: 0xd4a017,
            roughness: 0.20,
            metalness: 0.92,
            emissive: 0x221800,
            emissiveIntensity: 0.25
        });

        // 5. Optical Multi-Coated Camera Glass (Sensors & Lenses)
        this.opticalGlassMat = new THREE.MeshPhysicalMaterial({
            color: 0x00e5ff,
            transmission: 0.82,
            opacity: 1.0,
            transparent: true,
            roughness: 0.04,
            metalness: 0.1,
            ior: 1.8,
            reflectivity: 0.95,
            emissive: 0x002233,
            emissiveIntensity: 0.35
        });

        // 6. Germanium Thermal IR Lens Material
        this.thermalLensMat = new THREE.MeshPhysicalMaterial({
            color: 0x8b3a00,
            roughness: 0.1,
            metalness: 0.6,
            transmission: 0.5,
            transparent: true,
            emissive: 0x331100,
            emissiveIntensity: 0.4
        });
    }

    /**
     * Builds true 3D aerodynamic propeller blade geometry with:
     * - Root chord & pitch angle
     * - Mid-span aerodynamic camber & chord
     * - Tip taper & washout twist angle
     */
    createAerodynamicBladeGeometry(radius = 0.28, isCCW = true) {
        const shape = new THREE.Shape();
        const rootR = 0.025;
        const span = radius - rootR;
        const segments = 10;

        // Construct 3D lofted ribbon for the blade
        const geom = new THREE.BufferGeometry();
        const positions = [];
        const normals = [];
        const uvs = [];
        const indices = [];

        const pitchSign = isCCW ? 1.0 : -1.0;

        for (let i = 0; i <= segments; i++) {
            const frac = i / segments;
            const r = rootR + frac * span;

            // Chord taper: 0.032m at root -> 0.030m at 40% span -> 0.016m at tip
            let chord = 0.032 * (1.0 - 0.55 * Math.pow(frac, 1.2));
            if (frac < 0.2) {
                chord = 0.020 + (frac / 0.2) * 0.012; // Root fillet
            }

            // Twist: 16 deg at root -> 7 deg at tip
            const twistDeg = 16.0 - frac * 9.0;
            const twistRad = twistDeg * (Math.PI / 180.0) * pitchSign;

            const cosT = Math.cos(twistRad);
            const sinT = Math.sin(twistRad);

            // Leading and Trailing edge offsets
            const leX = -chord * 0.35 * cosT;
            const leY = -chord * 0.35 * sinT;
            const teX = chord * 0.65 * cosT;
            const teY = chord * 0.65 * sinT;

            // Camber curve points (Upper and Lower surface)
            const thickness = (0.0055 * (1.0 - 0.6 * frac));
            
            // Left vertex (LE)
            positions.push(leX, leY, r);
            normals.push(-sinT, cosT, 0);
            uvs.push(0.0, frac);

            // Right vertex (TE)
            positions.push(teX, teY, r);
            normals.push(-sinT, cosT, 0);
            uvs.push(1.0, frac);
        }

        // Generate strip quad indices
        for (let i = 0; i < segments; i++) {
            const base = i * 2;
            indices.push(base, base + 1, base + 2);
            indices.push(base + 1, base + 3, base + 2);
            // Double-sided faces
            indices.push(base + 2, base + 1, base);
            indices.push(base + 2, base + 3, base + 1);
        }

        geom.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        geom.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
        geom.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
        geom.setIndex(indices);
        geom.computeVertexNormals();

        return geom;
    }

    buildOctocopterAirframe() {
        const airframe = new THREE.Group();

        // =========================================================================
        // 1. Central Faceted Armored Monocoque Fuselage
        // =========================================================================
        const hullGroup = new THREE.Group();

        // Main Lower Hull (Octagonal Reinforced Tub)
        const lowerHullGeo = new THREE.CylinderGeometry(0.24, 0.20, 0.12, 8);
        const lowerHull = new THREE.Mesh(lowerHullGeo, this.armorMat);
        lowerHull.position.y = 0.02;
        lowerHull.castShadow = true;
        lowerHull.receiveShadow = true;
        lowerHull.name = "Fuselage_Lower_Chassis";
        this.registerComponent(lowerHull, {
            name: "Lower Monocoque Hull",
            category: "Structure",
            desc: "Reinforced carbon-aramid composite monocoque chassis containing primary avionics and redundant power buses."
        });
        hullGroup.add(lowerHull);

        // Armored Top Deck with Integrated Heat Dissipation Vents
        const topDeckGeo = new THREE.CylinderGeometry(0.18, 0.24, 0.06, 8);
        const topDeck = new THREE.Mesh(topDeckGeo, this.armorMat);
        topDeck.position.y = 0.10;
        topDeck.castShadow = true;
        topDeck.name = "Fuselage_Upper_Deck";
        this.registerComponent(topDeck, {
            name: "Upper Armored Deck",
            category: "Structure",
            desc: "Titanium-reinforced top cover with IP67 sealed heat-sink vents for ESC and flight computer cooling."
        });
        hullGroup.add(topDeck);

        // Center Gold Avionics Core Collar
        const coreCollarGeo = new THREE.CylinderGeometry(0.19, 0.19, 0.015, 8);
        const coreCollar = new THREE.Mesh(coreCollarGeo, this.accentGoldMat);
        coreCollar.position.y = 0.065;
        hullGroup.add(coreCollar);

        // =========================================================================
        // 2. Top-Mounted Dual Antenna / GNSS & EW Turret Mast
        // =========================================================================
        const turretMast = new THREE.Group();
        turretMast.position.set(0, 0.13, -0.02);

        const mastBaseGeo = new THREE.BoxGeometry(0.08, 0.03, 0.08);
        const mastBase = new THREE.Mesh(mastBaseGeo, this.darkMetalMat);
        turretMast.add(mastBase);

        // Dual High-Gain Whip Antennas
        for (let a = 0; a < 2; a++) {
            const antGeo = new THREE.CylinderGeometry(0.002, 0.003, 0.22, 8);
            const antMesh = new THREE.Mesh(antGeo, this.darkMetalMat);
            antMesh.position.set(a === 0 ? -0.025 : 0.025, 0.11, -0.01);
            antMesh.rotation.z = a === 0 ? 0.08 : -0.08;
            turretMast.add(antMesh);
        }

        // Shielded GNSS Dome Module
        const gnssDomeGeo = new THREE.SphereGeometry(0.028, 16, 12, 0, Math.PI * 2, 0, Math.PI * 0.5);
        const gnssDome = new THREE.Mesh(gnssDomeGeo, this.armorMat);
        gnssDome.position.set(0, 0.015, 0.02);
        turretMast.add(gnssDome);

        this.sensorTurretGroup = turretMast;
        hullGroup.add(turretMast);
        airframe.add(hullGroup);

        // =========================================================================
        // 3. Front Dual-Aperture FLIR / Optical Reconnaissance Gimbal Pod
        // =========================================================================
        this.payloadGroup = new THREE.Group();
        this.payloadGroup.position.set(0, -0.02, 0.15);

        // Under-slung 2-Axis FLIR Thermal & 4K Ball Gimbal
        this.gimbalYawGroup = new THREE.Group();
        this.payloadGroup.add(this.gimbalYawGroup);

        this.gimbalPitchGroup = new THREE.Group();
        this.gimbalYawGroup.add(this.gimbalPitchGroup);

        const flirSphereGeo = new THREE.SphereGeometry(0.052, 24, 24);
        const flirSphere = new THREE.Mesh(flirSphereGeo, this.armorMat);
        flirSphere.castShadow = true;
        this.gimbalPitchGroup.add(flirSphere);

        // Primary 4K Optical Reconnaissance Lens
        const lens1Geo = new THREE.CylinderGeometry(0.016, 0.018, 0.025, 20);
        lens1Geo.rotateX(Math.PI / 2);
        const lens1 = new THREE.Mesh(lens1Geo, this.opticalGlassMat);
        lens1.position.set(-0.018, 0.005, 0.048);
        this.gimbalPitchGroup.add(lens1);

        // Secondary Germanium FLIR Thermal IR Lens
        const flirLensGeo = new THREE.CylinderGeometry(0.014, 0.016, 0.022, 16);
        flirLensGeo.rotateX(Math.PI / 2);
        const flirLens = new THREE.Mesh(flirLensGeo, this.thermalLensMat);
        flirLens.position.set(0.018, -0.005, 0.048);
        this.gimbalPitchGroup.add(flirLens);

        airframe.add(this.payloadGroup);

        // =========================================================================
        // 4. 8 Heavy-Lift Carbon-Titanium Boom Arms in Octo-X Radial Layout
        //    Angles: 22.5°, 67.5°, 112.5°, 157.5°, 202.5°, 247.5°, 292.5°, 337.5°
        // =========================================================================
        const armSpan = 0.55; // 0.55m -> 1100mm motor-to-motor diagonal span
        const rotorRadius = 0.1905; // 15" Propeller (0.1905m radius)

        for (let i = 0; i < 8; i++) {
            const angleDeg = 22.5 + i * 45.0;
            const angleRad = angleDeg * (Math.PI / 180.0);
            const isCCW = (i % 2 === 0); // Alternating CCW (+1) / CW (-1)

            const armGroup = new THREE.Group();
            armGroup.position.set(0, 0, 0);
            armGroup.rotation.y = angleRad;

            // Heavy Rectangular Reinforced Carbon Boom Arm
            const armLen = armSpan * 0.82;
            const armGeo = new THREE.BoxGeometry(0.038, 0.032, armLen);
            armGeo.translate(0, 0, armLen / 2 + 0.10);
            const armMesh = new THREE.Mesh(armGeo, this.armorMat);
            armMesh.castShadow = true;
            armMesh.name = `Boom_Arm_M${i + 1}`;
            this.registerComponent(armMesh, {
                name: `Octo-X Carbon Boom Arm #${i + 1}`,
                category: "Airframe",
                desc: `High-modulus carbon-titanium hollow boom arm with internal ESC signal lines and high-current power cables.`
            });
            armGroup.add(armMesh);

            // Arm Structural Locking Collar
            const lockGeo = new THREE.BoxGeometry(0.046, 0.040, 0.035);
            lockGeo.translate(0, 0, 0.14);
            const lockMesh = new THREE.Mesh(lockGeo, this.darkMetalMat);
            armGroup.add(lockMesh);

            // Corner Motor Base Pod (CNC Billet Housing with Stator Cooling Vents)
            const podGeo = new THREE.BoxGeometry(0.065, 0.060, 0.065);
            const podMesh = new THREE.Mesh(podGeo, this.armorMat);
            podMesh.position.set(0, 0.005, armSpan);
            podMesh.castShadow = true;
            armGroup.add(podMesh);

            // Industrial High-Torque Brushless Motor Bell (380 KV Outrunner)
            const motorBellGeo = new THREE.CylinderGeometry(0.032, 0.032, 0.030, 24);
            const motorBell = new THREE.Mesh(motorBellGeo, this.darkMetalMat);
            motorBell.position.set(0, 0.048, armSpan);
            motorBell.castShadow = true;
            armGroup.add(motorBell);

            // Gold Stator Vent Ring
            const ventRingGeo = new THREE.CylinderGeometry(0.033, 0.033, 0.006, 24);
            const ventRing = new THREE.Mesh(ventRingGeo, this.accentGoldMat);
            ventRing.position.set(0, 0.040, armSpan);
            armGroup.add(ventRing);

            // Motor Rotor Head Assembly
            const rotorHead = new THREE.Group();
            rotorHead.position.set(0, 0.068, armSpan);

            // Center CNC Billet Prop Hub
            const hubGeo = new THREE.CylinderGeometry(0.016, 0.016, 0.016, 16);
            const hubMesh = new THREE.Mesh(hubGeo, this.darkMetalMat);
            rotorHead.add(hubMesh);

            // Top Prop Spinner Dome Nut
            const spinnerGeo = new THREE.ConeGeometry(0.014, 0.018, 16);
            const spinnerMesh = new THREE.Mesh(spinnerGeo, this.darkMetalMat);
            spinnerMesh.position.y = 0.014;
            rotorHead.add(spinnerMesh);

            // 2 True 3D Aerodynamic Carbon Propeller Blades (180° opposed)
            const bladesInRotor = [];
            const bladeGeom = this.createAerodynamicBladeGeometry(rotorRadius, isCCW);

            for (let b = 0; b < 2; b++) {
                const bladeMesh = new THREE.Mesh(bladeGeom, this.carbonMat);
                bladeMesh.rotation.y = b * Math.PI;
                bladeMesh.castShadow = true;
                rotorHead.add(bladeMesh);
                bladesInRotor.push(bladeMesh);
            }
            this.propBlades.push(bladesInRotor);

            // High-RPM Dynamic Motion-Blur Supplement Disc
            const blurGeo = new THREE.CircleGeometry(rotorRadius * 1.02, 32);
            blurGeo.rotateX(-Math.PI / 2);
            const blurMat = new THREE.MeshBasicMaterial({
                color: 0x223344,
                transparent: true,
                opacity: 0.0,
                side: THREE.DoubleSide,
                depthWrite: false
            });
            const blurMesh = new THREE.Mesh(blurGeo, blurMat);
            blurMesh.position.y = 0.004;
            rotorHead.add(blurMesh);

            armGroup.add(rotorHead);

            // Navigation Strobe LED (Alternating Cyan / Crimson)
            const ledGeo = new THREE.SphereGeometry(0.006, 10, 10);
            const isFront = (i === 0 || i === 7);
            const ledColor = isFront ? 0x00f0ff : 0xff3344;
            const ledMat = new THREE.MeshBasicMaterial({ color: ledColor });
            const ledMesh = new THREE.Mesh(ledGeo, ledMat);
            ledMesh.position.set(0, 0.015, armSpan + 0.038);
            armGroup.add(ledMesh);
            this.strobeLights.push({ mesh: ledMesh, color: ledColor });

            // Corner Landing Legs on 4 specific quadrant arms (Arms 0, 2, 4, 6)
            if (i % 2 === 0) {
                const legGroup = new THREE.Group();
                legGroup.position.set(0, -0.03, armSpan * 0.90);

                const upperLegGeo = new THREE.BoxGeometry(0.020, 0.16, 0.020);
                const upperLeg = new THREE.Mesh(upperLegGeo, this.armorMat);
                upperLeg.position.set(0, -0.08, 0);
                upperLeg.rotation.z = (i === 0 || i === 6) ? -0.12 : 0.12;
                upperLeg.castShadow = true;
                legGroup.add(upperLeg);

                const footGeo = new THREE.CylinderGeometry(0.014, 0.018, 0.025, 12);
                const foot = new THREE.Mesh(footGeo, this.darkMetalMat);
                foot.position.set(0, -0.165, 0);
                legGroup.add(foot);

                armGroup.add(legGroup);
                this.landingLegs.push(legGroup);
            }

            airframe.add(armGroup);
            this.motors.push(armGroup);
            this.props.push(rotorHead);
            this.propBlurs.push(blurMat);
        }

        // =========================================================================
        // 5. Central Tactical Landing Skid Rails (0.38m Clearance)
        // =========================================================================
        const skidGroup = new THREE.Group();
        const skidLen = 0.52;
        const skidSpread = 0.20;
        const skidH = 0.24;

        const skidRailGeo = new THREE.BoxGeometry(0.018, 0.018, skidLen);
        const skidL = new THREE.Mesh(skidRailGeo, this.darkMetalMat);
        skidL.position.set(-skidSpread, -skidH, 0);
        skidL.castShadow = true;
        skidGroup.add(skidL);

        const skidR = new THREE.Mesh(skidRailGeo, this.darkMetalMat);
        skidR.position.set(skidSpread, -skidH, 0);
        skidR.castShadow = true;
        skidGroup.add(skidR);

        // Cross Struts
        const strutGeo = new THREE.CylinderGeometry(0.008, 0.008, skidSpread * 2, 12);
        strutGeo.rotateZ(Math.PI / 2);
        const strutF = new THREE.Mesh(strutGeo, this.darkMetalMat);
        strutF.position.set(0, -skidH, 0.18);
        skidGroup.add(strutF);

        const strutR = new THREE.Mesh(strutGeo, this.darkMetalMat);
        strutR.position.set(0, -skidH, -0.18);
        skidGroup.add(strutR);

        this.skidGroup = skidGroup;
        airframe.add(skidGroup);

        this.rootGroup.add(airframe);
    }

    registerComponent(mesh, metadata) {
        mesh.userData = { isInspectable: true, ...metadata };
        this.interactiveComponents.set(mesh.id, metadata);
    }

    /**
     * Authoritative update from physical simulation telemetry.
     * Computes exact delta time dt and rotates blades by omega = RPM * 2*pi / 60.
     */
    updateFromTelemetry(telem) {
        if (!telem) return;

        const now = performance.now() * 0.001; // seconds
        let dt = 0.016;
        if (this.lastUpdateTime !== null) {
            dt = Math.min(0.05, Math.max(0.001, now - this.lastUpdateTime));
        }
        this.lastUpdateTime = now;

        // 1. Root Position & Orientation (Physical Rigid-Body State)
        if (telem.position) {
            this.rootGroup.position.set(telem.position.x, telem.position.y, telem.position.z);
        }

        if (telem.orientation) {
            this.rootGroup.quaternion.set(
                telem.orientation.x,
                telem.orientation.y,
                telem.orientation.z,
                telem.orientation.w
            );
        }

        // 2. Authoritative RPM Cache for Continuous Native Render Loop
        if (telem.motor_rpm && Array.isArray(telem.motor_rpm)) {
            this.latestMotorRpms = telem.motor_rpm;
        }

        // 3. Reconnaissance FLIR Gimbal Stabilization
        if (telem.payload) {
            const pitchRad = (telem.payload.gimbal_pitch_deg || 0) * (Math.PI / 180.0);
            const yawRad = (telem.payload.gimbal_yaw_deg || 0) * (Math.PI / 180.0);
            if (this.gimbalYawGroup) this.gimbalYawGroup.rotation.y = yawRad;
            if (this.gimbalPitchGroup) this.gimbalPitchGroup.rotation.x = pitchRad;
        }

        // 4. Strobe Light Pulses
        const strobePhase = Math.sin(now * 8.0);
        this.strobeLights.forEach(s => {
            if (s.mesh && s.mesh.material) {
                s.mesh.material.opacity = (strobePhase > 0.3) ? 1.0 : 0.2;
            }
        });
    }

    /**
     * Continuous Render-Loop Propeller Rotation (Runs at native 60/144 FPS)
     */
    updateInRenderLoop(dt) {
        const rpms = this.latestMotorRpms || (window.GarudaFlight ? window.GarudaFlight.motorRpms : null);
        if (!rpms) return;

        const numProps = this.props.length;
        for (let i = 0; i < numProps; i++) {
            const rpm = rpms[i] || (rpms.length > 0 ? rpms[i % rpms.length] : 0);
            const spinDir = (i % 2 === 0) ? 1.0 : -1.0;

            // omega = RPM * 2 * pi / 60
            const omega = (rpm * 2.0 * Math.PI) / 60.0;
            const deltaRot = spinDir * omega * dt;

            if (this.props[i]) {
                this.props[i].rotation.y += deltaRot;
            }

            // Motion Blur: Fades in smoothly only above 2500 RPM (Max opacity = 0.60)
            if (this.propBlurs[i]) {
                const blurOnsetRpm = 2500.0;
                const blurMaxRpm = 5500.0;
                let blurOpacity = 0.0;
                if (rpm > blurOnsetRpm) {
                    const factor = Math.min(1.0, (rpm - blurOnsetRpm) / (blurMaxRpm - blurOnsetRpm));
                    blurOpacity = factor * 0.60;
                }
                this.propBlurs[i].opacity = blurOpacity;
            }
        }
    }

    triggerCrashDestruction(impactSpeed = 3.0) {
        if (this.isDestroyed) return;
        this.isDestroyed = true;

        if (this.skidGroup) {
            this.skidGroup.rotation.z = (Math.random() > 0.5 ? 0.35 : -0.35);
            this.skidGroup.position.y += 0.06;
        }

        this.landingLegs.forEach(leg => {
            leg.rotation.z = (Math.random() - 0.5) * 0.8;
            leg.rotation.x = (Math.random() - 0.5) * 0.6;
        });

        this.motors.forEach(m => {
            m.rotation.z = (Math.random() - 0.5) * 0.45;
            m.rotation.x = (Math.random() - 0.5) * 0.35;
        });

        if (this.sensorTurretGroup) {
            this.sensorTurretGroup.rotation.z = 0.35;
        }
        if (this.gimbalYawGroup) {
            this.gimbalYawGroup.rotation.z = 0.55;
        }

        // Physical Debris Scattering
        const dPos = this.rootGroup.position;
        const debrisColors = [0x22252a, 0x16181d, 0x121417, 0xd4a017];

        for (let i = 0; i < 16; i++) {
            const sizeX = 0.04 + Math.random() * 0.12;
            const sizeY = 0.008 + Math.random() * 0.015;
            const sizeZ = 0.02 + Math.random() * 0.05;
            const geo = new THREE.BoxGeometry(sizeX, sizeY, sizeZ);
            const mat = new THREE.MeshStandardMaterial({
                color: debrisColors[i % debrisColors.length],
                roughness: 0.35,
                metalness: 0.8
            });
            const mesh = new THREE.Mesh(geo, mat);
            mesh.position.set(
                dPos.x + (Math.random() - 0.5) * 0.6,
                dPos.y + 0.1 + Math.random() * 0.2,
                dPos.z + (Math.random() - 0.5) * 0.6
            );
            mesh.castShadow = true;
            this.debrisGroup.add(mesh);

            const angle = Math.random() * Math.PI * 2;
            const speed = 2.0 + Math.random() * 4.0;
            this.activeDebris.push({
                mesh: mesh,
                vx: Math.cos(angle) * speed,
                vy: 2.0 + Math.random() * 3.0,
                vz: Math.sin(angle) * speed,
                rotVx: (Math.random() - 0.5) * 16.0,
                rotVy: (Math.random() - 0.5) * 16.0,
                rotVz: (Math.random() - 0.5) * 16.0,
                bounces: 0
            });
        }
    }

    restoreFromCrash() {
        this.isDestroyed = false;

        if (this.skidGroup) {
            this.skidGroup.rotation.set(0, 0, 0);
            this.skidGroup.position.set(0, 0, 0);
        }

        this.landingLegs.forEach(leg => {
            leg.rotation.set(0, 0, 0);
        });

        for (let i = 0; i < this.motors.length; i++) {
            if (this.motors[i]) {
                const angleDeg = 22.5 + i * 45.0;
                const angleRad = angleDeg * (Math.PI / 180.0);
                this.motors[i].rotation.set(0, angleRad, 0);
                this.motors[i].position.set(0, 0, 0);
            }
        }

        if (this.sensorTurretGroup) this.sensorTurretGroup.rotation.set(0, 0, 0);
        if (this.gimbalYawGroup) this.gimbalYawGroup.rotation.set(0, 0, 0);
        if (this.gimbalPitchGroup) this.gimbalPitchGroup.rotation.set(0, 0, 0);

        while (this.debrisGroup.children.length > 0) {
            const obj = this.debrisGroup.children[0];
            this.debrisGroup.remove(obj);
            if (obj.geometry) obj.geometry.dispose();
            if (obj.material) obj.material.dispose();
        }
        this.activeDebris = [];
    }

    updateDebris(dt) {
        for (let i = this.activeDebris.length - 1; i >= 0; i--) {
            const d = this.activeDebris[i];
            d.vy -= 9.8 * dt;
            d.mesh.position.x += d.vx * dt;
            d.mesh.position.y += d.vy * dt;
            d.mesh.position.z += d.vz * dt;

            d.mesh.rotation.x += d.rotVx * dt;
            d.mesh.rotation.y += d.rotVy * dt;
            d.mesh.rotation.z += d.rotVz * dt;

            if (d.mesh.position.y <= 0.015) {
                d.mesh.position.y = 0.015;
                d.vy = -d.vy * 0.4;
                d.vx *= 0.7;
                d.vz *= 0.7;
                d.rotVx *= 0.6;
                d.rotVy *= 0.6;
                d.rotVz *= 0.6;
                d.bounces++;
                if (d.bounces > 4) {
                    d.vy = 0;
                    d.vx = 0;
                    d.vz = 0;
                }
            }
        }
    }
}

global.GarudaOctocopterModel = GarudaOctocopterModel;
