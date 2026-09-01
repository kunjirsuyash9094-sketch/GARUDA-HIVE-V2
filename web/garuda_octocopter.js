/**
 * GARUDA HIVE V2 — Tactical Military Reconnaissance UAV ("GARUDA-01")
 * Exact 1:1 High-Fidelity 3D Recreation matching the Reference Image:
 * - Armored faceted stealth fuselage with composite armor plates, vents, and hex bolts
 * - Top-mounted rectangular Electro-Optical / EW Sensor Turret with dual whip antennas
 * - Front dual-aperture optical reconnaissance turret + under-slung thermal FLIR pod
 * - 4 heavy rectangular reinforced carbon boom arms with corner motor nacelles
 * - 4 heavy-duty industrial brushless motors with high-aspect-ratio carbon folding blades
 * - 4 corner articulated landing gear legs + center tactical skid rails
 * - Procedural military PBR textures with "GARUDA-01", "RECONNAISSANCE UAV", & Eagle Wing Insignias
 * - Physics crash destruction & debris animation
 */

class GarudaOctocopterModel {
    constructor(scene) {
        this.scene = scene;
        this.rootGroup = new THREE.Group();
        this.rootGroup.name = "GARUDA-01-UAV";
        this.scene.add(this.rootGroup);

        this.motors = [];
        this.props = [];
        this.propBlurs = [];
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

        // Physics Debris
        this.debrisGroup = new THREE.Group();
        this.scene.add(this.debrisGroup);
        this.activeDebris = [];
        this.isDestroyed = false;

        this.initMaterials();
        this.buildTacticalDrone();
    }

    initMaterials() {
        let hullTex = null;
        let carbonTex = null;

        if (window.GarudaTextureGenerator) {
            hullTex = GarudaTextureGenerator.createMilitaryHullTexture();
            carbonTex = GarudaTextureGenerator.createCarbonFiberTextures();
        }

        // 1. Military Matte Gunmetal Composite Armor (Main Hull & Arms)
        this.armorMat = new THREE.MeshStandardMaterial({
            color: 0x22252a,
            roughness: 0.32,
            metalness: 0.78,
            map: hullTex || null
        });

        // 2. Heavy CNC Matte Dark Metal (Chassis Framework, Hinges, Motor Bases)
        this.darkMetalMat = new THREE.MeshStandardMaterial({
            color: 0x181a1f,
            roughness: 0.25,
            metalness: 0.88
        });

        // 3. Carbon Fiber Composite (Propeller Blades & Reinforced Struts)
        this.carbonMat = new THREE.MeshStandardMaterial({
            color: 0x15171b,
            roughness: 0.30,
            metalness: 0.70,
            map: carbonTex ? carbonTex.diffuse : null
        });

        // 4. Tactical Gold / Amber Accents & Connectors
        this.accentGoldMat = new THREE.MeshStandardMaterial({
            color: 0xd4a017,
            roughness: 0.22,
            metalness: 0.90,
            emissive: 0x221800,
            emissiveIntensity: 0.2
        });

        // 5. Optical Multi-Coated Camera Glass (Sensors & Thermal Lenses)
        this.opticalGlassMat = new THREE.MeshPhysicalMaterial({
            color: 0x00e5ff,
            transmission: 0.8,
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

    buildTacticalDrone() {
        const airframe = new THREE.Group();

        // =========================================================================
        // 1. Central Armored Fuselage (Faceted Chamfered Stealth Hull)
        // =========================================================================
        const hullLength = 0.56;
        const hullWidth = 0.34;
        const hullHeight = 0.18;

        // Main Armored Hull Box with Chamfered Nose
        const mainHullGeo = new THREE.BoxGeometry(hullWidth, hullHeight, hullLength);
        const mainHullMesh = new THREE.Mesh(mainHullGeo, this.armorMat);
        mainHullMesh.position.set(0, 0, 0);
        mainHullMesh.castShadow = true;
        mainHullMesh.name = "Armored_Fuselage";
        this.registerComponent(mainHullMesh, {
            name: "GARUDA-01 Armored Composite Monocoque Fuselage",
            category: "Fuselage",
            desc: "Ballistic titanium-reinforced carbon composite monocoque housing quad flight avionics, encrypted satellite datalink, and dual modular battery bays."
        });
        airframe.add(mainHullMesh);

        // Sloped Nose Armor Plate (Beveled Front Hood with Eagle Decal)
        const noseGeo = new THREE.CylinderGeometry(hullWidth * 0.48, hullWidth * 0.50, 0.14, 4);
        noseGeo.rotateY(Math.PI / 4);
        noseGeo.rotateX(Math.PI / 6);
        const noseMesh = new THREE.Mesh(noseGeo, this.armorMat);
        noseMesh.position.set(0, 0.02, hullLength * 0.46);
        noseMesh.castShadow = true;
        airframe.add(noseMesh);

        // Sloped Rear Engine Deck Armor
        const rearDeckGeo = new THREE.BoxGeometry(hullWidth * 0.92, 0.08, 0.14);
        const rearDeckMesh = new THREE.Mesh(rearDeckGeo, this.darkMetalMat);
        rearDeckMesh.position.set(0, 0.06, -hullLength * 0.42);
        rearDeckMesh.castShadow = true;
        airframe.add(rearDeckMesh);

        // Lateral Armor Sponson Pods (Left & Right)
        const sponsonGeo = new THREE.BoxGeometry(0.06, 0.10, hullLength * 0.65);
        const sponsonL = new THREE.Mesh(sponsonGeo, this.armorMat);
        sponsonL.position.set(-hullWidth * 0.54, -0.01, 0);
        sponsonL.castShadow = true;
        airframe.add(sponsonL);

        const sponsonR = new THREE.Mesh(sponsonGeo, this.armorMat);
        sponsonR.position.set(hullWidth * 0.54, -0.01, 0);
        sponsonR.castShadow = true;
        airframe.add(sponsonR);

        // Armor Plate Hex Bolts & Fasteners (4 corners)
        for (let x of [-hullWidth * 0.45, hullWidth * 0.45]) {
            for (let z of [-hullLength * 0.4, hullLength * 0.4]) {
                const boltGeo = new THREE.CylinderGeometry(0.008, 0.008, 0.012, 6);
                const bolt = new THREE.Mesh(boltGeo, this.darkMetalMat);
                bolt.position.set(x, hullHeight * 0.5 + 0.005, z);
                airframe.add(bolt);
            }
        }

        // =========================================================================
        // 2. Top Electro-Optical / EW Sensor Turret & Dual Antennas
        // =========================================================================
        this.sensorTurretGroup = new THREE.Group();
        this.sensorTurretGroup.position.set(0, hullHeight * 0.5 + 0.01, -0.02);

        // Vertical Rectangular Sensor Mast Body
        const mastGeo = new THREE.BoxGeometry(0.08, 0.18, 0.10);
        const mastMesh = new THREE.Mesh(mastGeo, this.armorMat);
        mastMesh.position.set(0, 0.09, 0);
        mastMesh.castShadow = true;
        mastMesh.name = "EW_Sensor_Tower";
        this.registerComponent(mastMesh, {
            name: "Top-Mounted Electro-Optical / EW Sensor Turret",
            category: "Avionics",
            desc: "360° situational awareness mast with SATCOM datalink, electronic warfare countermeasure suite, and high-altitude weather radar."
        });
        this.sensorTurretGroup.add(mastMesh);

        // Turret Top Beveled Cap
        const mastCapGeo = new THREE.BoxGeometry(0.086, 0.02, 0.106);
        const mastCap = new THREE.Mesh(mastCapGeo, this.darkMetalMat);
        mastCap.position.set(0, 0.185, 0);
        this.sensorTurretGroup.add(mastCap);

        // Turret Optical Aperture Window
        const mastLensGeo = new THREE.CylinderGeometry(0.012, 0.012, 0.01, 16);
        mastLensGeo.rotateX(Math.PI / 2);
        const mastLens = new THREE.Mesh(mastLensGeo, this.opticalGlassMat);
        mastLens.position.set(0, 0.13, 0.052);
        this.sensorTurretGroup.add(mastLens);

        // Primary Tall Whip Antenna (Left)
        const ant1Geo = new THREE.CylinderGeometry(0.002, 0.003, 0.16, 8);
        const ant1 = new THREE.Mesh(ant1Geo, this.darkMetalMat);
        ant1.position.set(-0.022, 0.27, 0.01);
        this.sensorTurretGroup.add(ant1);

        // Secondary Telemetry Antenna (Right)
        const ant2Geo = new THREE.CylinderGeometry(0.002, 0.003, 0.12, 8);
        const ant2 = new THREE.Mesh(ant2Geo, this.darkMetalMat);
        ant2.position.set(0.022, 0.25, -0.01);
        this.sensorTurretGroup.add(ant2);

        airframe.add(this.sensorTurretGroup);

        // =========================================================================
        // 3. Front Multispectral Reconnaissance Gimbal Turret (Dual Optic + FLIR)
        // =========================================================================
        this.payloadGroup = new THREE.Group();
        this.payloadGroup.position.set(0, -0.04, hullLength * 0.44);

        // Upper Reconnaissance Gimbal Casing
        const gimbalUpperGeo = new THREE.BoxGeometry(0.12, 0.08, 0.10);
        const gimbalUpper = new THREE.Mesh(gimbalUpperGeo, this.darkMetalMat);
        gimbalUpper.castShadow = true;
        this.payloadGroup.add(gimbalUpper);

        // Dual Optical Recon Lenses
        const lens1Geo = new THREE.CylinderGeometry(0.018, 0.020, 0.025, 20);
        lens1Geo.rotateX(Math.PI / 2);
        const lens1 = new THREE.Mesh(lens1Geo, this.opticalGlassMat);
        lens1.position.set(-0.026, 0, 0.055);
        this.payloadGroup.add(lens1);

        const lens2Geo = new THREE.CylinderGeometry(0.014, 0.016, 0.025, 20);
        lens2Geo.rotateX(Math.PI / 2);
        const lens2 = new THREE.Mesh(lens2Geo, this.opticalGlassMat);
        lens2.position.set(0.026, 0, 0.055);
        this.payloadGroup.add(lens2);

        // Under-slung 3-Axis FLIR Thermal Ball Gimbal
        this.gimbalYawGroup = new THREE.Group();
        this.gimbalYawGroup.position.set(0, -0.07, 0);
        this.payloadGroup.add(this.gimbalYawGroup);

        this.gimbalPitchGroup = new THREE.Group();
        this.gimbalYawGroup.add(this.gimbalPitchGroup);

        const flirSphereGeo = new THREE.SphereGeometry(0.045, 20, 20);
        const flirSphere = new THREE.Mesh(flirSphereGeo, this.armorMat);
        flirSphere.castShadow = true;
        this.gimbalPitchGroup.add(flirSphere);

        // FLIR Germanium Thermal Lens
        const flirLensGeo = new THREE.CylinderGeometry(0.016, 0.018, 0.015, 16);
        flirLensGeo.rotateX(Math.PI / 2);
        const flirLens = new THREE.Mesh(flirLensGeo, this.thermalLensMat);
        flirLens.position.set(0, 0, 0.042);
        this.gimbalPitchGroup.add(flirLens);

        airframe.add(this.payloadGroup);

        // =========================================================================
        // 4. 4 Heavy Rectangular Composite Boom Arms & Corner Motor Pods
        // =========================================================================
        const armSpan = 0.58; // Center to corner motor distance
        const armAngles = [
            Math.PI * 0.25,  // Front-Right (45°)
            Math.PI * 0.75,  // Rear-Right (135°)
            Math.PI * 1.25,  // Rear-Left (225°)
            Math.PI * 1.75   // Front-Left (315°)
        ];

        for (let i = 0; i < 4; i++) {
            const angle = armAngles[i];
            const armGroup = new THREE.Group();
            armGroup.position.set(0, 0, 0);
            armGroup.rotation.y = angle;

            // Heavy Rectangular Reinforced Arm Boom
            const armLen = armSpan * 0.85;
            const armGeo = new THREE.BoxGeometry(0.052, 0.046, armLen);
            armGeo.translate(0, 0, armLen / 2 + 0.12);
            const armMesh = new THREE.Mesh(armGeo, this.armorMat);
            armMesh.castShadow = true;
            armMesh.name = `Boom_Arm_${i + 1}`;
            this.registerComponent(armMesh, {
                name: `Tactical Reinforced Boom Arm #${i + 1}`,
                category: "Airframe",
                desc: `High-modulus carbon-titanium hollow rectangular arm with internal power distribution and hydraulic arm locks.`
            });
            armGroup.add(armMesh);

            // Hydraulic Arm Lock Collar
            const lockGeo = new THREE.BoxGeometry(0.062, 0.056, 0.04);
            lockGeo.translate(0, 0, 0.16);
            const lockMesh = new THREE.Mesh(lockGeo, this.darkMetalMat);
            armGroup.add(lockMesh);

            // Corner Motor Base Pod (Heavy Billet Housing with "01" Stencil)
            const podGeo = new THREE.BoxGeometry(0.075, 0.080, 0.075);
            const podMesh = new THREE.Mesh(podGeo, this.armorMat);
            podMesh.position.set(0, 0.01, armSpan);
            podMesh.castShadow = true;
            armGroup.add(podMesh);

            // Industrial High-Torque Brushless Motor Bell
            const motorBellGeo = new THREE.CylinderGeometry(0.038, 0.038, 0.036, 24);
            const motorBell = new THREE.Mesh(motorBellGeo, this.darkMetalMat);
            motorBell.position.set(0, 0.065, armSpan);
            motorBell.castShadow = true;
            armGroup.add(motorBell);

            // Motor Rotor Head & Folding Carbon Propeller Blades
            const rotorHead = new THREE.Group();
            rotorHead.position.set(0, 0.088, armSpan);

            // Center Prop Hub
            const hubGeo = new THREE.CylinderGeometry(0.016, 0.016, 0.014, 16);
            const hubMesh = new THREE.Mesh(hubGeo, this.darkMetalMat);
            rotorHead.add(hubMesh);

            // 2 Folding Carbon Propeller Blades
            const bladeLen = 0.28;
            for (let b = 0; b < 2; b++) {
                const bladeGeo = new THREE.BoxGeometry(0.034, 0.005, bladeLen);
                bladeGeo.translate(0, 0, bladeLen / 2);
                bladeGeo.rotateX(0.08); // Aerodynamic angle of attack
                const bladeMesh = new THREE.Mesh(bladeGeo, this.carbonMat);
                bladeMesh.rotation.y = b * Math.PI;
                bladeMesh.castShadow = true;
                rotorHead.add(bladeMesh);
            }

            // High-RPM Blur Disc
            const blurGeo = new THREE.CircleGeometry(bladeLen * 1.05, 32);
            blurGeo.rotateX(-Math.PI / 2);
            const blurMat = new THREE.MeshBasicMaterial({
                color: 0x334455,
                transparent: true,
                opacity: 0.0,
                side: THREE.DoubleSide
            });
            const blurMesh = new THREE.Mesh(blurGeo, blurMat);
            blurMesh.position.y = 0.005;
            rotorHead.add(blurMesh);

            armGroup.add(rotorHead);

            // =========================================================================
            // 5. Articulated Corner Landing Leg Pod (Under each motor)
            // =========================================================================
            const legGroup = new THREE.Group();
            legGroup.position.set(0, -0.04, armSpan);

            // Upper Articulated Strut
            const upperLegGeo = new THREE.BoxGeometry(0.024, 0.16, 0.024);
            const upperLeg = new THREE.Mesh(upperLegGeo, this.armorMat);
            upperLeg.position.set(0, -0.08, 0);
            upperLeg.rotation.z = (i % 2 === 0 ? -0.15 : 0.15);
            upperLeg.castShadow = true;
            legGroup.add(upperLeg);

            // Lower Shock Foot Pad
            const footGeo = new THREE.CylinderGeometry(0.016, 0.020, 0.03, 12);
            const foot = new THREE.Mesh(footGeo, this.darkMetalMat);
            foot.position.set(0, -0.17, 0);
            legGroup.add(foot);

            armGroup.add(legGroup);
            this.landingLegs.push(legGroup);

            // Navigation Strobe LED
            const ledGeo = new THREE.SphereGeometry(0.007, 10, 10);
            let ledColor = (i === 0 || i === 3) ? 0x00f0ff : 0xff3344;
            const ledMat = new THREE.MeshBasicMaterial({ color: ledColor });
            const ledMesh = new THREE.Mesh(ledGeo, ledMat);
            ledMesh.position.set(0, 0.02, armSpan + 0.042);
            armGroup.add(ledMesh);
            this.strobeLights.push({ mesh: ledMesh, color: ledColor });

            airframe.add(armGroup);
            this.motors.push(armGroup);
            this.props.push(rotorHead);
            this.propBlurs.push(blurMat);
        }

        // =========================================================================
        // 6. Central Tactical Landing Skid Rails
        // =========================================================================
        const skidGroup = new THREE.Group();
        const skidLen = 0.50;
        const skidSpread = 0.18;
        const skidH = 0.22;

        const skidRailGeo = new THREE.BoxGeometry(0.018, 0.018, skidLen);
        const skidL = new THREE.Mesh(skidRailGeo, this.darkMetalMat);
        skidL.position.set(-skidSpread, -skidH, 0);
        skidL.castShadow = true;
        skidGroup.add(skidL);

        const skidR = new THREE.Mesh(skidRailGeo, this.darkMetalMat);
        skidR.position.set(skidSpread, -skidH, 0);
        skidR.castShadow = true;
        skidGroup.add(skidR);

        this.skidGroup = skidGroup;
        airframe.add(skidGroup);

        this.rootGroup.add(airframe);
    }

    registerComponent(mesh, metadata) {
        mesh.userData = { isInspectable: true, ...metadata };
        this.interactiveComponents.set(mesh.id, metadata);
    }

    updateFromTelemetry(telem) {
        if (!telem) return;

        // 1. Root Position & Orientation
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

        // 2. Propeller Spin Animations
        if (telem.motor_rpm) {
            for (let i = 0; i < this.props.length; i++) {
                const rpm = telem.motor_rpm[i] || telem.motor_rpm[i % telem.motor_rpm.length] || 0;
                const spinDir = (i % 2 === 0) ? 1 : -1;
                const deltaRot = spinDir * (rpm / 60.0) * (Math.PI * 2) * 0.016;

                if (this.props[i]) {
                    this.props[i].rotation.y += deltaRot;
                }

                if (this.propBlurs[i]) {
                    const blurOpacity = Math.min(0.85, (rpm / 3500) * 0.85);
                    this.propBlurs[i].opacity = blurOpacity;
                }
            }
        }

        // 3. Reconnaissance FLIR Gimbal Stabilization
        if (telem.payload) {
            const pitchRad = (telem.payload.gimbal_pitch_deg || 0) * (Math.PI / 180.0);
            const yawRad = (telem.payload.gimbal_yaw_deg || 0) * (Math.PI / 180.0);
            if (this.gimbalYawGroup) this.gimbalYawGroup.rotation.y = yawRad;
            if (this.gimbalPitchGroup) this.gimbalPitchGroup.rotation.x = pitchRad;
        }
    }

    triggerCrashDestruction(impactSpeed = 3.0) {
        if (this.isDestroyed) return;
        this.isDestroyed = true;

        // 1. Collapsed Landing Legs & Central Skids
        if (this.skidGroup) {
            this.skidGroup.rotation.z = (Math.random() > 0.5 ? 0.35 : -0.35);
            this.skidGroup.position.y += 0.06;
        }

        this.landingLegs.forEach(leg => {
            leg.rotation.z = (Math.random() - 0.5) * 0.8;
            leg.rotation.x = (Math.random() - 0.5) * 0.6;
        });

        // 2. Skewed Boom Arms
        this.motors.forEach(m => {
            m.rotation.z = (Math.random() - 0.5) * 0.45;
            m.rotation.x = (Math.random() - 0.5) * 0.35;
        });

        // 3. Sensor Mast & Gimbal Dangle
        if (this.sensorTurretGroup) {
            this.sensorTurretGroup.rotation.z = 0.35;
        }
        if (this.gimbalYawGroup) {
            this.gimbalYawGroup.rotation.z = 0.55;
        }

        // 4. Physical Debris Scattering (Carbon Blades & Armor Shards)
        const dPos = this.rootGroup.position;
        const debrisColors = [0x22252a, 0x181a1f, 0x15171b, 0xd4a017];

        for (let i = 0; i < 12; i++) {
            const sizeX = 0.05 + Math.random() * 0.15;
            const sizeY = 0.01 + Math.random() * 0.02;
            const sizeZ = 0.03 + Math.random() * 0.06;
            const geo = new THREE.BoxGeometry(sizeX, sizeY, sizeZ);
            const mat = new THREE.MeshStandardMaterial({
                color: debrisColors[i % debrisColors.length],
                roughness: 0.35,
                metalness: 0.8
            });
            const mesh = new THREE.Mesh(geo, mat);
            mesh.position.set(
                dPos.x + (Math.random() - 0.5) * 0.5,
                dPos.y + 0.1 + Math.random() * 0.2,
                dPos.z + (Math.random() - 0.5) * 0.5
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

        // Restore Landing Gear
        if (this.skidGroup) {
            this.skidGroup.rotation.set(0, 0, 0);
            this.skidGroup.position.set(0, 0, 0);
        }

        this.landingLegs.forEach(leg => {
            leg.rotation.set(0, 0, 0);
        });

        // Restore Arms
        const armAngles = [Math.PI * 0.25, Math.PI * 0.75, Math.PI * 1.25, Math.PI * 1.75];
        for (let i = 0; i < this.motors.length; i++) {
            if (this.motors[i]) {
                this.motors[i].rotation.set(0, armAngles[i], 0);
                this.motors[i].position.set(0, 0, 0);
            }
        }

        // Restore Turret & Gimbal
        if (this.sensorTurretGroup) this.sensorTurretGroup.rotation.set(0, 0, 0);
        if (this.gimbalYawGroup) this.gimbalYawGroup.rotation.set(0, 0, 0);
        if (this.gimbalPitchGroup) this.gimbalPitchGroup.rotation.set(0, 0, 0);

        // Clear Debris
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
