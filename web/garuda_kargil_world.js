/**
 * GARUDA HIVE V2 — PHOTOREALISTIC KARGIL HIMALAYAN DIGITAL TWIN & DRONE HIVE BASE
 * 
 * Features:
 * 1. Vast 1.2km x 1.2km Procedural Himalayan Mountain Valley (Tiger Hill / Tololing inspired peaks)
 * 2. Winding Glacial Suru Mountain River with Animated Turquoise Flow & Riverbed Boulders
 * 3. Engineered National Highway (NH-1) Grounded directly on Terrain with Steel Truss Bridge & 3D Traffic
 * 4. Realistic 3D Vehicles (Military 6x6 Trucks, 4x4 Jeeps, SUVs) with Wheels Planted on Asphalt
 * 5. Underground Military Drone Hive Complex embedded in Cliff Face with Sliding Steel Blast Doors
 * 6. High-Visibility Tactical Launching Pad LZ-01 on Ground Level with 3D LED Beacon Towers
 * 7. Analytical Height API getGroundHeight(x, z) for 6-DOF Physics & Collision Detection
 * 8. Dynamic Sky & Lighting Engine with Day, Dawn, Night, and Mountain Weather Presets
 */

(function(global) {
    'use strict';

    class GarudaKargilWorld {
        constructor(scene, camera, renderer) {
            this.scene = scene;
            this.camera = camera;
            this.renderer = renderer;

            this.worldGroup = new THREE.Group();
            this.worldGroup.name = "GARUDA_KARGIL_ENVIRONMENT";
            this.scene.add(this.worldGroup);

            // Simulation Clock & Animation tracking
            this.time = 0;
            this.trafficVehicles = [];

            // Hive Blast Doors
            this.hiveGateOpen = false;
            this.hiveGateProgress = 0.0; // 0.0 (closed) to 1.0 (open)
            this.hiveGateLeft = null;
            this.hiveGateRight = null;

            // Radar & Waypoint references
            this.radarDome = null;
            this.radarDish = null;
            this.waypoints = [];
            this.waypointMarkers = [];
            this.patrolPathMesh = null;
            this.sensorFootprintMesh = null;

            // Lighting & Atmosphere
            this.currentWeather = 'NORMAL';
            this.currentTimeOfDay = 'DAY';
            this.dirLight = null;
            this.hemiLight = null;
            this.ambientLight = null;

            // Terrain Geometry Parameters (Vast Spacious Himalayan Valley)
            this.terrainSize = 1200; // 1.2km extent
            this.terrainSegments = 220;
            this.terrainMesh = null;
            this.riverMesh = null;

            this.initWorld();
        }

        // =========================================================================
        // 1. Procedural Heightfield Formula (Vast Himalayan Terrain & Valley Basin)
        // =========================================================================
        calculateRawElevation(x, z) {
            // Suru River Valley centerline with natural mountain meanders
            const valleyCenterX = Math.sin(z * 0.007) * 75.0 + Math.cos(z * 0.0035) * 35.0;
            const distFromValley = Math.abs(x - valleyCenterX);

            // 1. Central Drone Hive & Helipad Base Zone (Spacious flat basin around 0, 0)
            const distFromHive = Math.hypot(x - 0.0, z - 0.0);
            let basinFlatten = 1.0;
            if (distFromHive < 60.0) {
                basinFlatten = Math.max(0.0, Math.min(1.0, (distFromHive - 16.0) / 44.0));
            }

            // 2. Steep Himalayan Valley Slopes & Flanking Ridge Profiles
            const valleyProfile = Math.pow(Math.min(1.0, distFromValley / 320.0), 1.6) * 75.0;

            // 3. Multi-frequency Ridge, Cliff & Strata Noise (Craggy mountain topography)
            const ridge1 = Math.sin(x * 0.018 + z * 0.012) * Math.cos(z * 0.016 - x * 0.009) * 26.0;
            const ridge2 = Math.abs(Math.sin(x * 0.035 + z * 0.028)) * 16.0;
            const ridge3 = Math.sin(x * 0.07 - z * 0.05) * 6.5;
            const strataNoise = Math.sin(x * 0.012 + z * 0.045) * 9.0;

            // 4. Northern High Peaks (Tiger Hill / Tololing inspired peaks up to 92m sim scale)
            const northPeak = Math.max(0.0, (z + 200.0) / 300.0) * Math.max(0.0, (Math.abs(x) - 45.0) / 160.0) * 65.0;
            const southRidge = Math.max(0.0, (-z + 200.0) / 300.0) * 45.0;

            // 5. River Channel Carve (Carves a 14m wide, 2.4m deep natural channel)
            let riverCarve = 0.0;
            if (distFromValley < 16.0) {
                const riverFactor = 1.0 - (distFromValley / 16.0);
                riverCarve = Math.pow(riverFactor, 2.0) * 2.4;
            }

            let elev = (valleyProfile + ridge1 + ridge2 + ridge3 + strataNoise + northPeak + southRidge) * basinFlatten - riverCarve;

            // Mountain Cliff Face for Drone Hive Portal at x in [-25, 25], z < -16
            if (z < -14.0 && z > -75.0 && Math.abs(x) < 36.0) {
                const cliffBlend = Math.min(1.0, (Math.abs(z + 35.0) / 30.0));
                elev = Math.max(elev, 28.0 * (1.0 - cliffBlend));
            }

            // Helipad LZ-01 & Immediate Tarmac Apron (Flat at ground level 0.0)
            if (distFromHive < 16.0) {
                return 0.0;
            }

            return Math.max(0.0, elev);
        }

        getGroundHeight(x, z) {
            // Analytical height query for 6-DOF physics, terrain collision & vehicle positioning
            return this.calculateRawElevation(x, z);
        }

        // =========================================================================
        // 2. Build Himalayan Terrain Mesh with Altitude-Dependent PBR Materials
        // =========================================================================
        buildTerrain() {
            const geo = new THREE.PlaneGeometry(this.terrainSize, this.terrainSize, this.terrainSegments, this.terrainSegments);
            geo.rotateX(-Math.PI / 2);

            const pos = geo.attributes.position;
            const count = pos.count;
            const colors = new Float32Array(count * 3);

            // Realistic Himalayan Color Palette
            const colGrass = new THREE.Color(0x3e5238);     // Alpine scrub & valley floor
            const colAlluvial = new THREE.Color(0x544e43);  // Valley gravel & silt
            const colSlateRock = new THREE.Color(0x3c4048); // Dark Himalayan granite slate
            const colStrataRock = new THREE.Color(0x61584c);// Weathered mountain brown strata
            const colSnowShade = new THREE.Color(0xb8ccde); // Shaded alpine snow
            const colPureSnow = new THREE.Color(0xf6faff);  // Pure high-peak snow cap

            for (let i = 0; i < count; i++) {
                const px = pos.getX(i);
                const pz = pos.getZ(i);
                const py = this.calculateRawElevation(px, pz);
                pos.setY(i, py);

                // Altitude and slope-dependent procedural texturing
                const slopeNoise = (Math.sin(px * 0.15) + Math.cos(pz * 0.15)) * 0.5;
                const vColor = new THREE.Color();

                if (py < 3.2) {
                    // River bank & valley basin
                    vColor.copy(colAlluvial).lerp(colGrass, Math.max(0.0, (py - 0.5) / 2.7));
                } else if (py < 16.0) {
                    // Lower mountain slopes: Scrub transitioning to rock
                    const t = (py - 3.2) / 12.8;
                    vColor.copy(colGrass).lerp(colStrataRock, t);
                } else if (py < 34.0) {
                    // Mid elevation: Barren rocky cliffs & shale strata
                    const t = (py - 16.0) / 18.0;
                    vColor.copy(colStrataRock).lerp(colSlateRock, t + slopeNoise * 0.15);
                } else if (py < 52.0) {
                    // High elevation snow line: Patchy snow accumulation in rock crevices
                    const t = (py - 34.0) / 18.0;
                    vColor.copy(colSlateRock).lerp(colSnowShade, t);
                } else {
                    // Highest Himalayan peaks: Permanent deep snow caps
                    const t = Math.min(1.0, (py - 52.0) / 24.0);
                    vColor.copy(colSnowShade).lerp(colPureSnow, t);
                }

                colors[i * 3] = vColor.r;
                colors[i * 3 + 1] = vColor.g;
                colors[i * 3 + 2] = vColor.b;
            }

            geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
            geo.computeVertexNormals();

            let rockTex = null;
            if (window.GarudaTextureGenerator && window.GarudaTextureGenerator.createMountainRockTexture) {
                rockTex = window.GarudaTextureGenerator.createMountainRockTexture();
                rockTex.wrapS = THREE.RepeatWrapping;
                rockTex.wrapT = THREE.RepeatWrapping;
                rockTex.repeat.set(32, 32);
            }

            const mat = new THREE.MeshStandardMaterial({
                vertexColors: true,
                roughness: 0.88,
                metalness: 0.12,
                map: rockTex || null,
                flatShading: false
            });

            this.terrainMesh = new THREE.Mesh(geo, mat);
            this.terrainMesh.receiveShadow = true;
            this.worldGroup.add(this.terrainMesh);
        }

        // =========================================================================
        // 3. Suru River System with Animated Flow & Specular Reflection
        // =========================================================================
        buildSuruRiver() {
            const riverPoints = [];
            const riverWidth = 14.0;
            const segments = 180;

            for (let i = -segments / 2; i <= segments / 2; i++) {
                const z = (i / (segments / 2)) * (this.terrainSize * 0.48);
                const x = Math.sin(z * 0.007) * 75.0 + Math.cos(z * 0.0035) * 35.0;
                riverPoints.push(new THREE.Vector3(x, 0.45, z));
            }

            const riverCurve = new THREE.CatmullRomCurve3(riverPoints);
            const riverGeo = new THREE.BufferGeometry();
            const positions = [];
            const uvs = [];
            const indices = [];

            const numSteps = 160;
            for (let i = 0; i <= numSteps; i++) {
                const t = i / numSteps;
                const pt = riverCurve.getPoint(t);
                const tangent = riverCurve.getTangent(t);
                const normal = new THREE.Vector3(-tangent.z, 0, tangent.x).normalize();

                const leftX = pt.x + normal.x * (riverWidth * 0.5);
                const leftZ = pt.z + normal.z * (riverWidth * 0.5);
                const rightX = pt.x - normal.x * (riverWidth * 0.5);
                const rightZ = pt.z - normal.z * (riverWidth * 0.5);

                positions.push(leftX, pt.y, leftZ);
                positions.push(rightX, pt.y, rightZ);

                uvs.push(0.0, t * 50.0);
                uvs.push(1.0, t * 50.0);
            }

            for (let i = 0; i < numSteps; i++) {
                const base = i * 2;
                indices.push(base, base + 1, base + 2);
                indices.push(base + 1, base + 3, base + 2);
            }

            riverGeo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
            riverGeo.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
            riverGeo.setIndex(indices);
            riverGeo.computeVertexNormals();

            // Physical Water Shader Material with Glacial Turquoise Tint
            const riverMat = new THREE.MeshPhysicalMaterial({
                color: 0x1a6b8c,
                emissive: 0x041c28,
                roughness: 0.08,
                metalness: 0.15,
                transmission: 0.65,
                transparent: true,
                opacity: 0.88,
                ior: 1.333,
                reflectivity: 0.85
            });

            this.riverMesh = new THREE.Mesh(riverGeo, riverMat);
            this.riverMesh.receiveShadow = true;
            this.worldGroup.add(this.riverMesh);

            // River shoreline gravel boulders
            const rockGeo = new THREE.DodecahedronGeometry(0.8, 1);
            const rockMat = new THREE.MeshStandardMaterial({ color: 0x3c3e44, roughness: 0.92 });
            const riverRockInst = new THREE.InstancedMesh(rockGeo, rockMat, 120);
            const dummy = new THREE.Object3D();

            for (let i = 0; i < 120; i++) {
                const t = Math.random();
                const pt = riverCurve.getPoint(t);
                const side = Math.random() > 0.5 ? 1 : -1;
                const offset = (riverWidth * 0.52) + Math.random() * 3.5;
                dummy.position.set(pt.x + side * offset, 0.4 + Math.random() * 0.4, pt.z + (Math.random() - 0.5) * 6.0);
                dummy.scale.set(0.6 + Math.random() * 1.2, 0.4 + Math.random() * 0.8, 0.6 + Math.random() * 1.2);
                dummy.rotation.set(Math.random() * Math.PI, Math.random() * Math.PI, Math.random() * Math.PI);
                dummy.updateMatrix();
                riverRockInst.setMatrixAt(i, dummy.matrix);
            }
            this.worldGroup.add(riverRockInst);
        }

        // =========================================================================
        // 4. Engineered Mountain Highway (NH-1) Grounded Directly on Terrain
        // =========================================================================
        buildHighwayNetwork() {
            // Strategic National Highway NH-1 Waypoints passing through the Kargil valley
            const roadControlPoints = [
                new THREE.Vector3(-140, 0, -500),
                new THREE.Vector3(-95, 0, -380),
                new THREE.Vector3(-60, 0, -260),
                new THREE.Vector3(-35, 0, -150),
                new THREE.Vector3(-18, 0, -60),
                new THREE.Vector3(22, 0, 35),     // Curves gracefully beside Helipad LZ-01
                new THREE.Vector3(55, 0, 120),
                new THREE.Vector3(85, 0, 220),
                new THREE.Vector3(120, 0, 340),
                new THREE.Vector3(165, 0, 480)
            ];

            // Project all waypoints onto terrain surface + 0.15m baseline
            roadControlPoints.forEach(p => {
                p.y = this.getGroundHeight(p.x, p.z) + 0.15;
            });

            this.roadCurve = new THREE.CatmullRomCurve3(roadControlPoints);
            const roadWidth = 8.5;
            const segments = 240;

            const roadGeo = new THREE.BufferGeometry();
            const positions = [];
            const uvs = [];
            const indices = [];

            for (let i = 0; i <= segments; i++) {
                const t = i / segments;
                const pt = this.roadCurve.getPoint(t);
                const tangent = this.roadCurve.getTangent(t);
                const normal = new THREE.Vector3(-tangent.z, 0, tangent.x).normalize();

                // Compute exact ground height at center and lateral shoulders
                const leftX = pt.x + normal.x * (roadWidth * 0.5);
                const leftZ = pt.z + normal.z * (roadWidth * 0.5);
                const rightX = pt.x - normal.x * (roadWidth * 0.5);
                const rightZ = pt.z - normal.z * (roadWidth * 0.5);

                const groundLeftY = this.getGroundHeight(leftX, leftZ);
                const groundRightY = this.getGroundHeight(rightX, rightZ);
                const roadY = Math.max(pt.y, Math.max(groundLeftY, groundRightY) + 0.12);

                positions.push(leftX, roadY, leftZ);
                positions.push(rightX, roadY, rightZ);

                uvs.push(0.0, t * 65.0);
                uvs.push(1.0, t * 65.0);
            }

            for (let i = 0; i < segments; i++) {
                const base = i * 2;
                indices.push(base, base + 1, base + 2);
                indices.push(base + 1, base + 3, base + 2);
            }

            roadGeo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
            roadGeo.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
            roadGeo.setIndex(indices);
            roadGeo.computeVertexNormals();

            // Asphalt Texture with Double Yellow Center Lines & White Shoulders
            const canvas = document.createElement('canvas');
            canvas.width = 256; canvas.height = 512;
            const ctx = canvas.getContext('2d');
            ctx.fillStyle = '#22252a';
            ctx.fillRect(0, 0, 256, 512);

            // Subtle asphalt aggregate noise
            for (let y = 0; y < 512; y += 4) {
                for (let x = 0; x < 256; x += 4) {
                    if (Math.random() > 0.5) {
                        ctx.fillStyle = 'rgba(255,255,255,0.03)';
                        ctx.fillRect(x, y, 4, 4);
                    }
                }
            }

            // Solid white boundary shoulder lines
            ctx.fillStyle = '#e2e8f0';
            ctx.fillRect(16, 0, 8, 512);
            ctx.fillRect(232, 0, 8, 512);

            // Yellow dashed double center divider lines
            ctx.fillStyle = '#ffb800';
            for (let y = 20; y < 512; y += 80) {
                ctx.fillRect(122, y, 5, 48);
                ctx.fillRect(129, y, 5, 48);
            }

            const roadTex = new THREE.CanvasTexture(canvas);
            roadTex.wrapS = THREE.RepeatWrapping;
            roadTex.wrapT = THREE.RepeatWrapping;
            roadTex.repeat.set(1, 32);

            const roadMat = new THREE.MeshStandardMaterial({
                map: roadTex,
                roughness: 0.82,
                metalness: 0.12
            });

            const roadMesh = new THREE.Mesh(roadGeo, roadMat);
            roadMesh.receiveShadow = true;
            this.worldGroup.add(roadMesh);

            // Concrete Foundation Curbing (prevents any gaps between road and terrain)
            const foundationGeo = new THREE.BufferGeometry();
            const fPositions = [];
            const fIndices = [];

            for (let i = 0; i <= segments; i++) {
                const t = i / segments;
                const pt = this.roadCurve.getPoint(t);
                const tangent = this.roadCurve.getTangent(t);
                const normal = new THREE.Vector3(-tangent.z, 0, tangent.x).normalize();

                const leftX = pt.x + normal.x * (roadWidth * 0.52);
                const leftZ = pt.z + normal.z * (roadWidth * 0.52);
                const rightX = pt.x - normal.x * (roadWidth * 0.52);
                const rightZ = pt.z - normal.z * (roadWidth * 0.52);

                const roadY = this.getGroundHeight(pt.x, pt.z) + 0.12;
                const deepY = Math.min(this.getGroundHeight(leftX, leftZ), this.getGroundHeight(rightX, rightZ)) - 0.4;

                fPositions.push(leftX, roadY, leftZ);
                fPositions.push(leftX, deepY, leftZ);
                fPositions.push(rightX, roadY, rightZ);
                fPositions.push(rightX, deepY, rightZ);
            }

            for (let i = 0; i < segments; i++) {
                const base = i * 4;
                // Left foundation wall
                fIndices.push(base, base + 1, base + 4);
                fIndices.push(base + 1, base + 5, base + 4);
                // Right foundation wall
                fIndices.push(base + 2, base + 6, base + 3);
                fIndices.push(base + 3, base + 6, base + 7);
            }

            foundationGeo.setAttribute('position', new THREE.Float32BufferAttribute(fPositions, 3));
            foundationGeo.setIndex(fIndices);
            foundationGeo.computeVertexNormals();

            const foundationMat = new THREE.MeshStandardMaterial({ color: 0x383c44, roughness: 0.95 });
            const foundationMesh = new THREE.Mesh(foundationGeo, foundationMat);
            this.worldGroup.add(foundationMesh);

            // Highway Steel Truss Bridge over River crossing near (3, 2.8, 5)
            this.buildSteelTrussBridge(new THREE.Vector3(3, 2.8, 5), 24.0, roadWidth);

            // Metal Safety Guardrails along Highway Edges
            this.buildHighwayGuardrails(this.roadCurve, segments, roadWidth);

            // Spawn Grounded Mountain Highway Traffic
            this.spawnMountainTraffic(this.roadCurve);
        }

        buildSteelTrussBridge(center, length, width) {
            const bridgeGroup = new THREE.Group();
            bridgeGroup.position.copy(center);

            // Heavy Concrete Piers rooted into Riverbed
            const pierGeo = new THREE.BoxGeometry(width + 3.0, 8.0, 3.8);
            const pierMat = new THREE.MeshStandardMaterial({ color: 0x484c54, roughness: 0.92 });
            const pierL = new THREE.Mesh(pierGeo, pierMat);
            pierL.position.set(0, -4.0, -length * 0.45);
            const pierR = new THREE.Mesh(pierGeo, pierMat);
            pierR.position.set(0, -4.0, length * 0.45);
            bridgeGroup.add(pierL, pierR);

            // Steel Side Truss Girders (Military Green-Blue)
            const steelMat = new THREE.MeshStandardMaterial({ color: 0x24556e, metalness: 0.85, roughness: 0.28 });
            const trussGeo = new THREE.BoxGeometry(0.4, 3.2, length);
            const trussL = new THREE.Mesh(trussGeo, steelMat);
            trussL.position.set(width * 0.52, 1.4, 0);
            const trussR = new THREE.Mesh(trussGeo, steelMat);
            trussR.position.set(-width * 0.52, 1.4, 0);
            bridgeGroup.add(trussL, trussR);

            // Diagonal Truss Cross Braces
            const braceGeo = new THREE.BoxGeometry(0.2, 0.2, 4.2);
            for (let z = -length * 0.4; z <= length * 0.4; z += 4.5) {
                const bL = new THREE.Mesh(braceGeo, steelMat);
                bL.position.set(width * 0.52, 1.4, z);
                bL.rotation.x = Math.PI / 4;
                const bR = new THREE.Mesh(braceGeo, steelMat);
                bR.position.set(-width * 0.52, 1.4, z);
                bR.rotation.x = -Math.PI / 4;
                bridgeGroup.add(bL, bR);
            }

            this.worldGroup.add(bridgeGroup);
        }

        buildHighwayGuardrails(curve, segments, roadWidth) {
            const postGeo = new THREE.CylinderGeometry(0.05, 0.05, 0.85, 8);
            const metalMat = new THREE.MeshStandardMaterial({ color: 0xa0acba, metalness: 0.88, roughness: 0.25 });

            const instPosts = new THREE.InstancedMesh(postGeo, metalMat, segments * 2);
            const dummy = new THREE.Object3D();

            let idx = 0;
            for (let i = 0; i < segments; i += 2) {
                const t = i / segments;
                const pt = curve.getPoint(t);
                const tangent = curve.getTangent(t);
                const normal = new THREE.Vector3(-tangent.z, 0, tangent.x).normalize();
                const y = this.getGroundHeight(pt.x, pt.z) + 0.14;

                // Right Post
                dummy.position.set(pt.x + normal.x * (roadWidth * 0.52), y + 0.42, pt.z + normal.z * (roadWidth * 0.52));
                dummy.updateMatrix();
                instPosts.setMatrixAt(idx++, dummy.matrix);

                // Left Post
                dummy.position.set(pt.x - normal.x * (roadWidth * 0.52), y + 0.42, pt.z - normal.z * (roadWidth * 0.52));
                dummy.updateMatrix();
                instPosts.setMatrixAt(idx++, dummy.matrix);
            }
            this.worldGroup.add(instPosts);
        }

        // =========================================================================
        // 5. 3D Mountain Highway Vehicles (Planted on Asphalt, Realistic Driving)
        // =========================================================================
        spawnMountainTraffic(curve) {
            const numVehicles = 8;
            this.trafficVehicles = [];

            for (let i = 0; i < numVehicles; i++) {
                const vGroup = new THREE.Group();
                const isTruck = (i % 3 === 0);
                const isJeep = (i % 3 === 1);
                const isSUV = (i % 3 === 2);

                const wheelGeo = new THREE.CylinderGeometry(0.38, 0.38, 0.28, 16);
                wheelGeo.rotateZ(Math.PI / 2);
                const wheelMat = new THREE.MeshStandardMaterial({ color: 0x181a1f, roughness: 0.9 });

                if (isTruck) {
                    // Indian Army 6x6 Military Stallion Supply Truck
                    const bodyMat = new THREE.MeshStandardMaterial({ color: 0x3d4a36, roughness: 0.6, metalness: 0.4 });
                    const cabMat = new THREE.MeshStandardMaterial({ color: 0x34402e, roughness: 0.5, metalness: 0.5 });
                    const canopyMat = new THREE.MeshStandardMaterial({ color: 0x2e3828, roughness: 0.95 });

                    // Truck Chassis & Cabin
                    const cab = new THREE.Mesh(new THREE.BoxGeometry(2.1, 1.6, 2.2), cabMat);
                    cab.position.set(0, 1.4, -2.1);
                    vGroup.add(cab);

                    // Canvas Cargo Canopy
                    const cargo = new THREE.Mesh(new THREE.BoxGeometry(2.2, 1.8, 4.2), canopyMat);
                    cargo.position.set(0, 1.7, 1.2);
                    vGroup.add(cargo);

                    // 6 Heavy Off-road Wheels
                    const wheelZ = [-2.1, 0.6, 2.2];
                    wheelZ.forEach(z => {
                        const wL = new THREE.Mesh(wheelGeo, wheelMat);
                        wL.position.set(1.15, 0.38, z);
                        const wR = new THREE.Mesh(wheelGeo, wheelMat);
                        wR.position.set(-1.15, 0.38, z);
                        vGroup.add(wL, wR);
                    });

                    // Headlights
                    const hl = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.2, 0.05), new THREE.MeshBasicMaterial({ color: 0xffffff }));
                    hl.position.set(0.7, 1.0, -3.22);
                    const hr = hl.clone();
                    hr.position.x = -0.7;
                    vGroup.add(hl, hr);

                } else if (isJeep) {
                    // Military Reconnaissance 4x4 Gypsy / Jeep
                    const jeepMat = new THREE.MeshStandardMaterial({ color: 0x48583d, roughness: 0.4, metalness: 0.6 });
                    const body = new THREE.Mesh(new THREE.BoxGeometry(1.7, 0.8, 3.2), jeepMat);
                    body.position.set(0, 0.8, 0);
                    vGroup.add(body);

                    const cab = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.7, 1.6), new THREE.MeshStandardMaterial({ color: 0x11161d, roughness: 0.1, metalness: 0.9 }));
                    cab.position.set(0, 1.4, -0.2);
                    vGroup.add(cab);

                    // 4 Wheels
                    [-1.0, 1.0].forEach(z => {
                        const wL = new THREE.Mesh(wheelGeo, wheelMat);
                        wL.position.set(0.95, 0.38, z);
                        const wR = new THREE.Mesh(wheelGeo, wheelMat);
                        wR.position.set(-0.95, 0.38, z);
                        vGroup.add(wL, wR);
                    });
                } else {
                    // Civilian Mountain Transport SUV
                    const suvColors = [0x1e3a5f, 0x8c2d19, 0x2d3436, 0x8a7f72];
                    const suvMat = new THREE.MeshStandardMaterial({ color: suvColors[i % suvColors.length], roughness: 0.3, metalness: 0.7 });
                    const body = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.85, 3.6), suvMat);
                    body.position.set(0, 0.85, 0);
                    vGroup.add(body);

                    const cab = new THREE.Mesh(new THREE.BoxGeometry(1.6, 0.7, 2.0), new THREE.MeshStandardMaterial({ color: 0x11161d, roughness: 0.1, metalness: 0.95 }));
                    cab.position.set(0, 1.5, -0.2);
                    vGroup.add(cab);

                    [-1.1, 1.1].forEach(z => {
                        const wL = new THREE.Mesh(wheelGeo, wheelMat);
                        wL.position.set(1.0, 0.38, z);
                        const wR = new THREE.Mesh(wheelGeo, wheelMat);
                        wR.position.set(-1.0, 0.38, z);
                        vGroup.add(wL, wR);
                    });
                }

                this.worldGroup.add(vGroup);
                this.trafficVehicles.push({
                    mesh: vGroup,
                    curve: curve,
                    progress: i / numVehicles,
                    speed: 0.012 + (i % 3) * 0.004,
                    direction: (i % 2 === 0) ? 1 : -1,
                    laneOffset: (i % 2 === 0) ? 2.1 : -2.1
                });
            }
        }

        // =========================================================================
        // 6. Underground Military Drone Hive Complex in Mountain Face
        // =========================================================================
        buildUndergroundHive() {
            const hiveGroup = new THREE.Group();
            hiveGroup.position.set(0, 0, -22.0); // Facing Helipad LZ-01

            // 1. Heavy Reinforced Concrete Portal Archway
            const portalArchGeo = new THREE.BoxGeometry(22.0, 10.0, 8.0);
            const portalMat = new THREE.MeshStandardMaterial({ color: 0x22262e, roughness: 0.8, metalness: 0.5 });
            const portalArch = new THREE.Mesh(portalArchGeo, portalMat);
            portalArch.position.set(0, 5.0, 0);
            portalArch.castShadow = true;
            hiveGroup.add(portalArch);

            // Angled Concrete Wing Retaining Walls
            const wingGeo = new THREE.BoxGeometry(12.0, 9.0, 3.0);
            const wingL = new THREE.Mesh(wingGeo, portalMat);
            wingL.position.set(-15.5, 4.5, 2.5);
            wingL.rotation.y = Math.PI / 6;
            const wingR = new THREE.Mesh(wingGeo, portalMat);
            wingR.position.set(15.5, 4.5, 2.5);
            wingR.rotation.y = -Math.PI / 6;
            hiveGroup.add(wingL, wingR);

            // Carved Portal Chamber
            const tunnelCutGeo = new THREE.BoxGeometry(12.0, 6.5, 8.5);
            const tunnelInteriorMat = new THREE.MeshStandardMaterial({ color: 0x0a0e14, roughness: 0.95 });
            const tunnelCut = new THREE.Mesh(tunnelCutGeo, tunnelInteriorMat);
            tunnelCut.position.set(0, 3.25, 0.1);
            hiveGroup.add(tunnelCut);

            // Yellow/Black Hazard Striping Header
            const hazardGeo = new THREE.BoxGeometry(12.4, 0.7, 0.2);
            const hazardMat = new THREE.MeshBasicMaterial({ color: 0xffb800 });
            const hazardHeader = new THREE.Mesh(hazardGeo, hazardMat);
            hazardHeader.position.set(0, 6.8, 4.1);
            hiveGroup.add(hazardHeader);

            // Stencil Signage: "GARUDA HIVE // BORDER SECTOR 01"
            const canvas = document.createElement('canvas');
            canvas.width = 512; canvas.height = 64;
            const ctx = canvas.getContext('2d');
            ctx.fillStyle = '#14171d';
            ctx.fillRect(0, 0, 512, 64);
            ctx.fillStyle = '#00f0ff';
            ctx.font = 'bold 24px "JetBrains Mono", monospace';
            ctx.textAlign = 'center';
            ctx.fillText('GARUDA HIVE // BORDER SECTOR 01', 256, 42);
            const signTex = new THREE.CanvasTexture(canvas);
            const signMesh = new THREE.Mesh(new THREE.PlaneGeometry(8.8, 1.1), new THREE.MeshBasicMaterial({ map: signTex }));
            signMesh.position.set(0, 7.8, 4.12);
            hiveGroup.add(signMesh);

            // 2. Motorized Heavy Double Blast Doors
            const gateWidth = 5.9;
            const gateHeight = 6.2;
            const gateDepth = 0.5;
            const gateMat = new THREE.MeshStandardMaterial({ color: 0x2b313b, metalness: 0.85, roughness: 0.3 });

            this.hiveGateLeft = new THREE.Mesh(new THREE.BoxGeometry(gateWidth, gateHeight, gateDepth), gateMat);
            this.hiveGateLeft.position.set(-gateWidth * 0.5, gateHeight * 0.5, 3.8);
            hiveGroup.add(this.hiveGateLeft);

            this.hiveGateRight = new THREE.Mesh(new THREE.BoxGeometry(gateWidth, gateHeight, gateDepth), gateMat);
            this.hiveGateRight.position.set(gateWidth * 0.5, gateHeight * 0.5, 3.8);
            hiveGroup.add(this.hiveGateRight);

            // 3. Interior Underground Hangar Facility (Visible when gates open)
            const hangarChamber = new THREE.Group();
            hangarChamber.position.set(0, 0, -14.0);

            // Hangar Floor
            const floorGeo = new THREE.BoxGeometry(28.0, 0.2, 34.0);
            const floorMat = new THREE.MeshStandardMaterial({ color: 0x141820, roughness: 0.35, metalness: 0.7 });
            const hangarFloor = new THREE.Mesh(floorGeo, floorMat);
            hangarFloor.position.y = 0.1;
            hangarChamber.add(hangarFloor);

            // 4 Drone Docking & Charging Bays (GARUDA-01 to GARUDA-04)
            const bayOffsets = [
                { x: -6.5, z: -6.0, id: 'D01', status: 'ACTIVE / AIRBORNE' },
                { x: 6.5, z: -6.0, id: 'D02', status: 'STANDBY READY' },
                { x: -6.5, z: -16.0, id: 'D03', status: 'CHARGING (98%)' },
                { x: 6.5, z: -16.0, id: 'D04', status: 'STANDBY READY' }
            ];

            bayOffsets.forEach((b, idx) => {
                const padRingGeo = new THREE.RingGeometry(1.8, 2.1, 32);
                padRingGeo.rotateX(-Math.PI / 2);
                const col = (idx === 0) ? 0x00f0ff : ((idx === 2) ? 0xffb800 : 0x39ff14);
                const padRingMat = new THREE.MeshBasicMaterial({ color: col, side: THREE.DoubleSide });
                const padRing = new THREE.Mesh(padRingGeo, padRingMat);
                padRing.position.set(b.x, 0.22, b.z);
                hangarChamber.add(padRing);

                // Standby Drone Models inside Hangar
                if (idx > 0) {
                    const dMesh = this.createStandbyDroneHangarMesh();
                    dMesh.position.set(b.x, 0.50, b.z);
                    hangarChamber.add(dMesh);
                }
            });

            // Elevated Mission Control Deck Glass Panes
            const controlDeckGeo = new THREE.BoxGeometry(16.0, 2.8, 3.2);
            const controlDeckMat = new THREE.MeshStandardMaterial({ color: 0x1a2332, roughness: 0.3 });
            const controlDeck = new THREE.Mesh(controlDeckGeo, controlDeckMat);
            controlDeck.position.set(0, 6.2, -24.0);
            hangarChamber.add(controlDeck);

            const glassGeo = new THREE.BoxGeometry(14.0, 1.4, 0.1);
            const glassMat = new THREE.MeshPhysicalMaterial({ color: 0x00e5ff, transmission: 0.85, opacity: 0.9, transparent: true, roughness: 0.05 });
            const glass = new THREE.Mesh(glassGeo, glassMat);
            glass.position.set(0, 6.2, -22.3);
            hangarChamber.add(glass);

            hiveGroup.add(hangarChamber);
            this.worldGroup.add(hiveGroup);
        }

        createStandbyDroneHangarMesh() {
            const g = new THREE.Group();
            const bodyMat = new THREE.MeshStandardMaterial({ color: 0x22252a, roughness: 0.3, metalness: 0.8 });
            const body = new THREE.Mesh(new THREE.CylinderGeometry(0.26, 0.24, 0.14, 8), bodyMat);
            g.add(body);

            for (let i = 0; i < 8; i++) {
                const ang = (22.5 + i * 45.0) * (Math.PI / 180.0);
                const arm = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.03, 0.60), bodyMat);
                arm.position.set(Math.cos(ang) * 0.30, 0.04, Math.sin(ang) * 0.30);
                arm.rotation.y = -ang;
                g.add(arm);
            }
            g.scale.set(0.85, 0.85, 0.85);
            return g;
        }

        // =========================================================================
        // 7. Tactical Launching Pad LZ-01 & Cliff Surveillance Radar
        // =========================================================================
        buildLaunchPadLZ01() {
            const padTex = (window.GarudaTextureGenerator && window.GarudaTextureGenerator.createLaunchPadTexture)
                ? window.GarudaTextureGenerator.createLaunchPadTexture()
                : null;

            // 1. Helipad Surface (Orange Safety Base with Black "H")
            const padGeo = new THREE.CircleGeometry(2.0, 64);
            const padMat = new THREE.MeshStandardMaterial({
                map: padTex ? padTex.diffuse : null,
                color: 0xffffff,
                roughness: 0.45,
                metalness: 0.15,
                transparent: true,
                side: THREE.DoubleSide
            });

            const padMesh = new THREE.Mesh(padGeo, padMat);
            padMesh.rotation.x = -Math.PI / 2;
            padMesh.position.set(0.0, 0.015, 0.0);
            padMesh.receiveShadow = true;
            this.worldGroup.add(padMesh);

            // Helipad Concrete Rim
            const rimGeo = new THREE.RingGeometry(1.95, 2.25, 64);
            rimGeo.rotateX(-Math.PI / 2);
            const rimMat = new THREE.MeshStandardMaterial({ color: 0x1f2329, roughness: 0.85, metalness: 0.2 });
            const rimMesh = new THREE.Mesh(rimGeo, rimMat);
            rimMesh.position.set(0.0, 0.012, 0.0);
            rimMesh.receiveShadow = true;
            this.worldGroup.add(rimMesh);

            // 4 Corner Perimeter Beacon Towers
            const beaconGeo = new THREE.CylinderGeometry(0.05, 0.06, 0.18, 12);
            const beaconMat = new THREE.MeshStandardMaterial({
                color: 0x00ff88,
                emissive: 0x00ff88,
                emissiveIntensity: 0.95,
                roughness: 0.2
            });

            const beaconAngles = [Math.PI / 4, (3 * Math.PI) / 4, (5 * Math.PI) / 4, (7 * Math.PI) / 4];
            beaconAngles.forEach(ang => {
                const b = new THREE.Mesh(beaconGeo, beaconMat);
                b.position.set(Math.cos(ang) * 2.12, 0.09, Math.sin(ang) * 2.12);
                this.worldGroup.add(b);
            });

            // Cliff-top Observation Radar Tower at (45, Cliff Peak, -25)
            const radarX = 48.0; const radarZ = -32.0;
            const radarY = this.getGroundHeight(radarX, radarZ);

            const radarGroup = new THREE.Group();
            radarGroup.position.set(radarX, radarY, radarZ);

            const towerGeo = new THREE.CylinderGeometry(2.2, 2.6, 6.5, 8);
            const towerMat = new THREE.MeshStandardMaterial({ color: 0x383e48, roughness: 0.85, metalness: 0.3 });
            const tower = new THREE.Mesh(towerGeo, towerMat);
            tower.position.y = 3.25;
            radarGroup.add(tower);

            // Spinning Radar Radome
            const radomeGeo = new THREE.SphereGeometry(1.6, 16, 16, 0, Math.PI * 2, 0, Math.PI * 0.65);
            const radomeMat = new THREE.MeshStandardMaterial({ color: 0xe5eaf2, roughness: 0.2, metalness: 0.5 });
            this.radarDome = new THREE.Mesh(radomeGeo, radomeMat);
            this.radarDome.position.y = 6.8;
            radarGroup.add(this.radarDome);

            // Communications Mast
            const mastGeo = new THREE.CylinderGeometry(0.08, 0.15, 9.0, 6);
            const mastMat = new THREE.MeshStandardMaterial({ color: 0xc83232, roughness: 0.4, metalness: 0.8 });
            const mast = new THREE.Mesh(mastGeo, mastMat);
            mast.position.set(3.5, 4.5, 1.5);
            radarGroup.add(mast);

            this.worldGroup.add(radarGroup);
        }

        // =========================================================================
        // 8. 3D Tactical Waypoint Corridors & Ground Sensor Footprint
        // =========================================================================
        buildSurveillanceSector() {
            this.waypoints = [
                { id: 'WP-A', name: 'HIVE LZ-01 EXIT', pos: new THREE.Vector3(0, 4.0, 10) },
                { id: 'WP-B', name: 'HIGHWAY SECTOR NH-01', pos: new THREE.Vector3(35, 6.5, 65) },
                { id: 'WP-C', name: 'RIDGE OBSERVATION POST', pos: new THREE.Vector3(55, 14.0, -25) },
                { id: 'WP-D', name: 'SURU RIVER VALLEY', pos: new THREE.Vector3(-35, 5.0, -85) }
            ];

            // 3D Glowing Waypoint Markers
            this.waypoints.forEach(wp => {
                const wpGroup = new THREE.Group();
                wpGroup.position.copy(wp.pos);

                // Diamond Marker
                const dGeo = new THREE.OctahedronGeometry(0.5);
                const dMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff, wireframe: true });
                const dMesh = new THREE.Mesh(dGeo, dMat);
                wpGroup.add(dMesh);

                // Ground Beacon Ring
                const ringGeo = new THREE.RingGeometry(0.9, 1.1, 24);
                ringGeo.rotateX(-Math.PI / 2);
                const ringMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff, side: THREE.DoubleSide });
                const ring = new THREE.Mesh(ringGeo, ringMat);
                ring.position.y = -wp.pos.y + this.getGroundHeight(wp.pos.x, wp.pos.z) + 0.05;
                wpGroup.add(ring);

                this.worldGroup.add(wpGroup);
                this.waypointMarkers.push({ group: wpGroup, diamond: dMesh, baseRing: ring });
            });

            // Glowing Tactical Flight Corridor Tube
            const curve = new THREE.CatmullRomCurve3([
                this.waypoints[0].pos,
                this.waypoints[1].pos,
                this.waypoints[2].pos,
                this.waypoints[3].pos,
                this.waypoints[0].pos
            ]);
            const tubeGeo = new THREE.TubeGeometry(curve, 80, 0.09, 6, true);
            const tubeMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff, transparent: true, opacity: 0.45, wireframe: true });
            this.patrolPathMesh = new THREE.Mesh(tubeGeo, tubeMat);
            this.worldGroup.add(this.patrolPathMesh);

            // Ground Sensor Footprint Projection
            const footGeo = new THREE.RingGeometry(1.4, 1.7, 32);
            footGeo.rotateX(-Math.PI / 2);
            const footMat = new THREE.MeshBasicMaterial({ color: 0x39ff14, transparent: true, opacity: 0.65, side: THREE.DoubleSide });
            this.sensorFootprintMesh = new THREE.Mesh(footGeo, footMat);
            this.worldGroup.add(this.sensorFootprintMesh);
        }

        // =========================================================================
        // 9. Hive Gate Mechanical Animation & Mission Controls
        // =========================================================================
        openHiveGate() {
            this.hiveGateOpen = true;
            if (window.GarudaAudio) window.GarudaAudio.playUiClick();
            console.log("[GARUDA World] 🚪 HIVE BLAST DOORS OPENING...");
        }

        closeHiveGate() {
            this.hiveGateOpen = false;
            if (window.GarudaAudio) window.GarudaAudio.playUiClick();
            console.log("[GARUDA World] 🚪 HIVE BLAST DOORS CLOSING...");
        }

        toggleHiveGate() {
            if (this.hiveGateOpen) this.closeHiveGate();
            else this.openHiveGate();
        }

        updateHiveGates(dt) {
            const target = this.hiveGateOpen ? 1.0 : 0.0;
            this.hiveGateProgress += (target - this.hiveGateProgress) * Math.min(1.0, dt * 2.4);

            if (this.hiveGateLeft && this.hiveGateRight) {
                const maxSlide = 5.2; // Slide distance in meters
                this.hiveGateLeft.position.x = -2.95 - this.hiveGateProgress * maxSlide;
                this.hiveGateRight.position.x = 2.95 + this.hiveGateProgress * maxSlide;
            }
        }

        // =========================================================================
        // 10. Atmospheric Lighting & Weather Engine
        // =========================================================================
        setupAtmosphere() {
            // Crisp Himalayan Azure Sky & Altitude Fog
            this.scene.background = new THREE.Color(0x78add4);
            this.scene.fog = new THREE.FogExp2(0x9cbcd6, 0.0018);

            this.hemiLight = new THREE.HemisphereLight(0xbde0fe, 0x3d4b58, 0.95);
            this.scene.add(this.hemiLight);

            this.ambientLight = new THREE.AmbientLight(0xffffff, 0.45);
            this.scene.add(this.ambientLight);

            this.dirLight = new THREE.DirectionalLight(0xfff8ee, 1.85);
            this.dirLight.position.set(120, 180, 90);
            this.dirLight.castShadow = true;
            this.dirLight.shadow.mapSize.width = 2048;
            this.dirLight.shadow.mapSize.height = 2048;
            this.dirLight.shadow.camera.near = 0.5;
            this.dirLight.shadow.camera.far = 600;
            this.dirLight.shadow.camera.left = -120;
            this.dirLight.shadow.camera.right = 120;
            this.dirLight.shadow.camera.top = 120;
            this.dirLight.shadow.camera.bottom = -120;
            this.dirLight.shadow.bias = -0.0004;
            this.scene.add(this.dirLight);
        }

        setWeather(preset) {
            this.currentWeather = preset;
            if (!this.scene.fog) {
                this.scene.fog = new THREE.FogExp2(0x78add4, 0.002);
            }

            if (preset === 'NORMAL') {
                this.scene.fog.density = 0.0018;
                this.scene.fog.color.setHex(0x9cbcd6);
                this.scene.background.setHex(0x78add4);
                if (this.dirLight) this.dirLight.intensity = 1.85;
            } else if (preset === 'LIGHT_SNOW') {
                this.scene.fog.density = 0.0045;
                this.scene.fog.color.setHex(0xd0dbe5);
                this.scene.background.setHex(0xc2d0de);
                if (this.dirLight) this.dirLight.intensity = 1.20;
            } else if (preset === 'FOG') {
                this.scene.fog.density = 0.012;
                this.scene.fog.color.setHex(0x9aa6b2);
                this.scene.background.setHex(0x8f9ba8);
                if (this.dirLight) this.dirLight.intensity = 0.65;
            } else if (preset === 'OVERCAST') {
                this.scene.fog.density = 0.0035;
                this.scene.fog.color.setHex(0x738091);
                this.scene.background.setHex(0x657385);
                if (this.dirLight) this.dirLight.intensity = 0.90;
            }
            if (window.GarudaAudio) window.GarudaAudio.playUiClick();
        }

        setTimeOfDay(tod) {
            this.currentTimeOfDay = tod;
            if (tod === 'DAY') {
                if (this.dirLight) {
                    this.dirLight.position.set(120, 180, 90);
                    this.dirLight.color.setHex(0xfff8ee);
                    this.dirLight.intensity = 1.85;
                }
                if (this.hemiLight) this.hemiLight.color.setHex(0xbde0fe);
                this.scene.background.setHex(0x78add4);
                this.scene.fog.color.setHex(0x9cbcd6);
            } else if (tod === 'DAWN') {
                if (this.dirLight) {
                    this.dirLight.position.set(200, 35, 30);
                    this.dirLight.color.setHex(0xff7733);
                    this.dirLight.intensity = 1.5;
                }
                if (this.hemiLight) this.hemiLight.color.setHex(0x6b4478);
                this.scene.background.setHex(0xd86a42);
                this.scene.fog.color.setHex(0xb85b40);
            } else if (tod === 'NIGHT') {
                if (this.dirLight) {
                    this.dirLight.position.set(40, 120, -60);
                    this.dirLight.color.setHex(0x3a5585);
                    this.dirLight.intensity = 0.45;
                }
                if (this.hemiLight) this.hemiLight.color.setHex(0x0f1828);
                this.scene.background.setHex(0x0a101d);
                this.scene.fog.color.setHex(0x0c1422);
            }
            if (window.GarudaAudio) window.GarudaAudio.playUiClick();
        }

        // =========================================================================
        // 11. Per-Frame World Tick (Traffic, Radar Rotation, Hive Doors, Sensor)
        // =========================================================================
        update(dt) {
            this.time += dt;

            // 1. Smooth Hive Blast Doors Mechanics
            this.updateHiveGates(dt);

            // 2. Mountain Highway Traffic Loop (Vehicles planted firmly on road surface)
            if (this.roadCurve && this.trafficVehicles) {
                this.trafficVehicles.forEach(v => {
                    v.progress += v.speed * dt * v.direction;
                    if (v.progress > 1.0) v.progress = 0.0;
                    if (v.progress < 0.0) v.progress = 1.0;

                    const pt = v.curve.getPoint(v.progress);
                    const tangent = v.curve.getTangent(v.progress);
                    const normal = new THREE.Vector3(-tangent.z, 0, tangent.x).normalize();

                    const posX = pt.x + normal.x * v.laneOffset;
                    const posZ = pt.z + normal.z * v.laneOffset;
                    // Exact surface elevation query
                    const posY = this.getGroundHeight(posX, posZ) + 0.14;

                    v.mesh.position.set(posX, posY, posZ);

                    const lookTarget = pt.clone().add(tangent.clone().multiplyScalar(v.direction));
                    const targetGroundY = this.getGroundHeight(lookTarget.x + normal.x * v.laneOffset, lookTarget.z + normal.z * v.laneOffset) + 0.14;
                    v.mesh.lookAt(lookTarget.x + normal.x * v.laneOffset, targetGroundY, lookTarget.z + normal.z * v.laneOffset);
                });
            }

            // 3. Cliff Radar Rotation
            if (this.radarDome) {
                this.radarDome.rotation.y += dt * 1.8;
            }

            // 4. Waypoint Marker Pulsing
            this.waypointMarkers.forEach((wm, idx) => {
                wm.diamond.rotation.y += dt * 1.5;
                wm.diamond.position.y = Math.sin(this.time * 3.0 + idx) * 0.15;
            });

            // 5. Update Sensor Footprint Projection
            if (window.GarudaFlight && this.sensorFootprintMesh) {
                const dPos = window.GarudaFlight.position;
                const groundY = this.getGroundHeight(dPos.x, dPos.z);
                this.sensorFootprintMesh.position.set(dPos.x, groundY + 0.05, dPos.z);
                const scale = Math.max(0.6, (dPos.y - groundY) * 0.45);
                this.sensorFootprintMesh.scale.set(scale, scale, scale);
            }
        }

        initWorld() {
            this.setupAtmosphere();
            this.buildTerrain();
            this.buildSuruRiver();
            this.buildHighwayNetwork();
            this.buildUndergroundHive();
            this.buildLaunchPadLZ01();
            this.buildSurveillanceSector();

            console.log("[GARUDA World] 🏔️ Photorealistic Kargil Himalayan digital twin initialized successfully.");
        }
    }

    global.GarudaKargilWorld = GarudaKargilWorld;
})(typeof window !== 'undefined' ? window : this);
