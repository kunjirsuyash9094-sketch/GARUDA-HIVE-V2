/**
 * GARUDA HIVE V2 — Procedural High-Fidelity PBR Texture & Shader Generator
 * Generates:
 * 1. Military Tactical UAV Fuselage Armor with "GARUDA-01 RECONNAISSANCE UAV", Eagle Wing Insignia, Hex Bolts & Stencils
 * 2. High-Aspect-Ratio 2x2 Twill Carbon Fiber Weave
 * 3. Alpine Mountain Rock & Snow PBR Textures
 * 4. Winter Tactical Mountain Helipad with Black "H"
 */

(function(global) {
    'use strict';

    class GarudaTextureGenerator {
        /**
         * Creates Military Armor Texture with "GARUDA-01", "RECONNAISSANCE UAV", Eagle Wings, and Panel Seams
         */
        static createMilitaryHullTexture() {
            const size = 1024;
            const canvas = document.createElement('canvas');
            canvas.width = size;
            canvas.height = size;
            const ctx = canvas.getContext('2d');

            // 1. Base Military Gunmetal / Charcoal Armor
            ctx.fillStyle = '#22252a';
            ctx.fillRect(0, 0, size, size);

            // Subtle composite roughness & noise
            const imgData = ctx.getImageData(0, 0, size, size);
            const data = imgData.data;
            for (let i = 0; i < data.length; i += 4) {
                const n = (Math.random() - 0.5) * 12;
                data[i] = Math.min(255, Math.max(0, data[i] + n));
                data[i + 1] = Math.min(255, Math.max(0, data[i + 1] + n));
                data[i + 2] = Math.min(255, Math.max(0, data[i + 2] + n));
            }
            ctx.putImageData(imgData, 0, 0);

            // 2. Armor Panel Seams & Bevels
            ctx.strokeStyle = '#121417';
            ctx.lineWidth = 4;

            // Longitudinal and lateral structural panel lines
            ctx.strokeRect(60, 60, size - 120, size - 120);
            ctx.strokeRect(180, 180, size - 360, size - 360);

            ctx.beginPath();
            ctx.moveTo(size / 2, 60); ctx.lineTo(size / 2, size - 60);
            ctx.moveTo(60, size / 2); ctx.lineTo(size - 60, size / 2);
            ctx.stroke();

            // Edge Highlights
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
            ctx.lineWidth = 2;
            ctx.strokeRect(62, 62, size - 124, size - 124);

            // 3. Hex Bolts & Rivets
            ctx.fillStyle = '#101215';
            const drawHexBolt = (x, y) => {
                ctx.beginPath();
                ctx.arc(x, y, 6, 0, Math.PI * 2);
                ctx.fill();
                ctx.strokeStyle = 'rgba(255,255,255,0.15)';
                ctx.lineWidth = 1.5;
                ctx.stroke();
            };

            for (let x = 80; x < size - 60; x += 110) {
                drawHexBolt(x, 70);
                drawHexBolt(x, size - 70);
            }
            for (let y = 80; y < size - 60; y += 110) {
                drawHexBolt(70, y);
                drawHexBolt(size - 70, y);
            }

            // 4. Stencil Decals: "GARUDA-01" & "RECONNAISSANCE UAV"
            ctx.fillStyle = '#c5ccd8';
            ctx.font = 'bold 36px "Inter", "Arial Black", sans-serif';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';

            // Top Fuselage Stencils
            ctx.fillText('GARUDA - 01', size / 2, 280);

            ctx.font = '700 16px "Inter", monospace';
            ctx.fillStyle = '#7a8599';
            ctx.fillText('RECONNAISSANCE UAV  //  TACTICAL SPEC', size / 2, 320);

            // Eagle Wings Insignia
            ctx.strokeStyle = '#c5ccd8';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(size / 2 - 40, 230);
            ctx.lineTo(size / 2 - 10, 245);
            ctx.lineTo(size / 2, 235);
            ctx.lineTo(size / 2 + 10, 245);
            ctx.lineTo(size / 2 + 40, 230);
            ctx.lineTo(size / 2 + 20, 255);
            ctx.lineTo(size / 2, 248);
            ctx.lineTo(size / 2 - 20, 255);
            ctx.closePath();
            ctx.fillStyle = 'rgba(197, 204, 216, 0.7)';
            ctx.fill();
            ctx.stroke();

            // Corner Arm Markings ("ARM R30-01" / "01")
            ctx.font = 'bold 22px "Inter", monospace';
            ctx.fillStyle = '#ffcc00';
            ctx.fillText('ARM R30-01 [01]', size / 2, 700);

            // Hazard Warning Chevron
            ctx.fillStyle = '#ffcc00';
            ctx.fillRect(size / 2 - 60, 740, 120, 8);
            ctx.fillStyle = '#111';
            for (let i = -50; i < 50; i += 20) {
                ctx.beginPath();
                ctx.moveTo(size / 2 + i, 740);
                ctx.lineTo(size / 2 + i + 10, 748);
                ctx.lineTo(size / 2 + i + 5, 748);
                ctx.lineTo(size / 2 + i - 5, 740);
                ctx.closePath();
                ctx.fill();
            }

            const tex = new THREE.CanvasTexture(canvas);
            tex.wrapS = THREE.RepeatWrapping;
            tex.wrapT = THREE.RepeatWrapping;
            return tex;
        }

        /**
         * 2x2 Twill Carbon Fiber Weave Texture
         */
        static createCarbonFiberTextures() {
            const size = 256;
            const canvas = document.createElement('canvas');
            canvas.width = size;
            canvas.height = size;
            const ctx = canvas.getContext('2d');

            ctx.fillStyle = '#181a1f';
            ctx.fillRect(0, 0, size, size);

            const tileSize = 8;
            for (let y = 0; y < size; y += tileSize) {
                for (let x = 0; x < size; x += tileSize) {
                    const isEven = ((x / tileSize) + (y / tileSize)) % 2 === 0;
                    ctx.fillStyle = isEven ? '#20242b' : '#14161a';
                    ctx.fillRect(x, y, tileSize, tileSize);

                    ctx.fillStyle = isEven ? 'rgba(255,255,255,0.06)' : 'rgba(0,0,0,0.12)';
                    ctx.fillRect(x, y, tileSize, tileSize / 2);
                }
            }

            const diffuse = new THREE.CanvasTexture(canvas);
            diffuse.wrapS = THREE.RepeatWrapping;
            diffuse.wrapT = THREE.RepeatWrapping;
            diffuse.repeat.set(4, 4);

            return { diffuse };
        }

        /**
         * Alpine Mountain Snow & Ice Ground Texture
         */
        static createSnowTerrainTexture() {
            const size = 512;
            const canvas = document.createElement('canvas');
            canvas.width = size;
            canvas.height = size;
            const ctx = canvas.getContext('2d');

            // Crisp White-Blue Alpine Snow
            ctx.fillStyle = '#e8eff7';
            ctx.fillRect(0, 0, size, size);

            const imgData = ctx.getImageData(0, 0, size, size);
            const data = imgData.data;

            for (let i = 0; i < data.length; i += 4) {
                const grain = (Math.random() - 0.5) * 18;
                data[i] = Math.min(255, Math.max(0, data[i] + grain));
                data[i + 1] = Math.min(255, Math.max(0, data[i + 1] + grain + 2));
                data[i + 2] = Math.min(255, Math.max(0, data[i + 2] + grain + 5));
            }
            ctx.putImageData(imgData, 0, 0);

            // Subtle wind-drift snow ripples
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
            ctx.lineWidth = 12;
            for (let y = 0; y < size; y += 40) {
                ctx.beginPath();
                ctx.moveTo(0, y);
                ctx.bezierCurveTo(size * 0.3, y + 15, size * 0.7, y - 15, size, y);
                ctx.stroke();
            }

            const tex = new THREE.CanvasTexture(canvas);
            tex.wrapS = THREE.RepeatWrapping;
            tex.wrapT = THREE.RepeatWrapping;
            tex.repeat.set(24, 24);
            return tex;
        }

        /**
         * Mountain Rock Cliff Strata Texture
         */
        static createMountainRockTexture() {
            const size = 512;
            const canvas = document.createElement('canvas');
            canvas.width = size;
            canvas.height = size;
            const ctx = canvas.getContext('2d');

            // Dark slate mountain granite
            ctx.fillStyle = '#2d333b';
            ctx.fillRect(0, 0, size, size);

            // Rock layer striations
            for (let y = 0; y < size; y += 8) {
                const shade = 35 + Math.floor(Math.random() * 25);
                ctx.fillStyle = `rgb(${shade}, ${shade + 3}, ${shade + 6})`;
                ctx.fillRect(0, y, size, 4 + Math.random() * 6);
            }

            // Snow dusting in rock crevices
            ctx.fillStyle = 'rgba(235, 245, 255, 0.35)';
            for (let i = 0; i < 40; i++) {
                const rx = Math.random() * size;
                const ry = Math.random() * size;
                ctx.fillRect(rx, ry, Math.random() * 80 + 20, Math.random() * 4 + 1);
            }

            const tex = new THREE.CanvasTexture(canvas);
            tex.wrapS = THREE.RepeatWrapping;
            tex.wrapT = THREE.RepeatWrapping;
            tex.repeat.set(6, 6);
            return tex;
        }

        /**
         * Tactical Winter Helipad with Heated De-Icing Grid & Bold Black "H"
         */
        static createLaunchPadTexture() {
            const size = 1024;
            const canvas = document.createElement('canvas');
            canvas.width = size;
            canvas.height = size;
            const ctx = canvas.getContext('2d');
            const center = size / 2;
            const radius = size * 0.48;

            ctx.clearRect(0, 0, size, size);

            // Circular Pad Base (High-Contrast Tactical Safety Orange)
            ctx.save();
            ctx.beginPath();
            ctx.arc(center, center, radius, 0, Math.PI * 2);
            ctx.closePath();
            ctx.fillStyle = '#d94414';
            ctx.fill();
            ctx.clip();

            // De-icing heated wire micro-grid
            ctx.strokeStyle = 'rgba(0, 0, 0, 0.08)';
            ctx.lineWidth = 2;
            for (let x = 0; x < size; x += 16) {
                ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, size); ctx.stroke();
            }
            for (let y = 0; y < size; y += 16) {
                ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(size, y); ctx.stroke();
            }

            // Matte Black Perimeter Ring
            ctx.strokeStyle = '#14161a';
            ctx.lineWidth = 54;
            ctx.beginPath();
            ctx.arc(center, center, radius - 27, 0, Math.PI * 2);
            ctx.stroke();

            // Inner Yellow/Black Hazard Border
            ctx.strokeStyle = '#ffcc00';
            ctx.lineWidth = 14;
            ctx.beginPath();
            ctx.arc(center, center, radius * 0.78, 0, Math.PI * 2);
            ctx.stroke();

            // Bold Matte Black Helipad "H"
            ctx.fillStyle = '#14161a';
            const hHeight = radius * 0.72;
            const hBarWidth = 46;
            const hSpan = radius * 0.54;

            // Left Bar
            ctx.fillRect(center - hSpan / 2 - hBarWidth / 2, center - hHeight / 2, hBarWidth, hHeight);
            // Right Bar
            ctx.fillRect(center + hSpan / 2 - hBarWidth / 2, center - hHeight / 2, hBarWidth, hHeight);
            // Cross Bar
            ctx.fillRect(center - hSpan / 2, center - hBarWidth / 2, hSpan, hBarWidth);

            // Stencil Coordinates
            ctx.fillStyle = '#ffffff';
            ctx.font = 'bold 20px monospace';
            ctx.textAlign = 'center';
            ctx.fillText('GARUDA LZ-01 // ELEV: 2450m', center, center + radius * 0.62);

            ctx.restore();

            const tex = new THREE.CanvasTexture(canvas);
            return { diffuse: tex };
        }

        /**
         * Creates Radial Gradient Motion-Blur Texture for High-RPM Propeller Spin
         */
        static createRotorBlurTexture() {
            const size = 512;
            const canvas = document.createElement('canvas');
            canvas.width = size;
            canvas.height = size;
            const ctx = canvas.getContext('2d');
            const cx = size / 2;
            const cy = size / 2;
            const r = size / 2 - 8;

            ctx.clearRect(0, 0, size, size);

            // Radial gradient simulating physical high-speed carbon blade disc
            const grad = ctx.createRadialGradient(cx, cy, 25, cx, cy, r);
            grad.addColorStop(0.00, 'rgba(15, 18, 24, 0.0)');
            grad.addColorStop(0.12, 'rgba(20, 24, 32, 0.20)');
            grad.addColorStop(0.55, 'rgba(25, 30, 40, 0.48)');
            grad.addColorStop(0.85, 'rgba(18, 22, 30, 0.65)');
            grad.addColorStop(0.94, 'rgba(210, 225, 245, 0.40)'); // Tip specular flash ring
            grad.addColorStop(0.98, 'rgba(15, 18, 24, 0.20)');
            grad.addColorStop(1.00, 'rgba(10, 12, 16, 0.0)');

            ctx.fillStyle = grad;
            ctx.beginPath();
            ctx.arc(cx, cy, r, 0, Math.PI * 2);
            ctx.fill();

            // Circular chord trail streaks
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
            ctx.lineWidth = 2.0;
            ctx.beginPath();
            ctx.arc(cx, cy, r * 0.72, 0, Math.PI * 2);
            ctx.stroke();

            ctx.beginPath();
            ctx.arc(cx, cy, r * 0.90, 0, Math.PI * 2);
            ctx.stroke();

            const tex = new THREE.CanvasTexture(canvas);
            tex.needsUpdate = true;
            return tex;
        }
    }

    global.GarudaTextureGenerator = GarudaTextureGenerator;
})(typeof window !== 'undefined' ? window : this);
