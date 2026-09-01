/**
 * GARUDA HIVE V2 — Web Client Authoritative Telemetry Bridge & Controller
 * Connects to C++20 Simulation Server via WebSocket (/ws/telemetry) with Seamless 400Hz Local Fallback
 */

class GarudaSimulationClient {
    constructor() {
        this.ws = null;
        this.connected = false;
        this.selectedDroneIndex = 0;
        this.latestSnapshot = null;
        this.onTelemetryUpdate = null;
        this.octoModel = null;
        this.currentPayloadType = 1;
        this.connect();
    }

    connect() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.host || '127.0.0.1:8000';
        const wsUrl = `${protocol}//${host}/ws/telemetry`;

        try {
            this.ws = new WebSocket(wsUrl);

            this.ws.onopen = () => {
                this.connected = true;
                console.log("[GARUDA Bridge] Connected to authoritative C++20 kernel (400 Hz).");
                this.updateConnectionUI(true);
            };

            this.ws.onmessage = (event) => {
                try {
                    const snapshot = JSON.parse(event.data);
                    this.latestSnapshot = snapshot;
                    this.updateSceneAndHUD(snapshot);
                    if (this.onTelemetryUpdate) {
                        this.onTelemetryUpdate(snapshot);
                    }
                } catch (err) {
                    console.error("[GARUDA Bridge] Telemetry parse error:", err);
                }
            };

            this.ws.onclose = () => {
                this.connected = false;
                this.updateConnectionUI(false);
                setTimeout(() => this.connect(), 4000);
            };

            this.ws.onerror = () => {
                this.connected = false;
                this.updateConnectionUI(false);
            };
        } catch (e) {
            this.connected = false;
            this.updateConnectionUI(false);
        }
    }

    sendAction(action, payload = {}) {
        if (this.connected && this.ws && this.ws.readyState === WebSocket.OPEN) {
            const droneId = this.latestSnapshot?.drones?.[this.selectedDroneIndex]?.drone_id || "GARUDA-HL-01";
            const msg = { action, drone_id: droneId, ...payload };
            this.ws.send(JSON.stringify(msg));
        }
    }

    arm() {
        if (window.GarudaFlight && !window.GarudaFlight.armed) {
            window.GarudaFlight.arm();
        }
        this.sendAction("arm");
    }

    disarm() {
        if (window.GarudaFlight && window.GarudaFlight.armed) {
            window.GarudaFlight.disarm();
        }
        this.sendAction("disarm");
    }

    takeoff() {
        if (window.GarudaFlight) {
            window.GarudaFlight.launch();
        }
        this.sendAction("takeoff");
    }

    land() {
        if (window.GarudaFlight) {
            window.GarudaFlight.land();
        }
        this.sendAction("land");
    }

    failMotor(index = 0) {
        if (window.GarudaFlight) {
            window.GarudaFlight.failMotor(index);
        }
        this.sendAction("fail_motor", { motor_index: index });
    }

    resetFailures() {
        if (window.GarudaFlight) {
            window.GarudaFlight.restoreFailures();
        }
        this.sendAction("reset_failures");
    }

    setControl(roll, pitch, yaw_rate, throttle) {
        this.sendAction("control", { roll, pitch, yaw_rate, throttle });
    }

    attachPayload(type = 1) {
        this.currentPayloadType = parseInt(type);
        const payloadMasses = [0.0, 1.50, 2.20, 1.10, 1.60, 3.50, 2.80, 4.20];
        const mass = payloadMasses[this.currentPayloadType] || 0.0;

        if (window.GarudaFlight) {
            window.GarudaFlight.setPayloadMass(mass);
        }
        if (this.octoModel) {
            this.octoModel.updatePayloadMesh(this.currentPayloadType);
        }

        const sel = document.getElementById('payload-select');
        if (sel) sel.value = String(this.currentPayloadType);

        this.sendAction("attach_payload", { payload_type: this.currentPayloadType });
    }

    detachPayload() {
        this.attachPayload(0);
        this.sendAction("detach_payload");
    }

    setGimbal(pitch_deg, yaw_deg, zoom = 1.0) {
        this.sendAction("set_gimbal", { pitch_deg: parseFloat(pitch_deg), yaw_deg: parseFloat(yaw_deg), zoom: parseFloat(zoom) });
        if (this.octoModel && this.octoModel.gimbalPitchGroup && this.octoModel.gimbalYawGroup) {
            this.octoModel.gimbalPitchGroup.rotation.x = parseFloat(pitch_deg) * (Math.PI / 180.0);
            this.octoModel.gimbalYawGroup.rotation.y = parseFloat(yaw_deg) * (Math.PI / 180.0);
        }
    }

    setSensorStatus(sensorIndex, status) {
        this.sendAction("set_sensor_status", { sensor_index: parseInt(sensorIndex), status: parseInt(status) });
    }

    async pauseSimulation() {
        if (this.connected) await fetch('/api/simulation/pause', { method: 'POST' }).catch(() => {});
    }

    async startSimulation() {
        if (this.connected) await fetch('/api/simulation/start', { method: 'POST' }).catch(() => {});
    }

    async resetSimulation() {
        if (window.GarudaFlight) {
            window.GarudaFlight.resetToPad();
        }
        if (this.connected) await fetch('/api/simulation/reset', { method: 'POST' }).catch(() => {});
    }

    async stepSimulation(n = 1) {
        if (this.connected) await fetch(`/api/simulation/step?count=${n}`, { method: 'POST' }).catch(() => {});
    }

    async setSpeed(multiplier) {
        if (this.connected) await fetch(`/api/simulation/speed?multiplier=${multiplier}`, { method: 'POST' }).catch(() => {});
    }

    updateSceneAndHUD(snapshot) {
        if (!snapshot.drones || snapshot.drones.length === 0) return;
        const d = snapshot.drones[this.selectedDroneIndex] || snapshot.drones[0];

        // 1. Update 3D Model in Three.js
        if (this.octoModel) {
            this.octoModel.updateFromTelemetry(d);
        }

        // 2. Update Primary Flight Display (PFD)
        const altElem = document.getElementById('pfd-alt');
        if (altElem) altElem.textContent = `${(d.altitude || 0).toFixed(2)} m`;

        const spdElem = document.getElementById('pfd-spd');
        if (spdElem) spdElem.textContent = `${((d.ground_speed || 0) * 3.6).toFixed(1)} km/h`;

        const vsElem = document.getElementById('pfd-vs');
        if (vsElem) vsElem.textContent = `${((d.vertical_speed || 0) >= 0 ? '+' : '')}${(d.vertical_speed || 0).toFixed(2)} m/s`;

        const rollElem = document.getElementById('pfd-roll');
        if (rollElem) rollElem.textContent = `${(d.rpy_deg?.roll || 0).toFixed(1)}°`;

        const pitchElem = document.getElementById('pfd-pitch');
        if (pitchElem) pitchElem.textContent = `${(d.rpy_deg?.pitch || 0).toFixed(1)}°`;

        const yawElem = document.getElementById('pfd-yaw');
        if (yawElem) yawElem.textContent = `${(d.rpy_deg?.yaw || 0).toFixed(1)}°`;

        const thrustElem = document.getElementById('pfd-thrust');
        if (thrustElem) thrustElem.textContent = `${(d.total_thrust || 0).toFixed(1)} N (TWR ${(d.twr || 0).toFixed(2)})`;

        // 3. Update 6S Battery & Electrical HUD
        if (d.battery) {
            const batPctElem = document.getElementById('bat-pct');
            if (batPctElem) batPctElem.textContent = `${(d.battery.soc * 100).toFixed(1)}%`;

            const batVoltElem = document.getElementById('bat-volt');
            if (batVoltElem) batVoltElem.textContent = `${(d.battery.voltage_terminal || 25.2).toFixed(2)} V`;

            const batCurrElem = document.getElementById('bat-curr');
            if (batCurrElem) batCurrElem.textContent = `${(d.battery.current_amps || 0).toFixed(1)} A`;

            const batPwrElem = document.getElementById('bat-pwr');
            if (batPwrElem) batPwrElem.textContent = `${(d.battery.power_w || 0).toFixed(0)} W`;

            const cellContainer = document.getElementById('cell-voltages-container');
            if (cellContainer && d.battery.cells) {
                cellContainer.innerHTML = d.battery.cells.map((cv, idx) => {
                    const fillPct = Math.max(0, Math.min(100, ((cv - 3.27) / (4.20 - 3.27)) * 100));
                    const col = cv > 3.7 ? '#39ff14' : (cv > 3.5 ? '#ffb800' : '#ff3366');
                    return `
                        <div style="display:flex;align-items:center;gap:6px;font-size:10px;margin-bottom:3px;">
                            <span style="width:20px;color:var(--fg-muted);">C${idx+1}</span>
                            <div style="flex:1;height:5px;background:rgba(255,255,255,0.1);border-radius:3px;overflow:hidden;">
                                <div style="width:${fillPct}%;height:100%;background:${col};"></div>
                            </div>
                            <span style="width:34px;text-align:right;font-family:var(--font-mono);">${cv.toFixed(2)}V</span>
                        </div>
                    `;
                }).join('');
            }
        }

        // 4. Update 8 Motors Telemetry Grid
        const motorGrid = document.getElementById('motor-telemetry-grid');
        if (motorGrid && d.motor_rpm) {
            motorGrid.innerHTML = d.motor_rpm.map((rpm, idx) => {
                const pct = Math.min(100, (rpm / 5000) * 100);
                const health = d.motor_health ? d.motor_health[idx] : 0;
                const isFailed = health === 2;
                const col = isFailed ? '#ff3366' : (rpm > 500 ? '#00f0ff' : '#8b9eb7');
                const thrustN = d.motor_thrust ? d.motor_thrust[idx] : 0;
                const tempC = d.motor_temp ? d.motor_temp[idx] : 26;
                const pwrW = d.motor_power ? d.motor_power[idx] : 0;

                return `
                    <div class="motor-card ${isFailed ? 'failed' : ''}" style="background:rgba(15,27,48,0.75);padding:6px 8px;border-radius:6px;border:1px solid ${isFailed ? '#ff3366' : 'rgba(44,90,134,0.35)'};">
                        <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:2px;">
                            <span style="font-weight:700;font-size:10px;color:${isFailed ? '#ff3366' : 'var(--fg-main)'};">M${idx+1} (${idx % 2 === 0 ? 'CCW' : 'CW'})</span>
                            <span style="font-size:9px;font-family:var(--font-mono);color:${col};font-weight:600;">${isFailed ? 'FAILED' : `${rpm.toFixed(0)} RPM`}</span>
                        </div>
                        <div style="height:4px;background:rgba(255,255,255,0.08);border-radius:2px;overflow:hidden;margin-bottom:4px;">
                            <div style="width:${pct}%;height:100%;background:${col};transition:width 0.05s linear;"></div>
                        </div>
                        <div style="display:flex;justify-content:space-between;font-size:9px;color:var(--fg-muted);font-family:var(--font-mono);">
                            <span>${thrustN.toFixed(1)}N</span>
                            <span>${pwrW.toFixed(0)}W</span>
                            <span>${tempC.toFixed(0)}°C</span>
                        </div>
                    </div>
                `;
            }).join('');
        }

        // 5. Update Modular Payload & Gimbal HUD
        if (d.payload) {
            const pMassElem = document.getElementById('payload-mass');
            if (pMassElem) pMassElem.textContent = `${(d.payload.mass_kg || 0).toFixed(2)} kg`;

            const pPwrBadge = document.getElementById('payload-power-badge');
            if (pPwrBadge) pPwrBadge.textContent = `${(d.payload.power_w || 0).toFixed(0)} W`;

            const pStateElem = document.getElementById('payload-state-label');
            if (pStateElem) {
                pStateElem.textContent = d.payload.attached ? "ATTACHED" : "DETACHED";
                pStateElem.style.color = d.payload.attached ? '#39ff14' : '#8b9eb7';
            }

            const totalMassElem = document.getElementById('total-vehicle-mass');
            if (totalMassElem) totalMassElem.textContent = `${(8.50 + (d.payload.mass_kg || 0)).toFixed(2)} kg`;
        }

        // 6. Update Extensible Sensors Grid
        const sensorGrid = document.getElementById('sensor-telemetry-grid');
        if (sensorGrid && d.sensors) {
            sensorGrid.innerHTML = d.sensors.map((s) => `
                <div class="sensor-card">
                    <div>
                        <span style="font-weight:700;color:var(--fg-main);">${s.name}</span>
                        <span style="font-family:var(--font-mono);font-size:9px;color:var(--accent-cyan);margin-left:4px;">${s.val || ''}</span>
                    </div>
                    <span style="font-weight:700;font-family:var(--font-mono);font-size:9px;color:#39ff14;">${s.status}</span>
                </div>
            `).join('');
        }

        // 7. Update LifeCycle, RTH Status & Health Badges
        const stateBadge = document.getElementById('vehicle-state-badge');
        if (stateBadge) {
            const isArmed = d.armed || (window.GarudaFlight && window.GarudaFlight.armed);
            const flightMode = d.flight_mode || (window.GarudaFlight && window.GarudaFlight.flightMode) || 'ALT_HOLD';
            const rthStage = d.rth_stage || 0;
            const distPad = d.dist_to_pad !== undefined ? d.dist_to_pad : Math.hypot(d.position?.x || 0, d.position?.z || 0);

            if (rthStage > 0) {
                let stageText = "RTH: RETURNING TO PAD";
                if (rthStage === 1) stageText = "RTH: CLIMBING SAFE ALT";
                else if (rthStage === 2) stageText = `RTH: TRANSIT (${distPad.toFixed(1)}m)`;
                else if (rthStage === 3) stageText = `RTH: ALIGNING OVER "H"`;
                else if (rthStage === 4) stageText = "RTH: FINAL PAD DESCENT";
                else if (rthStage === 5) stageText = "TOUCHDOWN ON PAD";
                stateBadge.textContent = stageText;
                stateBadge.style.color = '#ffb800';
                stateBadge.style.borderColor = '#ffb800';
            } else {
                stateBadge.textContent = isArmed ? `ARMED [${flightMode}]` : 'DISARMED';
                stateBadge.style.color = isArmed ? '#39ff14' : '#8b9eb7';
                stateBadge.style.borderColor = isArmed ? '#39ff14' : 'rgba(44,90,134,0.35)';
            }
        }

        const distPadElem = document.getElementById('pfd-pad-dist');
        if (distPadElem) {
            const dist = d.dist_to_pad !== undefined ? d.dist_to_pad : Math.hypot(d.position?.x || 0, d.position?.z || 0);
            distPadElem.textContent = `${dist.toFixed(2)} m`;
        }

        const tickElem = document.getElementById('sim-tick-val');
        if (tickElem) tickElem.textContent = `TICK: ${snapshot.tick || 0} (${(snapshot.time || 0).toFixed(2)}s)`;
    }

    updateConnectionUI(isConnected) {
        let badge = document.getElementById('garuda-server-badge');
        if (!badge) {
            badge = document.createElement('div');
            badge.id = 'garuda-server-badge';
            badge.style.position = 'fixed';
            badge.style.top = '14px';
            badge.style.left = '50%';
            badge.style.transform = 'translateX(-50%)';
            badge.style.zIndex = '9999';
            badge.style.padding = '6px 16px';
            badge.style.borderRadius = '20px';
            badge.style.fontSize = '11px';
            badge.style.fontWeight = '600';
            badge.style.fontFamily = "'JetBrains Mono', monospace";
            badge.style.backdropFilter = 'blur(14px)';
            badge.style.display = 'flex';
            badge.style.alignItems = 'center';
            badge.style.gap = '8px';
            badge.style.boxShadow = '0 4px 24px rgba(0,0,0,0.6)';
            document.body.appendChild(badge);
        }

        if (isConnected) {
            badge.style.background = 'rgba(10, 30, 20, 0.92)';
            badge.style.border = '1px solid #39ff14';
            badge.style.color = '#39ff14';
            badge.innerHTML = `<span style="width:8px;height:8px;border-radius:50%;background:#39ff14;display:inline-block;box-shadow:0 0 8px #39ff14;"></span> C++20 SERVER CONNECTED [400 Hz]`;
        } else {
            badge.style.background = 'rgba(15, 27, 48, 0.92)';
            badge.style.border = '1px solid #00f0ff';
            badge.style.color = '#00f0ff';
            badge.innerHTML = `<span style="width:8px;height:8px;border-radius:50%;background:#00f0ff;display:inline-block;box-shadow:0 0 8px #00f0ff;"></span> STANDALONE 400Hz 6-DOF PHYSICS [ACTIVE]`;
        }
    }
}

window.GarudaClient = new GarudaSimulationClient();
