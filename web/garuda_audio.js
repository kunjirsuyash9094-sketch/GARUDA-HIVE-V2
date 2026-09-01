/**
 * GARUDA HIVE V2 — Web Audio API Procedural Drone Sound Synthesizer
 * Synthesizes realistic 8-rotor BLDC motor harmonic whine, propeller blade chopping pink noise,
 * ESC startup melodies, launch thrust roar, landing touchdown thumps, and avionics alerts.
 */

(function(global) {
    'use strict';

    class GarudaAudioSystem {
        constructor() {
            this.ctx = null;
            this.initialized = false;
            this.muted = false;
            this.masterVolume = 0.6;

            // Audio Nodes
            this.masterGain = null;
            this.motorGain = null;
            this.propWashGain = null;
            this.noiseNode = null;
            this.noiseFilter = null;

            // 8 Motor Harmonic Oscillators
            this.motorOscs = [];
            this.motorOscGains = [];
            this.lastRpm = 0;

            // Audio context auto-unlock on first user interaction
            this.unlockHandler = () => {
                this.init();
                window.removeEventListener('click', this.unlockHandler);
                window.removeEventListener('keydown', this.unlockHandler);
            };
            window.addEventListener('click', this.unlockHandler);
            window.addEventListener('keydown', this.unlockHandler);
        }

        init() {
            if (this.initialized) return;
            try {
                const AudioCtx = window.AudioContext || window.webkitAudioContext;
                this.ctx = new AudioCtx();

                // Master Gain
                this.masterGain = this.ctx.createGain();
                this.masterGain.gain.value = this.muted ? 0.0 : this.masterVolume;
                this.masterGain.connect(this.ctx.destination);

                // Motor Whine Sub-Gain
                this.motorGain = this.ctx.createGain();
                this.motorGain.gain.value = 0.0;
                this.motorGain.connect(this.masterGain);

                // Blade Prop-Wash Turbulence (Filtered Pink Noise)
                this.createNoiseGenerator();

                // Multi-harmonic BLDC Motor Oscillators (Fundamental + 2nd + 3rd harmonics + ESC switching ripple)
                const harmonicMultipliers = [1.0, 2.0, 3.0, 4.0, 8.0];
                const harmonicGains = [0.25, 0.18, 0.10, 0.06, 0.03];

                for (let i = 0; i < harmonicMultipliers.length; i++) {
                    const osc = this.ctx.createOscillator();
                    osc.type = (i % 2 === 0) ? 'sawtooth' : 'triangle';
                    osc.frequency.setValueAtTime(80 * harmonicMultipliers[i], this.ctx.currentTime);

                    const g = this.ctx.createGain();
                    g.gain.setValueAtTime(harmonicGains[i], this.ctx.currentTime);

                    osc.connect(g);
                    g.connect(this.motorGain);
                    osc.start();

                    this.motorOscs.push({ osc, mult: harmonicMultipliers[i] });
                    this.motorOscGains.push(g);
                }

                this.initialized = true;
                console.log("[GARUDA Audio] Web Audio Synthesizer initialized successfully.");
            } catch (err) {
                console.warn("[GARUDA Audio] AudioContext could not be initialized:", err);
            }
        }

        createNoiseGenerator() {
            if (!this.ctx) return;
            // Generate 2 seconds of pink noise buffer
            const bufferSize = this.ctx.sampleRate * 2;
            const buffer = this.ctx.createBuffer(1, bufferSize, this.ctx.sampleRate);
            const output = buffer.getChannelData(0);

            let b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0;
            for (let i = 0; i < bufferSize; i++) {
                const white = Math.random() * 2 - 1;
                b0 = 0.99886 * b0 + white * 0.0555179;
                b1 = 0.99332 * b1 + white * 0.0750759;
                b2 = 0.96900 * b2 + white * 0.1538520;
                b3 = 0.86650 * b3 + white * 0.3104856;
                b4 = 0.55000 * b4 + white * 0.5329522;
                b5 = -0.7616 * b5 - white * 0.0168980;
                output[i] = (b0 + b1 + b2 + b3 + b4 + b5 + b6 + white * 0.5362) * 0.11;
                b6 = white * 0.115926;
            }

            const noiseSource = this.ctx.createBufferSource();
            noiseSource.buffer = buffer;
            noiseSource.loop = true;

            // Bandpass filter for blade air displacement chop
            this.noiseFilter = this.ctx.createBiquadFilter();
            this.noiseFilter.type = 'bandpass';
            this.noiseFilter.frequency.setValueAtTime(320, this.ctx.currentTime);
            this.noiseFilter.Q.setValueAtTime(1.8, this.ctx.currentTime);

            this.propWashGain = this.ctx.createGain();
            this.propWashGain.gain.setValueAtTime(0.0, this.ctx.currentTime);

            noiseSource.connect(this.noiseFilter);
            this.noiseFilter.connect(this.propWashGain);
            this.propWashGain.connect(this.masterGain);

            noiseSource.start();
        }

        ensureContext() {
            if (!this.initialized) this.init();
            if (this.ctx && this.ctx.state === 'suspended') {
                this.ctx.resume().catch(() => {});
            }
        }

        /**
         * Real-time audio modulation based on current motor RPM, vertical speed, and thrust
         */
        updateMotorSound(avgRpm, thrustRatio = 0.5) {
            this.ensureContext();
            if (!this.initialized || !this.ctx || this.muted) return;

            const t = this.ctx.currentTime;
            this.lastRpm = avgRpm;

            if (avgRpm < 50) {
                // Motors off
                this.motorGain.gain.setTargetAtTime(0.0, t, 0.05);
                if (this.propWashGain) this.propWashGain.gain.setTargetAtTime(0.0, t, 0.05);
                return;
            }

            // Blade Pass Frequency: (RPM / 60) * 2 blades * 8 rotors * scaling
            const bpf = Math.max(35, (avgRpm / 60.0) * 2.0 * 2.2);

            // Update harmonic oscillator frequencies
            for (let i = 0; i < this.motorOscs.length; i++) {
                const targetFreq = bpf * this.motorOscs[i].mult;
                this.motorOscs[i].osc.frequency.setTargetAtTime(targetFreq, t, 0.04);
            }

            // Motor Whine volume based on RPM
            const rpmNorm = Math.min(1.0, avgRpm / 4800.0);
            const motorVol = Math.pow(rpmNorm, 1.4) * 0.45;
            this.motorGain.gain.setTargetAtTime(motorVol, t, 0.05);

            // Prop wash noise filter frequency & gain
            if (this.noiseFilter && this.propWashGain) {
                const filterFreq = 180 + rpmNorm * 900 + thrustRatio * 400;
                this.noiseFilter.frequency.setTargetAtTime(filterFreq, t, 0.05);

                const washVol = Math.pow(rpmNorm, 1.6) * 0.35 + thrustRatio * 0.15;
                this.propWashGain.gain.setTargetAtTime(washVol, t, 0.05);
            }
        }

        /**
         * ESC Initialization Chime (Di-Do-Da-Ding)
         */
        playArmChime() {
            this.ensureContext();
            if (!this.initialized || !this.ctx || this.muted) return;
            const t = this.ctx.currentTime;
            const notes = [440, 554.37, 659.25, 880]; // A4, C#5, E5, A5
            const durations = [0.08, 0.08, 0.08, 0.22];
            let offset = 0;

            notes.forEach((freq, idx) => {
                const osc = this.ctx.createOscillator();
                osc.type = 'sine';
                osc.frequency.setValueAtTime(freq, t + offset);

                const g = this.ctx.createGain();
                g.gain.setValueAtTime(0.2, t + offset);
                g.gain.exponentialRampToValueAtTime(0.001, t + offset + durations[idx]);

                osc.connect(g);
                g.connect(this.masterGain);

                osc.start(t + offset);
                osc.stop(t + offset + durations[idx]);

                offset += durations[idx] + 0.02;
            });
        }

        /**
         * Disarm Sound (Descending Tone)
         */
        playDisarmSound() {
            this.ensureContext();
            if (!this.initialized || !this.ctx || this.muted) return;
            const t = this.ctx.currentTime;
            const notes = [659.25, 554.37, 440, 220];
            let offset = 0;

            notes.forEach((freq) => {
                const osc = this.ctx.createOscillator();
                osc.type = 'sine';
                osc.frequency.setValueAtTime(freq, t + offset);

                const g = this.ctx.createGain();
                g.gain.setValueAtTime(0.18, t + offset);
                g.gain.exponentialRampToValueAtTime(0.001, t + offset + 0.09);

                osc.connect(g);
                g.connect(this.masterGain);

                osc.start(t + offset);
                osc.stop(t + offset + 0.09);
                offset += 0.09;
            });
        }

        /**
         * Launch Thrust Roar & Ascent Whoosh
         */
        playLaunchSound() {
            this.ensureContext();
            if (!this.initialized || !this.ctx || this.muted) return;
            const t = this.ctx.currentTime;

            // Low frequency sub-bass rumble
            const osc = this.ctx.createOscillator();
            osc.type = 'sine';
            osc.frequency.setValueAtTime(60, t);
            osc.frequency.exponentialRampToValueAtTime(140, t + 1.2);

            const g = this.ctx.createGain();
            g.gain.setValueAtTime(0.35, t);
            g.gain.exponentialRampToValueAtTime(0.01, t + 1.5);

            osc.connect(g);
            g.connect(this.masterGain);

            osc.start(t);
            osc.stop(t + 1.5);
        }

        /**
         * Touchdown Landing Impact Thud
         */
        playTouchdownSound() {
            this.ensureContext();
            if (!this.initialized || !this.ctx || this.muted) return;
            const t = this.ctx.currentTime;

            const osc = this.ctx.createOscillator();
            osc.type = 'triangle';
            osc.frequency.setValueAtTime(90, t);
            osc.frequency.exponentialRampToValueAtTime(30, t + 0.25);

            const g = this.ctx.createGain();
            g.gain.setValueAtTime(0.4, t);
            g.gain.exponentialRampToValueAtTime(0.001, t + 0.25);

            osc.connect(g);
            g.connect(this.masterGain);

            osc.start(t);
            osc.stop(t + 0.25);
        }

        /**
         * Mechanical Futuristic UI Button Click
         */
        playUiClick() {
            this.ensureContext();
            if (!this.initialized || !this.ctx || this.muted) return;
            const t = this.ctx.currentTime;

            const osc = this.ctx.createOscillator();
            osc.type = 'sine';
            osc.frequency.setValueAtTime(1400, t);
            osc.frequency.exponentialRampToValueAtTime(600, t + 0.035);

            const g = this.ctx.createGain();
            g.gain.setValueAtTime(0.08, t);
            g.gain.exponentialRampToValueAtTime(0.001, t + 0.035);

            osc.connect(g);
            g.connect(this.masterGain);

            osc.start(t);
            osc.stop(t + 0.035);
        }

        /**
         * Catastrophic Crash Impact Sound (Heavy crunch + distorted metal thump)
         */
        playCrashSound() {
            this.ensureContext();
            if (!this.initialized || !this.ctx || this.muted) return;
            const t = this.ctx.currentTime;

            // Low heavy impact thud
            const osc = this.ctx.createOscillator();
            osc.type = 'sawtooth';
            osc.frequency.setValueAtTime(140, t);
            osc.frequency.exponentialRampToValueAtTime(18, t + 0.6);

            const g = this.ctx.createGain();
            g.gain.setValueAtTime(0.7, t);
            g.gain.exponentialRampToValueAtTime(0.001, t + 0.6);

            osc.connect(g);
            g.connect(this.masterGain);
            osc.start(t);
            osc.stop(t + 0.6);

            // High frequency metallic snap
            const snap = this.ctx.createOscillator();
            snap.type = 'square';
            snap.frequency.setValueAtTime(2400, t);
            snap.frequency.exponentialRampToValueAtTime(80, t + 0.2);

            const snapG = this.ctx.createGain();
            snapG.gain.setValueAtTime(0.4, t);
            snapG.gain.exponentialRampToValueAtTime(0.001, t + 0.2);

            snap.connect(snapG);
            snapG.connect(this.masterGain);
            snap.start(t);
            snap.stop(t + 0.2);
        }

        /**
         * Electrical Motor Failure POP / Spark
         */
        playMotorFailSound() {
            this.ensureContext();
            if (!this.initialized || !this.ctx || this.muted) return;
            const t = this.ctx.currentTime;

            const osc = this.ctx.createOscillator();
            osc.type = 'square';
            osc.frequency.setValueAtTime(3200, t);
            osc.frequency.exponentialRampToValueAtTime(60, t + 0.18);

            const g = this.ctx.createGain();
            g.gain.setValueAtTime(0.4, t);
            g.gain.exponentialRampToValueAtTime(0.001, t + 0.18);

            osc.connect(g);
            g.connect(this.masterGain);
            osc.start(t);
            osc.stop(t + 0.18);

            this.playWarningAlert();
        }

        /**
         * Avionics Warning Siren / Alert Beep
         */
        playWarningAlert() {
            this.ensureContext();
            if (!this.initialized || !this.ctx || this.muted) return;
            const t = this.ctx.currentTime;

            const osc = this.ctx.createOscillator();
            osc.type = 'square';
            osc.frequency.setValueAtTime(1200, t);
            osc.frequency.setValueAtTime(1600, t + 0.08);

            const g = this.ctx.createGain();
            g.gain.setValueAtTime(0.12, t);
            g.gain.exponentialRampToValueAtTime(0.001, t + 0.18);

            osc.connect(g);
            g.connect(this.masterGain);

            osc.start(t);
            osc.stop(t + 0.18);
        }

        setVolume(val) {
            this.masterVolume = Math.max(0, Math.min(1, val));
            if (this.masterGain && !this.muted) {
                this.masterGain.gain.setValueAtTime(this.masterVolume, this.ctx.currentTime);
            }
        }

        toggleMute() {
            this.muted = !this.muted;
            if (this.masterGain) {
                this.masterGain.gain.setValueAtTime(this.muted ? 0.0 : this.masterVolume, this.ctx.currentTime);
            }
            return this.muted;
        }
    }

    global.GarudaAudio = new GarudaAudioSystem();
})(typeof window !== 'undefined' ? window : this);
