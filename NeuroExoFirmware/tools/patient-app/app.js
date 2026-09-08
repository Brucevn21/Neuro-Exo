/*
 * NeuroExo Rehab Trainer - patient-facing session flow over Web Bluetooth.
 * Talks to Nano33BLEFirmware.ino (service 180C / command 2A56 / telemetry 2A57),
 * which bridges to the Teensy joint controller over I2C.
 */

// Mechanical travel limits from motorLimit in src/main.cpp, used only to draw
// the reference arc on the dial (matches tools/visualizer's DEFAULT_MIN/MAX_DEG).
const JOINT_MIN_DEG = -150;
const JOINT_MAX_DEG = 80;

const TARGET_MIN_DEG = 20;
const TARGET_MAX_DEG = 120;

// A trial counts as "reached" once the encoder stays within this tolerance
// of the target for a short, debounced stretch (telemetry arrives at 20Hz).
const REACHED_TOLERANCE_DEG = 4;
const REACHED_HOLD_SAMPLES = 5;

// Mode isn't yet differentiated by the firmware's control algorithm, so this
// stays a single constant here rather than exposing a choice the device can't
// actually honor differently.
const SESSION_MODE = NeuroExoProtocol.Mode.Assistive;

const HISTORY_SECONDS = 8;

const el = (id) => document.getElementById(id);

const screens = {
  setup: el('screen-setup'),
  trial: el('screen-trial'),
  success: el('screen-success'),
};

function showScreen(name) {
  for (const key of Object.keys(screens)) {
    screens[key].classList.toggle('active', key === name);
  }
}

// ---------- BLE layer ----------

class JointBleClient {
  constructor() {
    this.device = null;
    this.commandChar = null;
    this.telemetryChar = null;
    this.onTelemetry = null;
    this.onDisconnected = null;
  }

  async connect() {
    this.device = await navigator.bluetooth.requestDevice({
      filters: [{ services: [NeuroExoProtocol.SERVICE_UUID] }],
    });

    this.device.addEventListener('gattserverdisconnected', () => {
      if (this.onDisconnected) this.onDisconnected();
    });

    const server = await this.device.gatt.connect();
    const service = await server.getPrimaryService(NeuroExoProtocol.SERVICE_UUID);
    this.commandChar = await service.getCharacteristic(NeuroExoProtocol.COMMAND_CHAR_UUID);
    this.telemetryChar = await service.getCharacteristic(NeuroExoProtocol.TELEMETRY_CHAR_UUID);

    await this.telemetryChar.startNotifications();
    this.telemetryChar.addEventListener('characteristicvaluechanged', (event) => {
      const packet = NeuroExoProtocol.decodeJointPacket(event.target.value);
      if (this.onTelemetry) this.onTelemetry(packet);
    });

    return this.device.name || 'NeuroExo device';
  }

  async sendJointPacket(packet) {
    const buf = NeuroExoProtocol.encodeJointPacket(packet);
    await this.commandChar.writeValue(buf);
  }

  disconnect() {
    if (this.device && this.device.gatt.connected) {
      this.device.gatt.disconnect();
    }
  }
}

// ---------- Dial rendering (mirrors tools/visualizer/joint_visualizer.py) ----------

function polar(deg, radius) {
  const rad = (deg * Math.PI) / 180;
  return [radius * Math.cos(rad), -radius * Math.sin(rad)];
}

function describeArc(minDeg, maxDeg, radius) {
  const [x1, y1] = polar(minDeg, radius);
  const [x2, y2] = polar(maxDeg, radius);
  const largeArc = Math.abs(maxDeg - minDeg) > 180 ? 1 : 0;
  return `M ${x1} ${y1} A ${radius} ${radius} 0 ${largeArc} 0 ${x2} ${y2}`;
}

function setupDial() {
  const svg = el('dial');
  const NS = 'http://www.w3.org/2000/svg';

  const limitArc = document.createElementNS(NS, 'path');
  limitArc.setAttribute('d', describeArc(JOINT_MIN_DEG, JOINT_MAX_DEG, 105));
  limitArc.setAttribute('class', 'dial-limit-arc');
  svg.appendChild(limitArc);

  const base = document.createElementNS(NS, 'circle');
  base.setAttribute('cx', 0);
  base.setAttribute('cy', 0);
  base.setAttribute('r', 6);
  base.setAttribute('class', 'dial-base');
  svg.appendChild(base);

  const targetLine = document.createElementNS(NS, 'line');
  targetLine.setAttribute('class', 'dial-target-line');
  svg.appendChild(targetLine);

  const currentLine = document.createElementNS(NS, 'line');
  currentLine.setAttribute('class', 'dial-current-line');
  svg.appendChild(currentLine);

  return { targetLine, currentLine };
}

function updateDial(dial, currentDeg, targetDeg) {
  const [cx, cy] = polar(currentDeg, 95);
  dial.currentLine.setAttribute('x1', 0);
  dial.currentLine.setAttribute('y1', 0);
  dial.currentLine.setAttribute('x2', cx);
  dial.currentLine.setAttribute('y2', cy);

  const [tx, ty] = polar(targetDeg, 108);
  dial.targetLine.setAttribute('x1', 0);
  dial.targetLine.setAttribute('y1', 0);
  dial.targetLine.setAttribute('x2', tx);
  dial.targetLine.setAttribute('y2', ty);
}

// ---------- History chart (plain canvas, no dependencies) ----------

class HistoryChart {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.samples = []; // {t, current, target}
    this.startTime = null;
  }

  reset() {
    this.samples = [];
    this.startTime = null;
  }

  push(currentDeg, targetDeg) {
    const now = performance.now() / 1000;
    if (this.startTime === null) this.startTime = now;
    const t = now - this.startTime;
    this.samples.push({ t, current: currentDeg, target: targetDeg });
    const cutoff = t - HISTORY_SECONDS;
    while (this.samples.length && this.samples[0].t < cutoff) this.samples.shift();
    this.draw();
  }

  draw() {
    const { ctx, canvas } = this;
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    if (this.samples.length < 2) return;

    const tMax = this.samples[this.samples.length - 1].t;
    const tMin = Math.max(0, tMax - HISTORY_SECONDS);
    let yMin = Math.min(...this.samples.map((s) => Math.min(s.current, s.target)));
    let yMax = Math.max(...this.samples.map((s) => Math.max(s.current, s.target)));
    if (yMax - yMin < 10) {
      const mid = (yMax + yMin) / 2;
      yMin = mid - 5;
      yMax = mid + 5;
    }
    const pad = (yMax - yMin) * 0.1;
    yMin -= pad;
    yMax += pad;

    const xOf = (t) => ((t - tMin) / (tMax - tMin || 1)) * w;
    const yOf = (v) => h - ((v - yMin) / (yMax - yMin || 1)) * h;

    const drawLine = (key, style) => {
      ctx.beginPath();
      ctx.strokeStyle = style;
      ctx.lineWidth = 2;
      this.samples.forEach((s, i) => {
        const x = xOf(s.t);
        const y = yOf(s[key]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    };

    drawLine('target', '#e0554f');
    drawLine('current', '#2f6fed');
  }
}

// ---------- App state machine ----------

const ble = new JointBleClient();
const dial = setupDial();
const chart = new HistoryChart(el('historyChart'));

let selectedSpeed = null;
let trialNumber = 0;
let currentTarget = null;
let initialDistance = 0;
let reachedStreak = 0;
let latestCurrentAngle = 0;

function setConnected(connected, label) {
  el('connDot').classList.toggle('connected', connected);
  el('connLabel').textContent = label;
  refreshStartEnabled();
}

function refreshStartEnabled() {
  const ready = selectedSpeed !== null && ble.device && ble.device.gatt.connected;
  el('startBtn').disabled = !ready;
  el('setupHint').textContent = ready
    ? 'Ready when you are.'
    : 'Select a speed and connect the device to begin.';
}

el('speedGroup').addEventListener('click', (event) => {
  const btn = event.target.closest('.speed-btn');
  if (!btn) return;
  selectedSpeed = Number(btn.dataset.speed);
  document.querySelectorAll('.speed-btn').forEach((b) => b.classList.toggle('selected', b === btn));
  refreshStartEnabled();
});

el('connectBtn').addEventListener('click', async () => {
  try {
    el('connectBtn').disabled = true;
    el('connectBtn').textContent = 'Connecting...';
    ble.onDisconnected = () => {
      setConnected(false, 'Disconnected');
      el('connectBtn').disabled = false;
      el('connectBtn').textContent = 'Connect Device';
    };
    ble.onTelemetry = handleTelemetry;
    const name = await ble.connect();
    setConnected(true, `Connected: ${name}`);
    el('connectBtn').textContent = 'Connected';
  } catch (err) {
    console.error(err);
    setConnected(false, 'Not connected');
    el('connectBtn').disabled = false;
    el('connectBtn').textContent = 'Connect Device';
    if (err.name !== 'NotFoundError') {
      alert(`Could not connect: ${err.message}`);
    }
  }
});

el('startBtn').addEventListener('click', () => {
  trialNumber = 0;
  startNextTrial();
});

el('nextTrialBtn').addEventListener('click', () => {
  startNextTrial();
});

el('quitBtn').addEventListener('click', endSession);
el('quitDuringTrialBtn').addEventListener('click', endSession);

function endSession() {
  showScreen('setup');
  el('setupHint').textContent = 'Session ended. Select a speed to start another.';
}

function randomTargetAngle() {
  return Math.floor(Math.random() * (TARGET_MAX_DEG - TARGET_MIN_DEG + 1)) + TARGET_MIN_DEG;
}

async function startNextTrial() {
  trialNumber += 1;
  currentTarget = randomTargetAngle();
  reachedStreak = 0;
  initialDistance = Math.abs(latestCurrentAngle - currentTarget) || 1;

  el('trialCounter').textContent = `Trial ${trialNumber}`;
  el('targetReadout').textContent = currentTarget;
  el('targetReadout2').textContent = `${currentTarget}°`;
  el('progressFill').style.width = '0%';
  el('progressLabel').textContent = '0% there';
  chart.reset();

  showScreen('trial');

  try {
    await ble.sendJointPacket({
      mode: SESSION_MODE,
      speed: selectedSpeed,
      targetAngleDeg: currentTarget,
    });
  } catch (err) {
    console.error(err);
    alert(`Could not send command to device: ${err.message}`);
  }
}

function handleTelemetry(packet) {
  latestCurrentAngle = packet.currentAngleDeg;

  el('currentReadout').textContent = `${packet.currentAngleDeg}°`;
  updateDial(dial, packet.currentAngleDeg, currentTarget ?? packet.targetAngleDeg);

  if (screens.trial.classList.contains('active') && currentTarget !== null) {
    chart.push(packet.currentAngleDeg, currentTarget);

    const distance = Math.abs(packet.currentAngleDeg - currentTarget);
    const progress = Math.min(1, Math.max(0, 1 - distance / initialDistance));
    el('progressFill').style.width = `${Math.round(progress * 100)}%`;
    el('progressLabel').textContent = `${Math.round(progress * 100)}% there`;

    if (distance <= REACHED_TOLERANCE_DEG) {
      reachedStreak += 1;
      if (reachedStreak >= REACHED_HOLD_SAMPLES) {
        onTargetReached();
      }
    } else {
      reachedStreak = 0;
    }
  }
}

function onTargetReached() {
  currentTarget = null;
  el('successSubtitle').textContent = `You reached the target angle on trial ${trialNumber}.`;
  showScreen('success');
}

if (!navigator.bluetooth) {
  el('connectBtn').disabled = true;
  el('setupHint').textContent = 'Web Bluetooth is not available in this browser. Use Chrome or Edge on desktop or Android.';
}

showScreen('setup');
