"use strict";

const $ = id => document.getElementById(id);
const token = location.hash.slice(1) || sessionStorage.getItem("neuroexo-token") || "";
if (token) {
  sessionStorage.setItem("neuroexo-token", token);
  history.replaceState(null, "", location.pathname);
}
let state = null;
let online = false;
let actionPending = false;
let librarySignature = "";

async function request(path, options = {}) {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 3500);
  try {
    const response = await fetch(path, {
      ...options,
      signal: controller.signal,
      headers: {Authorization: `Bearer ${token}`, ...options.headers},
      cache: "no-store",
    });
    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.error || `Request failed: ${response.status}`);
    }
    return response;
  } finally {
    clearTimeout(timeout);
  }
}

function message(text, error = false) {
  $("message").textContent = text;
  $("message").classList.toggle("error", error);
}

async function action(name, args = {}) {
  // Stop remains available while any other request is in flight.
  actionPending = true;
  updateControls();
  try {
    await request("/api/action", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({action: name, ...args})});
    message(name === "stop" ? "Stop requested; waiting for device confirmation." : "Request accepted; waiting for device telemetry.");
    await pollOnce();
  } catch (error) {
    message(`${error.message}. ${name === "stop" ? "Stop is unconfirmed; check the computer and device." : "Check the connection to the Python computer."}`, true);
  } finally {
    actionPending = false;
    updateControls();
  }
}

function start(mode = $("mode").value) {
  const target = ["demo", "home"].includes(mode) ? 0 : Number($("target").value);
  if (!Number.isInteger(target) || target < -150 || target > 80 || (!["demo", "home"].includes(mode) && $("target").value.trim() === "")) {
    message("Enter a whole target angle from −150° to 80°.", true);
    return;
  }
  const setting = ["assistive", "resistive"].includes(mode) ? Number($("level").value) : mode === "calibrate" ? 0 : Number($("speed").value);
  action("start", {mode, setting, target});
}

$("connect").addEventListener("click", () => state?.connected
  ? action("disconnect") : action("connect", {source: $("source").value, identifier: $("identifier").value}));
$("start").addEventListener("click", () => start());
$("end").addEventListener("click", () => action("stop", {reason: "operator_ended"}));
$("stop").addEventListener("click", () => action("stop"));
$("demo").addEventListener("click", () => start("demo"));
$("home").addEventListener("click", () => start("home"));
$("calibrate").addEventListener("click", () => start("calibrate"));
$("level").addEventListener("input", () => { $("level-value").textContent = `${Number($("level").value) * 10}%`; });
$("mode").addEventListener("change", () => {
  $("level-label").textContent = $("mode").value === "resistive" ? "Resistance" : "Assistance";
  updateControls();
});

function updateControls() {
  const connected = online && state?.connected;
  const busy = Boolean(state?.active || state?.pending);
  const ready = Boolean(online && state?.can_start && !actionPending);
  if (state?.connected || state?.connecting) $("source").value = state.source;
  $("connect").disabled = !online || actionPending || Boolean(state?.connecting);
  $("connect").textContent = state?.connecting ? "Connecting…" : connected ? "Disconnect" : "Connect";
  $("source").disabled = Boolean(connected || state?.connecting);
  $("identifier").disabled = $("source").disabled;
  for (const id of ["mode", "target", "speed"]) $(id).disabled = busy || actionPending;
  $("level").disabled = busy || actionPending || $("mode").value === "neutral";
  $("start").disabled = !ready || ($("mode").value !== "neutral" && !state?.calibrated);
  for (const id of ["demo", "home", "calibrate"]) $(id).disabled = !ready;
  $("end").disabled = !connected || !busy;
  $("stop").disabled = !connected;
}

async function download(trial) {
  try {
    const response = await request(`/api/trials/${encodeURIComponent(trial.id)}`);
    const blob = await response.blob();
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = trial.file;
    document.body.appendChild(link);
    link.click();
    link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 30000);
  } catch (error) {
    message(`Export failed: ${error.message}`, true);
  }
}

function renderLibrary(trials) {
  const signature = JSON.stringify(trials);
  if (signature === librarySignature) return;
  librarySignature = signature;
  const container = $("trials");
  container.replaceChildren();
  if (!trials.length) {
    const empty = document.createElement("p");
    empty.className = "empty";
    empty.textContent = "Completed and interrupted trials will appear here.";
    container.appendChild(empty);
    return;
  }
  for (const trial of trials) {
    const row = document.createElement("div");
    row.className = "trial-row";
    const name = document.createElement("div");
    const title = document.createElement("p");
    title.textContent = `${trial.mode} · ${trial.source}`;
    const detail = document.createElement("small");
    detail.textContent = trial.id.replace(/^trial_/, "");
    name.append(title, detail);
    const samples = document.createElement("span");
    samples.className = "samples";
    samples.textContent = `${trial.samples} samples`;
    const result = document.createElement("span");
    result.className = `result ${trial.result === "target_reached" ? "" : "interrupted"}`;
    result.textContent = trial.result.replaceAll("_", " ");
    const button = document.createElement("button");
    button.textContent = "Export CSV ↓";
    button.addEventListener("click", () => download(trial));
    row.append(name, samples, result, button);
    container.appendChild(row);
  }
}

async function pollOnce() {
  const response = await request("/api/state");
  state = await response.json();
  online = true;
  const live = state.connected && state.fresh;
  $("badge").textContent = `${state.source === "simulation" ? "SIMULATOR" : state.source === "bluetooth" ? "BLUETOOTH" : "DEVICE"} · ${live ? "LIVE" : "OFFLINE"}`;
  $("badge").classList.toggle("live", live);
  $("motion").textContent = state.pending ? `Awaiting ${state.pending} confirmation` : state.active ? `${state.mode || "Trial"} · ${state.elapsed_s.toFixed(1)} s` : live ? "Ready · motor idle" : "Waiting for telemetry";
  $("angle").textContent = live ? `${state.angle.toFixed(1)}°` : "—";
  $("current").textContent = live ? `${state.current_a.toFixed(3)} A` : "—";
  $("effort").textContent = live ? `${state.effort_nm.toFixed(2)} N·m` : "—";
  $("calibration").textContent = state.calibrated ? "Calibrated" : "Not calibrated";
  message(state.message, Boolean(state.status & 8));
  updateControls();
  renderLibrary(state.trials);
  renderArm();
  renderChart();
}

async function poll() {
  try {
    await pollOnce();
  } catch (error) {
    online = false;
    $("badge").textContent = "COMPUTER OFFLINE";
    $("badge").classList.remove("live");
    $("motion").textContent = "Connection unavailable · last known pose";
    for (const id of ["angle", "current", "effort"]) $(id).textContent = "—";
    message(`${error.message}. Open the complete phone link and keep the Python app running.`, true);
    updateControls();
  }
  setTimeout(poll, 200);
}

// Real 3D geometry with camera projection and depth-sorted, shaded polygons.
// Encoder 0° is drawn as a straight elbow; this is a schematic, not a CAD model.
let azimuth = 35, elevation = 18, zoom = 1;
const armCanvas = $("arm");
let pointer = null;
armCanvas.addEventListener("pointerdown", event => {
  pointer = [event.clientX, event.clientY];
  armCanvas.setPointerCapture(event.pointerId);
});
armCanvas.addEventListener("pointermove", event => {
  if (!pointer) return;
  azimuth += (event.clientX - pointer[0]) * .5;
  elevation = Math.max(-75, Math.min(75, elevation + (event.clientY - pointer[1]) * .5));
  pointer = [event.clientX, event.clientY];
  renderArm();
});
armCanvas.addEventListener("pointerup", () => { pointer = null; });
armCanvas.addEventListener("pointercancel", () => { pointer = null; });
armCanvas.addEventListener("wheel", event => {
  event.preventDefault();
  zoom = Math.max(.5, Math.min(2.5, zoom * (event.deltaY < 0 ? 1.1 : .9)));
  renderArm();
}, {passive: false});
$("reset").addEventListener("click", () => { azimuth = 35; elevation = 18; zoom = 1; renderArm(); });

function prepare(canvas) {
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio)) {
    canvas.width = Math.round(w * ratio);
    canvas.height = Math.round(h * ratio);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return [ctx, w, h];
}

const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
function cylinder(a, b, radius, color) {
  let axis = b.map((v, i) => v-a[i]);
  const length = Math.hypot(...axis);
  axis = axis.map(v => v/length);
  let u = cross(axis, Math.abs(axis[1]) < .9 ? [0, 1, 0] : [1, 0, 0]);
  const norm = Math.hypot(...u);
  u = u.map(v => v/norm);
  const v = cross(axis, u), n = 16;
  const rings = [a, b].map(point => Array.from({length: n}, (_, i) => point.map((p, j) => p + radius*(u[j]*Math.cos(i*2*Math.PI/n)+v[j]*Math.sin(i*2*Math.PI/n)))));
  const faces = rings.map(points => ({points, color}));
  for (let i = 0; i < n; ++i) faces.push({points: [rings[0][i], rings[0][(i+1)%n], rings[1][(i+1)%n], rings[1][i]], color});
  return faces;
}

function renderArm() {
  const [ctx, w, h] = prepare(armCanvas);
  const az = azimuth*Math.PI/180, el = elevation*Math.PI/180;
  function project([x, y, z]) {
    const horizontal = x*Math.cos(az)-y*Math.sin(az);
    const away = x*Math.sin(az)+y*Math.cos(az);
    const vertical = z*Math.cos(el)-away*Math.sin(el);
    const depth = away*Math.cos(el)+z*Math.sin(el);
    const scale = Math.min(w, h)*.31*zoom*5/(5-depth);
    return [w/2+horizontal*scale, h/2-vertical*scale, depth];
  }
  function line(a, b, color, dash = []) {
    a = project(a); b = project(b);
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
    ctx.strokeStyle = color; ctx.setLineDash(dash); ctx.stroke(); ctx.setLineDash([]);
  }
  ctx.lineWidth = 1;
  for (let i = -4; i <= 4; ++i) {
    line([i*.35,-1.4,-1.4], [i*.35,1.4,-1.4], "#20364d");
    line([-1.4,i*.35,-1.4], [1.4,i*.35,-1.4], "#20364d");
  }
  const angle = (state?.angle || 0)*Math.PI/180;
  const along = t => [t*Math.sin(angle), 0, -t*Math.cos(angle)];
  const faces = [
    ...cylinder([0,0,.06], [0,0,1.2], .105, "#8196ad"),
    ...cylinder([0,0,.8], [0,0,1.02], .15, "#293f58"),
    ...cylinder(along(.07), along(1.05), .1, "#49c8b7"),
    ...cylinder(along(.64), along(.85), .145, "#254e59"),
    ...cylinder([0,-.15,0], [0,.15,0], .19, "#d2deeb"),
    ...cylinder([0,-.17,0], [0,-.15,0], .105, "#298f87"),
    ...cylinder(along(1.05), along(1.3), .075, "#a1b5c9"),
  ].map(face => {
    const points = face.points.map(project);
    return {...face, points, depth: points.reduce((s,p) => s+p[2],0)/points.length};
  }).sort((a,b) => a.depth-b.depth);
  for (const face of faces) {
    const shade = Math.max(.62, Math.min(1.12, .86+face.depth*.16));
    const rgb = [1,3,5].map(i => Math.min(255, Math.round(parseInt(face.color.slice(i,i+2),16)*shade)));
    ctx.fillStyle = `rgb(${rgb.join(",")})`;
    ctx.beginPath(); face.points.forEach((p,i) => i ? ctx.lineTo(p[0],p[1]) : ctx.moveTo(p[0],p[1])); ctx.closePath(); ctx.fill();
  }
  if (state?.target !== null && state?.target !== undefined) {
    const target = state.target*Math.PI/180;
    ctx.lineWidth = 2;
    line([0,0,0], [Math.sin(target)*1.3,0,-Math.cos(target)*1.3], "#f5bb69", [5,4]);
  }
}

function renderChart() {
  const [ctx,w,h] = prepare($("chart"));
  const history = state?.history || [];
  ctx.font = "10px Segoe UI, system-ui";
  ctx.fillStyle = "#91a6bc";
  if (history.length < 2) { ctx.fillText("Trial angle history appears here", 14, h/2); return; }
  const values = history.map(p => p[1]);
  const low = Math.min(...values)-5, high = Math.max(...values)+5;
  for (let i = 0; i < 4; ++i) {
    const y = 16+i*(h-40)/3;
    ctx.strokeStyle = "#24364b"; ctx.beginPath(); ctx.moveTo(36,y); ctx.lineTo(w-8,y); ctx.stroke();
    ctx.fillStyle = "#91a6bc"; ctx.fillText(`${Math.round(high-(high-low)*i/3)}°`,0,y+3);
  }
  const duration = Math.max(1, history.at(-1)[0]-history[0][0]);
  ctx.beginPath();
  history.forEach((p,i) => {
    const x = 36+(p[0]-history[0][0])/duration*(w-48);
    const y = h-24-(p[1]-low)/(high-low)*(h-40);
    if (i) ctx.lineTo(x,y); else ctx.moveTo(x,y);
  });
  ctx.strokeStyle = "#49c8b7"; ctx.lineWidth = 2; ctx.stroke();
  ctx.fillStyle = "#91a6bc"; ctx.fillText(`${history.at(-1)[0].toFixed(1)} s`, w-50, h-4);
}

new ResizeObserver(() => { renderArm(); renderChart(); }).observe($("arm"));
renderArm(); renderChart(); updateControls(); poll();
