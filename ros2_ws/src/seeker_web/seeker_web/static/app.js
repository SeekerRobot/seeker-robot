// app.js — Seeker controller frontend.
// Opens a WebSocket to /ws for live telemetry; sends commands either over the
// same socket or via the REST endpoints (both work). Pulls camera/mic URLs
// from /api/config so the HTML doesn't need to hardcode the robot's IP.

"use strict";

// ---------------------------------------------------------------------------
// State + DOM handles
// ---------------------------------------------------------------------------
const $ = (id) => document.getElementById(id);

const wsPill = $("ws-pill");
const hbPill = $("hb-pill");
const battPill = $("batt-pill");
const mcuPill = $("mcu-pill");

const imuOri = $("imu-ori");
const imuAng = $("imu-ang");
const imuAcc = $("imu-acc");

const lidarCanvas = $("lidar-canvas");
const lidarCtx = lidarCanvas.getContext("2d");

const logView = $("log-view");

const cmdReadout = $("cmd-readout");
const linMax = $("lin-max");
const yawMax = $("yaw-max");
const linMaxLbl = $("lin-max-lbl");
const yawMaxLbl = $("yaw-max-lbl");
linMax.addEventListener("input", () => (linMaxLbl.textContent = (+linMax.value).toFixed(2)));
yawMax.addEventListener("input", () => (yawMaxLbl.textContent = (+yawMax.value).toFixed(2)));

let ws = null;
let wsReady = false;
let cmd = { vx: 0, vy: 0, wz: 0 };
let lastHeartbeat = null;
let lastHeartbeatTime = 0;

// ---------------------------------------------------------------------------
// Config fetch — populate camera/mic URLs
// ---------------------------------------------------------------------------
fetch("/api/config")
  .then((r) => r.json())
  .then((cfg) => {
    $("cam-img").src = cfg.cam_stream_url || cfg.cam_url;
    $("cam-url-note").textContent = cfg.cam_stream_url || cfg.cam_url;
    $("mic-url-note").textContent = cfg.mic_url;
    $("mic-btn").addEventListener("click", () => {
      const audio = $("mic-audio");
      audio.src = cfg.mic_url;
      audio.play().catch((err) => console.warn("mic play failed", err));
    });
    mcuPill.textContent = `MCU: ${cfg.mcu_ip}`;
  })
  .catch((err) => {
    console.warn("config fetch failed", err);
    mcuPill.textContent = "MCU: ?";
  });

// ---------------------------------------------------------------------------
// WebSocket
// ---------------------------------------------------------------------------
function connectWs() {
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
  ws = new WebSocket(`${scheme}//${location.host}/ws`);
  ws.addEventListener("open", () => {
    wsReady = true;
    wsPill.textContent = "WS: OK";
    wsPill.className = "pill ok";
  });
  ws.addEventListener("close", () => {
    wsReady = false;
    wsPill.textContent = "WS: OFF";
    wsPill.className = "pill err";
    setTimeout(connectWs, 1000);
  });
  ws.addEventListener("error", () => {
    wsPill.textContent = "WS: ERR";
    wsPill.className = "pill warn";
  });
  ws.addEventListener("message", (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    switch (msg.type) {
      case "status":         onStatus(msg); break;
      case "imu":            onImu(msg); break;
      case "lidar":          onLidar(msg); break;
      case "log":            onLog(msg); break;
      case "speaker_active": onSpeakerActive(msg); break;
    }
  });
}
connectWs();

function sendWs(obj) {
  if (wsReady && ws.readyState === 1) ws.send(JSON.stringify(obj));
}

// ---------------------------------------------------------------------------
// Telemetry handlers
// ---------------------------------------------------------------------------
function onStatus(msg) {
  if (msg.battery != null) {
    const v = msg.battery.toFixed(2);
    battPill.textContent = `BATT: ${v} V`;
    battPill.className = "pill " + (msg.battery < 11.1 ? "err" : msg.battery < 11.4 ? "warn" : "ok");
  }
  if (msg.heartbeat != null) {
    if (msg.heartbeat !== lastHeartbeat) {
      lastHeartbeat = msg.heartbeat;
      lastHeartbeatTime = performance.now();
    }
    const stale = (performance.now() - lastHeartbeatTime) / 1000;
    hbPill.textContent = `HB: ${msg.heartbeat}`;
    hbPill.className = "pill " + (stale > 3 ? "err" : "ok");
  }
}

function onImu(msg) {
  const f = (a) => a.map((x) => (+x).toFixed(3)).join("  ");
  imuOri.textContent = f(msg.ori);
  imuAng.textContent = f(msg.ang);
  imuAcc.textContent = f(msg.acc);
}

function onLog(msg) {
  const max = 200;
  logView.textContent += msg.text + "\n";
  const lines = logView.textContent.split("\n");
  if (lines.length > max) logView.textContent = lines.slice(-max).join("\n");
  logView.scrollTop = logView.scrollHeight;
}
$("log-clear").addEventListener("click", () => (logView.textContent = ""));

// Mute the mic audio element while the speaker is actively playing. Muting
// (not .pause()) is deliberate — we keep the stream socket hot so the browser
// doesn't reconnect and refetch the PCM endpoint at every TTS clip.
function onSpeakerActive(msg) {
  const audio = $("mic-audio");
  if (!audio) return;
  audio.muted = !!msg.active;
}

// ---------------------------------------------------------------------------
// Lidar polar plot
// ---------------------------------------------------------------------------
function onLidar(msg) {
  const W = lidarCanvas.width, H = lidarCanvas.height;
  const cx = W / 2, cy = H / 2;
  const rMax = Math.min(W, H) / 2 - 10;
  const maxRange = 3.0; // meters — scale beyond this is clipped

  lidarCtx.clearRect(0, 0, W, H);

  // Grid rings
  lidarCtx.strokeStyle = "#21262d";
  lidarCtx.lineWidth = 1;
  for (let f = 0.25; f <= 1.0; f += 0.25) {
    lidarCtx.beginPath();
    lidarCtx.arc(cx, cy, rMax * f, 0, Math.PI * 2);
    lidarCtx.stroke();
  }
  // Heading indicator (forward = +x)
  lidarCtx.strokeStyle = "#30363d";
  lidarCtx.beginPath();
  lidarCtx.moveTo(cx, cy);
  lidarCtx.lineTo(cx + rMax, cy);
  lidarCtx.stroke();

  // Points
  lidarCtx.fillStyle = "#58a6ff";
  for (let i = 0; i < msg.angles.length; i++) {
    const r = msg.ranges[i];
    if (!isFinite(r) || r <= 0 || r > maxRange) continue;
    const a = msg.angles[i];
    // LaserScan frame: x forward, y left, angle CCW from +x. Screen +y down,
    // so flip the y component.
    const px = cx + Math.cos(a) * (r / maxRange) * rMax;
    const py = cy - Math.sin(a) * (r / maxRange) * rMax;
    lidarCtx.fillRect(px - 1, py - 1, 2, 2);
  }
}

// ---------------------------------------------------------------------------
// Virtual joystick
// ---------------------------------------------------------------------------
const joy = $("joystick");
const joyCtx = joy.getContext("2d");
let joyActive = false;
let joyVec = { x: 0, y: 0 };

function drawJoystick() {
  const W = joy.width, H = joy.height;
  const cx = W / 2, cy = H / 2, r = Math.min(W, H) / 2 - 4;
  joyCtx.clearRect(0, 0, W, H);

  joyCtx.strokeStyle = "#30363d";
  joyCtx.lineWidth = 2;
  joyCtx.beginPath();
  joyCtx.arc(cx, cy, r, 0, Math.PI * 2);
  joyCtx.stroke();

  joyCtx.strokeStyle = "#21262d";
  joyCtx.beginPath();
  joyCtx.moveTo(cx - r, cy); joyCtx.lineTo(cx + r, cy);
  joyCtx.moveTo(cx, cy - r); joyCtx.lineTo(cx, cy + r);
  joyCtx.stroke();

  const knobX = cx + joyVec.x * r;
  const knobY = cy + joyVec.y * r;
  joyCtx.fillStyle = joyActive ? "#58a6ff" : "#30363d";
  joyCtx.beginPath();
  joyCtx.arc(knobX, knobY, 18, 0, Math.PI * 2);
  joyCtx.fill();
}

function joyFromEvent(e) {
  const rect = joy.getBoundingClientRect();
  const t = e.touches ? e.touches[0] : e;
  const x = (t.clientX - rect.left) / rect.width;
  const y = (t.clientY - rect.top) / rect.height;
  // normalize to [-1, 1] with the center at 0
  let vx = x * 2 - 1;
  let vy = y * 2 - 1;
  const m = Math.hypot(vx, vy);
  if (m > 1) { vx /= m; vy /= m; }
  joyVec.x = vx;
  joyVec.y = vy;
}

function joyEnd() {
  joyActive = false;
  joyVec = { x: 0, y: 0 };
  drawJoystick();
}

joy.addEventListener("mousedown", (e) => { joyActive = true; joyFromEvent(e); drawJoystick(); });
joy.addEventListener("mousemove", (e) => { if (joyActive) { joyFromEvent(e); drawJoystick(); } });
window.addEventListener("mouseup", joyEnd);
joy.addEventListener("touchstart", (e) => { e.preventDefault(); joyActive = true; joyFromEvent(e); drawJoystick(); });
joy.addEventListener("touchmove", (e) => { e.preventDefault(); if (joyActive) { joyFromEvent(e); drawJoystick(); } });
joy.addEventListener("touchend", (e) => { e.preventDefault(); joyEnd(); });

drawJoystick();

// ---------------------------------------------------------------------------
// Keyboard
// ---------------------------------------------------------------------------
const keys = new Set();
window.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT" || e.target.tagName === "TEXTAREA") return;
  if (e.code === "Space") { sendWs({ type: "stop" }); return; }
  keys.add(e.code);
});
window.addEventListener("keyup", (e) => keys.delete(e.code));

function keyboardVec() {
  let vx = 0, wz = 0;
  if (keys.has("KeyW")) vx += 1;
  if (keys.has("KeyS")) vx -= 1;
  if (keys.has("KeyA")) wz += 1;
  if (keys.has("KeyD")) wz -= 1;
  if (keys.has("KeyQ")) wz += 1;
  if (keys.has("KeyE")) wz -= 1;
  return { vx, vy: 0, wz };
}

// ---------------------------------------------------------------------------
// Command tick — 20 Hz, blends joystick + keyboard into one Twist
// ---------------------------------------------------------------------------
setInterval(() => {
  const lin = +linMax.value;
  const yaw = +yawMax.value;

  // Joystick: y up = forward (vx+), x right = yaw right (wz-)
  let jvx = -joyVec.y;
  let jwz = -joyVec.x;

  const k = keyboardVec();
  const vx = (jvx + k.vx) * lin;
  const wz = (jwz + k.wz) * yaw;
  const vy = 0;

  // Clamp
  const c = (v, m) => Math.max(-m, Math.min(m, v));
  cmd.vx = c(vx, lin);
  cmd.vy = c(vy, lin);
  cmd.wz = c(wz, yaw);

  cmdReadout.textContent =
    `vx=${cmd.vx.toFixed(3)}  vy=${cmd.vy.toFixed(3)}  wz=${cmd.wz.toFixed(3)}`;
  sendWs({ type: "cmd_vel", vx: cmd.vx, vy: cmd.vy, wz: cmd.wz });
}, 50);

// ---------------------------------------------------------------------------
// Forms
// ---------------------------------------------------------------------------
$("estop-btn").addEventListener("click", () => {
  sendWs({ type: "stop" });
  fetch("/api/stop", { method: "POST" });
});

$("tts-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const text = $("tts-text").value.trim();
  if (!text) return;
  await fetch("/api/tts", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ text }),
  });
});

$("wav-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const path = $("wav-path").value.trim();
  if (!path) return;
  await fetch("/api/play_wav", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ path }),
  });
});
