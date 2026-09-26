"use strict";

const FIELD_LENGTH_M = 16.46;
const FIELD_WIDTH_M = 8.23;
const VALID_TASKS = new Set([
  "PICKUP", "SCORE_SWITCH", "SCORE_SCALE", "DELIVER_VAULT",
  "CROSS_LINE", "PARK", "CLIMB", "WAIT",
]);
const FLOAT_KEYS = new Set([
  "x", "y", "heading", "length_m", "width_m", "max_velocity_mps",
  "max_acceleration_mps2", "initial_confidence", "corridor_radius_m",
]);
const ROBOT_COLORS = ["#2dd4ff", "#a879ff"];

function pythonFloat(value) {
  if (!Number.isFinite(value)) throw new Error("Kanonik belgede sonlu olmayan sayı var.");
  if (Object.is(value, -0)) return "-0.0";
  if (Number.isInteger(value)) return `${value}.0`;
  const magnitude = Math.abs(value);
  let text = magnitude !== 0 && (magnitude < 1e-4 || magnitude >= 1e16)
    ? value.toExponential()
    : String(value);
  return text.replace(/e([+-]?)(\d+)$/i, (_, sign, exponent) => {
    const normalizedSign = sign || "+";
    return `e${normalizedSign}${exponent.padStart(2, "0")}`;
  });
}

function canonicalStringify(value, key = "") {
  if (value === null) return "null";
  if (typeof value === "string") return JSON.stringify(value);
  if (typeof value === "boolean") return value ? "true" : "false";
  if (typeof value === "number") {
    return FLOAT_KEYS.has(key) ? pythonFloat(value) : String(Math.trunc(value));
  }
  if (Array.isArray(value)) {
    return `[${value.map((item) => canonicalStringify(item, key)).join(",")}]`;
  }
  const keys = Object.keys(value).sort();
  return `{${keys.map((itemKey) => `${JSON.stringify(itemKey)}:${canonicalStringify(value[itemKey], itemKey)}`).join(",")}}`;
}

async function sha256Hex(text) {
  const digest = await globalThis.crypto.subtle.digest(
    "SHA-256",
    new TextEncoder().encode(text),
  );
  return Array.from(new Uint8Array(digest), (byte) => byte.toString(16).padStart(2, "0")).join("");
}

function round(value, digits = 6) {
  const scale = 10 ** digits;
  return Math.round((value + Number.EPSILON) * scale) / scale;
}

function makeSegment(index, startSeconds = 0) {
  return {
    id: `segment-${index + 1}`,
    task: "CROSS_LINE",
    targetId: "auto-line",
    startSeconds,
    durationSeconds: 2,
    corridorRadius: 0.2,
    points: [],
  };
}

function makeRobot(index) {
  return {
    teamNumber: index === 0 ? 254 : 1678,
    label: index === 0 ? "ally-left" : "ally-right",
    footprintLength: 0.9,
    footprintWidth: 0.9,
    maxVelocity: 2,
    maxAcceleration: 4,
    initialConfidence: index === 0 ? 0.85 : 0.75,
    segments: [makeSegment(0)],
  };
}

function initialState() {
  return {
    planId: "demo-blue-alliance",
    alliance: "blue",
    redPreview: false,
    robots: [makeRobot(0), makeRobot(1)],
  };
}

function canonicalPayload(state) {
  return {
    schema_version: 1,
    field_version: "2018-power-up-v1",
    alliance: state.alliance,
    plan_id: state.planId.trim(),
    robots: state.robots.map((robot) => {
      const firstPoint = robot.segments[0]?.points[0] || { x: 0, y: 0, heading: 0 };
      return {
        team_number: Math.trunc(robot.teamNumber),
        robot_label: robot.label.trim(),
        start_pose: {
          x: round(firstPoint.x),
          y: round(firstPoint.y),
          heading: round(firstPoint.heading),
        },
        footprint: {
          length_m: round(robot.footprintLength),
          width_m: round(robot.footprintWidth),
        },
        max_velocity_mps: round(robot.maxVelocity),
        max_acceleration_mps2: round(robot.maxAcceleration),
        initial_confidence: round(robot.initialConfidence),
        segments: robot.segments.map((segment) => ({
          segment_id: segment.id.trim(),
          task: segment.task,
          target_id: segment.targetId.trim(),
          earliest_start_us: Math.round(segment.startSeconds * 1e6),
          latest_start_us: Math.round(segment.startSeconds * 1e6),
          expected_duration_us: Math.round(segment.durationSeconds * 1e6),
          corridor_radius_m: round(segment.corridorRadius),
          fallback_segment_id: "",
          path: segment.points.map((point) => ({
            time_us: Math.round(point.timeUs),
            x: round(point.x),
            y: round(point.y),
            heading: round(point.heading),
          })),
        })),
      };
    }),
  };
}

function validateState(state) {
  const errors = [];
  if (!state.planId.trim()) errors.push("Plan kimliği boş olamaz.");
  if (!new Set(["blue", "red"]).has(state.alliance)) errors.push("İttifak blue veya red olmalı.");
  if (state.robots.length < 1 || state.robots.length > 2) errors.push("Plan bir veya iki takım robotu içermeli.");
  const teamNumbers = new Set();
  const labels = new Set();
  state.robots.forEach((robot) => {
    const prefix = `#${robot.teamNumber || "?"}`;
    if (!Number.isInteger(robot.teamNumber) || robot.teamNumber <= 0) errors.push(`${prefix}: takım numarası geçersiz.`);
    if (teamNumbers.has(robot.teamNumber)) errors.push(`${prefix}: takım numarası tekrarlanıyor.`);
    teamNumbers.add(robot.teamNumber);
    if (!robot.label.trim()) errors.push(`${prefix}: robot etiketi boş.`);
    if (labels.has(robot.label.trim())) errors.push(`${prefix}: robot etiketi tekrarlanıyor.`);
    labels.add(robot.label.trim());
    if (robot.footprintLength <= 0 || robot.footprintWidth <= 0) errors.push(`${prefix}: gövde ölçüleri pozitif olmalı.`);
    if (robot.maxVelocity <= 0 || robot.maxAcceleration <= 0) errors.push(`${prefix}: hareket limitleri pozitif olmalı.`);
    if (robot.initialConfidence < 0 || robot.initialConfidence > 1) errors.push(`${prefix}: güven [0,1] aralığında olmalı.`);
    if (!robot.segments.length) errors.push(`${prefix}: en az bir segment gerekli.`);
    const ids = new Set();
    let previousEnd = -1;
    robot.segments.forEach((segment) => {
      const name = `${prefix}/${segment.id || "segment"}`;
      if (!segment.id.trim() || ids.has(segment.id.trim())) errors.push(`${name}: segment kimliği boş veya tekrarlı.`);
      ids.add(segment.id.trim());
      if (!VALID_TASKS.has(segment.task)) errors.push(`${name}: görev tipi desteklenmiyor.`);
      if (segment.startSeconds < 0 || segment.durationSeconds <= 0) errors.push(`${name}: zaman aralığı geçersiz.`);
      if (segment.corridorRadius < 0) errors.push(`${name}: koridor yarıçapı negatif olamaz.`);
      if (segment.points.length < 2) errors.push(`${name}: en az iki yol noktası gerekli.`);
      const startUs = Math.round(segment.startSeconds * 1e6);
      const endUs = startUs + Math.round(segment.durationSeconds * 1e6);
      if (startUs < previousEnd) errors.push(`${name}: önceki segmentle zaman çakışması var.`);
      previousEnd = endUs;
      let previousVelocity = null;
      segment.points.forEach((point, index) => {
        if (![point.x, point.y, point.heading, point.timeUs].every(Number.isFinite)) {
          errors.push(`${name}: sonlu olmayan yol noktası var.`);
          return;
        }
        if (point.x < 0 || point.x > FIELD_LENGTH_M || point.y < 0 || point.y > FIELD_WIDTH_M) {
          errors.push(`${name}: ${index + 1}. nokta alan dışında.`);
        }
        if (point.timeUs < startUs || point.timeUs > endUs) errors.push(`${name}: nokta zamanı segment dışında.`);
        if (index > 0) {
          const before = segment.points[index - 1];
          const dt = (point.timeUs - before.timeUs) / 1e6;
          if (dt <= 0) {
            errors.push(`${name}: nokta zamanları kesin artmalı.`);
          } else {
            const vx = (point.x - before.x) / dt;
            const vy = (point.y - before.y) / dt;
            if (Math.hypot(vx, vy) > robot.maxVelocity + 1e-9) errors.push(`${name}: azami hız aşılıyor.`);
            if (previousVelocity) {
              const acceleration = Math.hypot(vx - previousVelocity.vx, vy - previousVelocity.vy) /
                ((dt + previousVelocity.dt) / 2);
              if (acceleration > robot.maxAcceleration + 1e-9) errors.push(`${name}: azami ivme aşılıyor.`);
            }
            previousVelocity = { vx, vy, dt };
          }
        }
      });
    });
  });
  return [...new Set(errors)];
}

function interpolatePose(points, timeUs) {
  if (timeUs <= points[0].timeUs) return points[0];
  if (timeUs >= points[points.length - 1].timeUs) return points[points.length - 1];
  for (let index = 1; index < points.length; index += 1) {
    const second = points[index];
    if (timeUs <= second.timeUs) {
      const first = points[index - 1];
      const ratio = (timeUs - first.timeUs) / (second.timeUs - first.timeUs);
      let headingDelta = ((second.heading - first.heading + Math.PI) % (2 * Math.PI)) - Math.PI;
      if (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;
      return {
        x: first.x + (second.x - first.x) * ratio,
        y: first.y + (second.y - first.y) * ratio,
        heading: first.heading + headingDelta * ratio,
        timeUs,
      };
    }
  }
  return points[points.length - 1];
}

function sampleRobot(robot, timeUs) {
  let previous = null;
  for (const segment of robot.segments) {
    if (!segment.points.length) continue;
    const start = segment.points[0].timeUs;
    const end = segment.points[segment.points.length - 1].timeUs;
    const bodyRadius = Math.hypot(robot.footprintLength, robot.footprintWidth) / 2;
    if (timeUs < start) {
      if (!previous) return { ...segment.points[0], radius: bodyRadius + segment.corridorRadius };
      return { ...previous.points[previous.points.length - 1], radius: bodyRadius + previous.corridorRadius };
    }
    if (timeUs <= end) return { ...interpolatePose(segment.points, timeUs), radius: bodyRadius + segment.corridorRadius };
    previous = segment;
  }
  if (!previous) return null;
  const bodyRadius = Math.hypot(robot.footprintLength, robot.footprintWidth) / 2;
  return { ...previous.points[previous.points.length - 1], radius: bodyRadius + previous.corridorRadius };
}

function analyzeConflicts(state, periodUs = 100000) {
  const conflicts = [];
  for (let firstIndex = 0; firstIndex < state.robots.length; firstIndex += 1) {
    for (let secondIndex = firstIndex + 1; secondIndex < state.robots.length; secondIndex += 1) {
      const first = state.robots[firstIndex];
      const second = state.robots[secondIndex];
      const firstPoints = first.segments.flatMap((segment) => segment.points);
      const secondPoints = second.segments.flatMap((segment) => segment.points);
      if (!firstPoints.length || !secondPoints.length) continue;
      const start = Math.max(firstPoints[0].timeUs, secondPoints[0].timeUs);
      const end = Math.min(firstPoints[firstPoints.length - 1].timeUs, secondPoints[secondPoints.length - 1].timeUs);
      if (start > end) continue;
      let active = null;
      for (let timeUs = start; timeUs <= end + periodUs; timeUs += periodUs) {
        const sampleTime = Math.min(timeUs, end);
        const firstPose = sampleRobot(first, sampleTime);
        const secondPose = sampleRobot(second, sampleTime);
        const clearance = Math.hypot(firstPose.x - secondPose.x, firstPose.y - secondPose.y) - firstPose.radius - secondPose.radius;
        if (clearance <= 0) {
          if (!active) active = { startUs: sampleTime, endUs: sampleTime, minimumClearance: clearance, x: 0, y: 0 };
          active.endUs = sampleTime;
          if (clearance <= active.minimumClearance) {
            active.minimumClearance = clearance;
            active.x = (firstPose.x + secondPose.x) / 2;
            active.y = (firstPose.y + secondPose.y) / 2;
          }
        } else if (active) {
          conflicts.push({ firstIndex, secondIndex, ...active });
          active = null;
        }
        if (sampleTime === end) break;
      }
      if (active) conflicts.push({ firstIndex, secondIndex, ...active });
    }
  }
  return conflicts;
}

function importCanonicalDocument(document) {
  if (document.schema_version !== 1 || document.field_version !== "2018-power-up-v1") {
    throw new Error("Yalnızca schema 1 / 2018-power-up-v1 destekleniyor.");
  }
  const state = {
    planId: String(document.plan_id || ""),
    alliance: document.alliance,
    redPreview: false,
    robots: (document.robots || []).map((robot) => ({
      teamNumber: Number(robot.team_number),
      label: String(robot.robot_label || ""),
      footprintLength: Number(robot.footprint?.length_m),
      footprintWidth: Number(robot.footprint?.width_m),
      maxVelocity: Number(robot.max_velocity_mps),
      maxAcceleration: Number(robot.max_acceleration_mps2),
      initialConfidence: Number(robot.initial_confidence),
      segments: (robot.segments || []).map((segment) => ({
        id: String(segment.segment_id || ""),
        task: segment.task,
        targetId: String(segment.target_id || ""),
        startSeconds: Number(segment.earliest_start_us) / 1e6,
        durationSeconds: Number(segment.expected_duration_us) / 1e6,
        corridorRadius: Number(segment.corridor_radius_m),
        points: (segment.path || []).map((point) => ({
          timeUs: Number(point.time_us),
          x: Number(point.x),
          y: Number(point.y),
          heading: Number(point.heading),
        })),
      })),
    })),
  };
  const errors = validateState(state);
  if (errors.length) throw new Error(errors.join(" "));
  return state;
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { canonicalStringify, canonicalPayload, importCanonicalDocument, validateState, analyzeConflicts };
}

if (typeof document !== "undefined") {
  const byId = (id) => document.getElementById(id);
  const canvas = byId("field-canvas");
  const context = canvas.getContext("2d");
  const elements = {
    planId: byId("plan-id"), alliance: byId("alliance"), redPreview: byId("red-preview"),
    robotTabs: byId("robot-tabs"), teamNumber: byId("team-number"), robotLabel: byId("robot-label"),
    footprintLength: byId("footprint-length"), footprintWidth: byId("footprint-width"),
    maxVelocity: byId("max-velocity"), maxAcceleration: byId("max-acceleration"),
    initialConfidence: byId("initial-confidence"), confidenceOutput: byId("confidence-output"),
    segmentSelect: byId("segment-select"), task: byId("task"), targetId: byId("target-id"),
    startTime: byId("start-time"), duration: byId("duration"), corridorRadius: byId("corridor-radius"),
    diagnostics: byId("diagnostics"), conflicts: byId("conflicts"), contentHash: byId("content-hash"),
    validityDot: byId("validity-dot"), saveState: byId("save-state"), toast: byId("toast"),
  };
  let state = initialState();
  let activeRobot = 0;
  let activeSegment = 0;
  let selectedPoint = null;
  let dragging = false;
  let undoStack = [];
  let analysis = { errors: [], conflicts: [] };
  let fieldRect = { x: 0, y: 0, width: 1, height: 1, scale: 1 };
  let hashGeneration = 0;
  let toastTimer = null;

  function currentRobot() { return state.robots[activeRobot]; }
  function currentSegment() { return currentRobot().segments[activeSegment]; }
  function deepCopy(value) { return JSON.parse(JSON.stringify(value)); }
  function numeric(element) { return Number(element.value); }

  function snapshot() {
    undoStack.push({ state: deepCopy(state), activeRobot, activeSegment });
    if (undoStack.length > 50) undoStack.shift();
  }

  function undo() {
    const previous = undoStack.pop();
    if (!previous) return showToast("Geri alınacak işlem yok.");
    state = previous.state;
    activeRobot = Math.min(previous.activeRobot, state.robots.length - 1);
    activeSegment = Math.min(previous.activeSegment, currentRobot().segments.length - 1);
    selectedPoint = null;
    renderControls();
    updateAll();
  }

  function showToast(message, error = false) {
    clearTimeout(toastTimer);
    elements.toast.textContent = message;
    elements.toast.className = `toast show${error ? " error" : ""}`;
    toastTimer = setTimeout(() => { elements.toast.className = "toast"; }, 2600);
  }

  function renderControls() {
    elements.planId.value = state.planId;
    elements.alliance.value = state.alliance;
    elements.redPreview.checked = state.redPreview;
    elements.robotTabs.replaceChildren(...state.robots.map((robot, index) => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = `tab${index === activeRobot ? " active" : ""}`;
      button.role = "tab";
      button.textContent = `#${robot.teamNumber} ${robot.label}`;
      button.onclick = () => {
        activeRobot = index; activeSegment = 0; selectedPoint = null;
        renderControls(); updateAll();
      };
      return button;
    }));
    const robot = currentRobot();
    elements.teamNumber.value = robot.teamNumber;
    elements.robotLabel.value = robot.label;
    elements.footprintLength.value = robot.footprintLength;
    elements.footprintWidth.value = robot.footprintWidth;
    elements.maxVelocity.value = robot.maxVelocity;
    elements.maxAcceleration.value = robot.maxAcceleration;
    elements.initialConfidence.value = robot.initialConfidence;
    elements.confidenceOutput.textContent = `${Math.round(robot.initialConfidence * 100)}%`;
    elements.segmentSelect.replaceChildren(...robot.segments.map((segment, index) => {
      const option = document.createElement("option");
      option.value = String(index);
      option.textContent = `${index + 1}. ${segment.task} · ${segment.id}`;
      return option;
    }));
    elements.segmentSelect.value = String(activeSegment);
    const segment = currentSegment();
    elements.task.value = segment.task;
    elements.targetId.value = segment.targetId;
    elements.startTime.value = segment.startSeconds;
    elements.duration.value = segment.durationSeconds;
    elements.corridorRadius.value = segment.corridorRadius;
    byId("add-robot").disabled = state.robots.length >= 2;
    byId("delete-segment").disabled = robot.segments.length <= 1;
  }

  function retimeSegment(segment) {
    if (!segment.points.length) return;
    const startUs = Math.round(segment.startSeconds * 1e6);
    const durationUs = Math.round(segment.durationSeconds * 1e6);
    segment.points.forEach((point, index) => {
      const ratio = segment.points.length === 1 ? 0 : index / (segment.points.length - 1);
      point.timeUs = startUs + Math.round(durationUs * ratio);
    });
  }

  function updateHeadings(segment) {
    segment.points.forEach((point, index) => {
      const neighbor = segment.points[index + 1] || segment.points[index - 1];
      if (neighbor) point.heading = round(Math.atan2(neighbor.y - point.y, neighbor.x - point.x));
    });
  }

  function updateAll() {
    analysis.errors = validateState(state);
    analysis.conflicts = analyzeConflicts(state);
    renderCanvas();
    renderAnalysis();
    scheduleHash();
    elements.saveState.textContent = "Değişti";
  }

  function renderAnalysis() {
    const pointCount = state.robots.reduce((sum, robot) => sum + robot.segments.reduce((inner, segment) => inner + segment.points.length, 0), 0);
    byId("metric-robots").textContent = state.robots.length;
    byId("metric-points").textContent = pointCount;
    byId("metric-conflicts").textContent = analysis.conflicts.length;
    const messages = analysis.errors.length ? analysis.errors : ["Şema, alan sınırı ve hareket limitleri geçerli."];
    elements.diagnostics.replaceChildren(...messages.slice(0, 8).map((message) => {
      const item = document.createElement("li");
      item.className = analysis.errors.length ? "" : "ok";
      item.textContent = message;
      return item;
    }));
    elements.validityDot.className = `health-dot ${analysis.errors.length ? "invalid" : "valid"}`;
    if (!analysis.conflicts.length) {
      elements.conflicts.innerHTML = '<p class="empty-result">Zamansal koridor çakışması yok.</p>';
    } else {
      elements.conflicts.replaceChildren(...analysis.conflicts.map((conflict) => {
        const card = document.createElement("div");
        card.className = "conflict-card";
        const first = state.robots[conflict.firstIndex];
        const second = state.robots[conflict.secondIndex];
        card.innerHTML = `<strong>#${first.teamNumber} ↔ #${second.teamNumber}</strong>` +
          `${(conflict.startUs / 1e6).toFixed(1)}–${(conflict.endUs / 1e6).toFixed(1)} s · ` +
          `${Math.abs(conflict.minimumClearance).toFixed(2)} m örtüşme`;
        return card;
      }));
    }
    const latestSeconds = Math.max(0, ...state.robots.flatMap((robot) => robot.segments.map((segment) => segment.startSeconds + segment.durationSeconds)));
    byId("timeline-progress").style.width = `${Math.min(100, latestSeconds / 15 * 100)}%`;
    byId("canvas-empty").className = `canvas-empty${pointCount ? " hidden" : ""}`;
  }

  function scheduleHash() {
    const generation = ++hashGeneration;
    if (analysis.errors.length) {
      elements.contentHash.textContent = "Plan geçerli olduğunda hesaplanır.";
      return;
    }
    elements.contentHash.textContent = "hesaplanıyor…";
    const payload = canonicalPayload(state);
    sha256Hex(canonicalStringify(payload)).then((hash) => {
      if (generation === hashGeneration) {
        elements.contentHash.textContent = hash;
        elements.saveState.textContent = "Hash hazır";
      }
    }).catch((error) => {
      elements.contentHash.textContent = error.message;
    });
  }

  function resizeCanvas() {
    const bounds = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(bounds.width * ratio));
    canvas.height = Math.max(1, Math.round(bounds.height * ratio));
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    const margin = 34;
    const usableWidth = Math.max(1, bounds.width - margin * 2);
    const usableHeight = Math.max(1, bounds.height - margin * 2);
    const scale = Math.min(usableWidth / FIELD_LENGTH_M, usableHeight / FIELD_WIDTH_M);
    fieldRect = {
      x: (bounds.width - FIELD_LENGTH_M * scale) / 2,
      y: (bounds.height - FIELD_WIDTH_M * scale) / 2,
      width: FIELD_LENGTH_M * scale,
      height: FIELD_WIDTH_M * scale,
      scale,
    };
    renderCanvas();
  }

  function previewPose(point) {
    if (!state.redPreview) return point;
    return { ...point, x: FIELD_LENGTH_M - point.x, y: FIELD_WIDTH_M - point.y, heading: point.heading + Math.PI };
  }

  function fieldToCanvas(point) {
    const display = previewPose(point);
    return {
      x: fieldRect.x + display.x * fieldRect.scale,
      y: fieldRect.y + (FIELD_WIDTH_M - display.y) * fieldRect.scale,
    };
  }

  function canvasToField(x, y) {
    let fieldX = (x - fieldRect.x) / fieldRect.scale;
    let fieldY = FIELD_WIDTH_M - (y - fieldRect.y) / fieldRect.scale;
    if (state.redPreview) {
      fieldX = FIELD_LENGTH_M - fieldX;
      fieldY = FIELD_WIDTH_M - fieldY;
    }
    return {
      x: round(Math.min(FIELD_LENGTH_M, Math.max(0, fieldX)), 3),
      y: round(Math.min(FIELD_WIDTH_M, Math.max(0, fieldY)), 3),
    };
  }

  function drawField() {
    const { x, y, width, height, scale } = fieldRect;
    context.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
    context.fillStyle = "#0a1620";
    context.fillRect(x, y, width, height);
    const endWidth = 1.15 * scale;
    const blue = context.createLinearGradient(x, 0, x + endWidth, 0);
    blue.addColorStop(0, "rgba(30,98,225,.42)"); blue.addColorStop(1, "rgba(30,98,225,0)");
    context.fillStyle = blue; context.fillRect(x, y, endWidth, height);
    const red = context.createLinearGradient(x + width, 0, x + width - endWidth, 0);
    red.addColorStop(0, "rgba(238,49,76,.40)"); red.addColorStop(1, "rgba(238,49,76,0)");
    context.fillStyle = red; context.fillRect(x + width - endWidth, y, endWidth, height);
    context.strokeStyle = "rgba(148,190,210,.12)";
    context.lineWidth = 1;
    for (let meter = 1; meter < FIELD_LENGTH_M; meter += 1) {
      const px = x + meter * scale; context.beginPath(); context.moveTo(px, y); context.lineTo(px, y + height); context.stroke();
    }
    for (let meter = 1; meter < FIELD_WIDTH_M; meter += 1) {
      const py = y + meter * scale; context.beginPath(); context.moveTo(x, py); context.lineTo(x + width, py); context.stroke();
    }
    context.setLineDash([6, 6]); context.strokeStyle = "rgba(255,255,255,.38)";
    [3.05, FIELD_LENGTH_M - 3.05].forEach((meter) => {
      const px = x + meter * scale; context.beginPath(); context.moveTo(px, y); context.lineTo(px, y + height); context.stroke();
    });
    context.setLineDash([]);
    drawStructure(FIELD_LENGTH_M / 2 - 1.3, 1.7, 2.6, 4.83, "rgba(240,190,76,.16)");
    drawStructure(4.25, 2.47, 0.65, 3.29, "rgba(45,212,255,.13)");
    drawStructure(FIELD_LENGTH_M - 4.9, 2.47, 0.65, 3.29, "rgba(255,82,104,.13)");
    context.strokeStyle = "rgba(182,215,230,.48)"; context.lineWidth = 2; context.strokeRect(x, y, width, height);
    context.fillStyle = "rgba(220,235,245,.55)"; context.font = "10px ui-monospace, monospace";
    context.fillText(state.redPreview ? "RED PREVIEW · BLUE ORIGIN CANONICAL" : "BLUE ORIGIN · CANONICAL", x + 8, y + 15);
  }

  function drawStructure(fieldX, fieldY, fieldWidth, fieldHeight, color) {
    const topLeft = fieldToCanvas({ x: fieldX, y: fieldY + fieldHeight });
    context.fillStyle = color;
    context.fillRect(topLeft.x, topLeft.y, fieldWidth * fieldRect.scale, fieldHeight * fieldRect.scale);
    context.strokeStyle = "rgba(255,210,100,.34)"; context.lineWidth = 1;
    context.strokeRect(topLeft.x, topLeft.y, fieldWidth * fieldRect.scale, fieldHeight * fieldRect.scale);
  }

  function renderCanvas() {
    drawField();
    state.robots.forEach((robot, robotIndex) => robot.segments.forEach((segment, segmentIndex) => {
      if (!segment.points.length) return;
      const color = ROBOT_COLORS[robotIndex];
      const points = segment.points.map(fieldToCanvas);
      context.lineCap = "round"; context.lineJoin = "round";
      context.beginPath(); context.moveTo(points[0].x, points[0].y);
      points.slice(1).forEach((point) => context.lineTo(point.x, point.y));
      context.strokeStyle = `${color}26`;
      context.lineWidth = Math.max(3, 2 * (Math.hypot(robot.footprintLength, robot.footprintWidth) / 2 + segment.corridorRadius) * fieldRect.scale);
      context.stroke();
      context.beginPath(); context.moveTo(points[0].x, points[0].y);
      points.slice(1).forEach((point) => context.lineTo(point.x, point.y));
      context.strokeStyle = color; context.lineWidth = robotIndex === activeRobot && segmentIndex === activeSegment ? 4 : 2; context.stroke();
      points.forEach((point, pointIndex) => {
        const selected = selectedPoint && selectedPoint.robotIndex === robotIndex && selectedPoint.segmentIndex === segmentIndex && selectedPoint.pointIndex === pointIndex;
        context.beginPath(); context.arc(point.x, point.y, selected ? 8 : 5, 0, Math.PI * 2);
        context.fillStyle = selected ? "#ffffff" : color; context.fill();
        context.strokeStyle = "#07111c"; context.lineWidth = 2; context.stroke();
        if (robotIndex === activeRobot && segmentIndex === activeSegment) {
          context.fillStyle = "rgba(240,247,255,.78)"; context.font = "10px ui-monospace, monospace";
          context.fillText(`${(segment.points[pointIndex].timeUs / 1e6).toFixed(1)}s`, point.x + 8, point.y - 8);
        }
      });
    }));
    analysis.conflicts.forEach((conflict) => {
      const point = fieldToCanvas(conflict);
      const pulse = 10 + (Date.now() % 1200) / 120;
      context.beginPath(); context.arc(point.x, point.y, pulse, 0, Math.PI * 2);
      context.fillStyle = "rgba(255,82,104,.28)"; context.fill();
      context.strokeStyle = "#ff5268"; context.lineWidth = 2; context.stroke();
    });
  }

  function pointerCoordinates(event) {
    const bounds = canvas.getBoundingClientRect();
    return { x: event.clientX - bounds.left, y: event.clientY - bounds.top };
  }

  function hitPoint(x, y) {
    const segment = currentSegment();
    let best = null;
    segment.points.forEach((point, pointIndex) => {
      const screen = fieldToCanvas(point);
      const distance = Math.hypot(x - screen.x, y - screen.y);
      if (distance <= 12 && (!best || distance < best.distance)) best = { pointIndex, distance };
    });
    return best;
  }

  function addPoint(fieldPoint) {
    snapshot();
    const segment = currentSegment();
    const startUs = Math.round(segment.startSeconds * 1e6);
    segment.points.push({ timeUs: startUs, x: fieldPoint.x, y: fieldPoint.y, heading: 0 });
    if (segment.points.length > 1) {
      const previous = segment.points[segment.points.length - 2];
      const travel = Math.hypot(fieldPoint.x - previous.x, fieldPoint.y - previous.y) / Math.max(currentRobot().maxVelocity * 0.7, 0.1);
      const desiredEnd = (previous.timeUs - startUs) / 1e6 + Math.max(0.25, travel);
      segment.durationSeconds = round(Math.max(segment.durationSeconds, desiredEnd), 2);
    }
    retimeSegment(segment);
    updateHeadings(segment);
    selectedPoint = { robotIndex: activeRobot, segmentIndex: activeSegment, pointIndex: segment.points.length - 1 };
    renderControls(); updateAll();
  }

  function deleteSelectedPoint() {
    if (!selectedPoint || selectedPoint.robotIndex !== activeRobot || selectedPoint.segmentIndex !== activeSegment) return showToast("Önce bir yol noktası seç.");
    snapshot();
    currentSegment().points.splice(selectedPoint.pointIndex, 1);
    retimeSegment(currentSegment()); updateHeadings(currentSegment());
    selectedPoint = null; updateAll();
  }

  canvas.addEventListener("pointerdown", (event) => {
    const position = pointerCoordinates(event);
    if (position.x < fieldRect.x || position.x > fieldRect.x + fieldRect.width || position.y < fieldRect.y || position.y > fieldRect.y + fieldRect.height) return;
    const hit = hitPoint(position.x, position.y);
    if (hit) {
      snapshot();
      selectedPoint = { robotIndex: activeRobot, segmentIndex: activeSegment, pointIndex: hit.pointIndex };
      dragging = true; canvas.classList.add("dragging"); canvas.setPointerCapture(event.pointerId); renderCanvas();
    } else {
      addPoint(canvasToField(position.x, position.y));
    }
    canvas.focus();
  });
  canvas.addEventListener("pointermove", (event) => {
    const position = pointerCoordinates(event);
    const field = canvasToField(position.x, position.y);
    byId("cursor-position").textContent = `x ${field.x.toFixed(2)} · y ${field.y.toFixed(2)} m`;
    if (!dragging || !selectedPoint) return;
    const point = currentSegment().points[selectedPoint.pointIndex];
    point.x = field.x; point.y = field.y; updateHeadings(currentSegment()); updateAll();
  });
  const stopDragging = () => { dragging = false; canvas.classList.remove("dragging"); };
  canvas.addEventListener("pointerup", stopDragging);
  canvas.addEventListener("pointercancel", stopDragging);

  function bindInput(element, apply, render = false) {
    element.addEventListener("input", () => {
      apply();
      if (render) renderControls();
      updateAll();
    });
  }
  bindInput(elements.planId, () => { state.planId = elements.planId.value; });
  bindInput(elements.alliance, () => { state.alliance = elements.alliance.value; });
  bindInput(elements.redPreview, () => { state.redPreview = elements.redPreview.checked; });
  bindInput(elements.teamNumber, () => { currentRobot().teamNumber = Math.trunc(numeric(elements.teamNumber)); }, true);
  bindInput(elements.robotLabel, () => { currentRobot().label = elements.robotLabel.value; }, true);
  bindInput(elements.footprintLength, () => { currentRobot().footprintLength = numeric(elements.footprintLength); });
  bindInput(elements.footprintWidth, () => { currentRobot().footprintWidth = numeric(elements.footprintWidth); });
  bindInput(elements.maxVelocity, () => { currentRobot().maxVelocity = numeric(elements.maxVelocity); });
  bindInput(elements.maxAcceleration, () => { currentRobot().maxAcceleration = numeric(elements.maxAcceleration); });
  bindInput(elements.initialConfidence, () => {
    currentRobot().initialConfidence = numeric(elements.initialConfidence);
    elements.confidenceOutput.textContent = `${Math.round(currentRobot().initialConfidence * 100)}%`;
  });
  bindInput(elements.task, () => { currentSegment().task = elements.task.value; }, true);
  bindInput(elements.targetId, () => { currentSegment().targetId = elements.targetId.value; });
  bindInput(elements.startTime, () => { currentSegment().startSeconds = numeric(elements.startTime); retimeSegment(currentSegment()); });
  bindInput(elements.duration, () => { currentSegment().durationSeconds = numeric(elements.duration); retimeSegment(currentSegment()); });
  bindInput(elements.corridorRadius, () => { currentSegment().corridorRadius = numeric(elements.corridorRadius); });
  elements.segmentSelect.addEventListener("change", () => { activeSegment = Number(elements.segmentSelect.value); selectedPoint = null; renderControls(); updateAll(); });

  byId("add-robot").onclick = () => {
    if (state.robots.length >= 2) return;
    snapshot(); state.robots.push(makeRobot(state.robots.length)); activeRobot = state.robots.length - 1; activeSegment = 0; renderControls(); updateAll();
  };
  byId("add-segment").onclick = () => {
    snapshot();
    const robot = currentRobot();
    const previous = robot.segments[robot.segments.length - 1];
    robot.segments.push(makeSegment(robot.segments.length, previous.startSeconds + previous.durationSeconds));
    activeSegment = robot.segments.length - 1; selectedPoint = null; renderControls(); updateAll();
  };
  byId("delete-segment").onclick = () => {
    if (currentRobot().segments.length <= 1) return showToast("Robotun en az bir segmenti kalmalı.");
    snapshot(); currentRobot().segments.splice(activeSegment, 1); activeSegment = Math.max(0, activeSegment - 1); selectedPoint = null; renderControls(); updateAll();
  };
  byId("delete-point").onclick = deleteSelectedPoint;
  byId("undo").onclick = undo;
  byId("clear-segment").onclick = () => {
    if (!currentSegment().points.length) return;
    snapshot(); currentSegment().points = []; selectedPoint = null; updateAll();
  };

  document.addEventListener("keydown", (event) => {
    const editing = ["INPUT", "SELECT", "TEXTAREA"].includes(document.activeElement?.tagName);
    if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "z") { event.preventDefault(); undo(); return; }
    if (editing) return;
    if (event.key === "Delete" || event.key === "Backspace") { event.preventDefault(); deleteSelectedPoint(); return; }
    if (event.key === "Escape") { selectedPoint = null; renderCanvas(); return; }
    const offsets = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, 1], ArrowDown: [0, -1] };
    if (selectedPoint && offsets[event.key]) {
      event.preventDefault(); snapshot();
      const step = event.shiftKey ? 0.25 : 0.05;
      const point = currentSegment().points[selectedPoint.pointIndex];
      point.x = round(Math.min(FIELD_LENGTH_M, Math.max(0, point.x + offsets[event.key][0] * step)), 3);
      point.y = round(Math.min(FIELD_WIDTH_M, Math.max(0, point.y + offsets[event.key][1] * step)), 3);
      updateHeadings(currentSegment()); updateAll();
    }
  });

  function parseCsv(text) {
    const lines = text.replace(/^\uFEFF/, "").trim().split(/\r?\n/);
    const expected = "time,x,y,heading,vx,vy,omega,event";
    if (lines.shift()?.trim() !== expected) throw new Error(`CSV başlığı tam olarak ${expected} olmalı.`);
    const rows = lines.map((line, index) => {
      const columns = line.split(",");
      if (columns.length !== 8) throw new Error(`CSV ${index + 2}. satırda 8 sütun içermeli.`);
      const numbers = columns.slice(0, 7).map(Number);
      if (!numbers.every(Number.isFinite)) throw new Error(`CSV ${index + 2}. satırda geçersiz sayı var.`);
      return { time: numbers[0], x: numbers[1], y: numbers[2], heading: numbers[3], vx: numbers[4], vy: numbers[5], event: columns[7].trim() };
    });
    if (rows.length < 2) throw new Error("CSV en az iki örnek içermeli.");
    rows.slice(1).forEach((row, index) => {
      const previous = rows[index];
      const dt = row.time - previous.time;
      if (dt <= 0) throw new Error("CSV zamanları kesin artmalı.");
      const vx = (row.x - previous.x) / dt;
      const vy = (row.y - previous.y) / dt;
      if (Math.hypot(vx - row.vx, vy - row.vy) > 0.25) throw new Error("CSV konum ve hızları uyuşmuyor.");
    });
    const starts = rows.map((row, index) => row.event ? index : -1).filter((index) => index >= 0);
    if (!starts.length || starts[0] !== 0) starts.unshift(0);
    return starts.map((start, segmentIndex) => {
      const end = segmentIndex + 1 < starts.length ? starts[segmentIndex + 1] : rows.length - 1;
      if (end <= start) throw new Error("Her CSV görev segmentinde en az iki örnek olmalı.");
      const [taskText, targetText] = (rows[start].event || "CROSS_LINE:auto-line").split(":");
      const task = taskText.toUpperCase();
      if (!VALID_TASKS.has(task)) throw new Error(`Desteklenmeyen CSV görevi: ${task}`);
      return {
        id: `csv-${task.toLowerCase().replaceAll("_", "-")}-${segmentIndex + 1}`,
        task,
        targetId: targetText || "auto-line",
        startSeconds: rows[start].time,
        durationSeconds: rows[end].time - rows[start].time,
        corridorRadius: 0.2,
        points: rows.slice(start, end + 1).map((row) => ({ timeUs: Math.round(row.time * 1e6), x: row.x, y: row.y, heading: round(row.heading * Math.PI / 180) })),
      };
    });
  }

  byId("file-input").addEventListener("change", async (event) => {
    const file = event.target.files[0];
    if (!file) return;
    try {
      const text = await file.text();
      snapshot();
      if (file.name.toLowerCase().endsWith(".csv")) {
        currentRobot().segments = parseCsv(text); activeSegment = 0;
      } else {
        const documentValue = JSON.parse(text);
        const imported = importCanonicalDocument(documentValue);
        if (documentValue.content_sha256) {
          const hash = await sha256Hex(canonicalStringify(canonicalPayload(imported)));
          if (hash !== documentValue.content_sha256) throw new Error("JSON içerik hash'i doğrulanamadı.");
        }
        state = imported; activeRobot = 0; activeSegment = 0;
      }
      selectedPoint = null; renderControls(); updateAll(); showToast(`${file.name} içe aktarıldı.`);
    } catch (error) {
      undoStack.pop(); showToast(error.message, true);
    } finally {
      event.target.value = "";
    }
  });

  byId("validate-button").onclick = () => {
    updateAll();
    showToast(analysis.errors.length ? `${analysis.errors.length} doğrulama sorunu var.` : "Plan geçerli ve dışa aktarmaya hazır.", Boolean(analysis.errors.length));
  };

  function download(name, blob) {
    const link = document.createElement("a");
    link.href = URL.createObjectURL(blob); link.download = name; link.click();
    setTimeout(() => URL.revokeObjectURL(link.href), 1000);
  }

  byId("export-json").onclick = async () => {
    updateAll();
    if (analysis.errors.length) return showToast("JSON dışa aktarmadan önce tanıları düzelt.", true);
    const payload = canonicalPayload(state);
    const hash = await sha256Hex(canonicalStringify(payload));
    const documentValue = { ...payload, content_sha256: hash };
    download(`${state.planId || "alliance-plan"}.json`, new Blob([`${JSON.stringify(documentValue, null, 2)}\n`], { type: "application/json" }));
    showToast("Kanonik plan ve SHA-256 indirildi.");
  };
  byId("export-png").onclick = () => {
    canvas.toBlob((blob) => {
      if (blob) { download(`${state.planId || "alliance-plan"}-preview.png`, blob); showToast("Sunum görseli indirildi."); }
    }, "image/png");
  };

  new ResizeObserver(resizeCanvas).observe(canvas.parentElement);
  renderControls(); resizeCanvas(); updateAll();
}
