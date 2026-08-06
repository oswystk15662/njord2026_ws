const FIX_TOPIC = "/sensor/vehicle_gnss/fix/raw";
const SPEED_TOPIC = "/gui/ground_speed_mps";
const BATTERY_PERCENT_TOPIC = "/gui/battery_percent";
const CELL_VOLTAGES_TOPIC = "/bms/cell_voltages";
const BMS_TEMPERATURE_TOPIC = "/bms/tempereture_c";
const TF_TOPIC = "/tf";
const TF_STATIC_TOPIC = "/tf_static";
const WORLD_FRAME = "map";
const VESSEL_FRAME = "base_link";

const PANEL_CSS = `
.gnss-map-root{height:100%;position:relative;overflow:hidden;background:#15202b}
.gnss-map-canvas{height:100%;width:100%;font-family:system-ui,sans-serif}
.leaflet-pane,.leaflet-tile,.leaflet-marker-icon,.leaflet-marker-shadow,.leaflet-tile-container,
.leaflet-pane>svg,.leaflet-pane>canvas,.leaflet-zoom-box,.leaflet-image-layer,.leaflet-layer{position:absolute;left:0;top:0}
.leaflet-container{overflow:hidden;background:#d9d9d9;outline:0;-webkit-tap-highlight-color:transparent}
.leaflet-tile,.leaflet-marker-icon,.leaflet-marker-shadow{user-select:none;-webkit-user-drag:none}
.leaflet-container .leaflet-tile-pane img,.leaflet-container .leaflet-tile{max-width:none!important;max-height:none!important;width:auto;padding:0}
.leaflet-container.leaflet-touch-zoom{touch-action:pan-x pan-y}
.leaflet-container.leaflet-touch-drag{touch-action:none;touch-action:pinch-zoom}
.leaflet-container.leaflet-touch-drag.leaflet-touch-zoom{touch-action:none}
.leaflet-tile{filter:inherit;visibility:hidden}.leaflet-tile-loaded{visibility:inherit}
.leaflet-pane{z-index:400}.leaflet-tile-pane{z-index:200}.leaflet-overlay-pane{z-index:400}
.leaflet-shadow-pane{z-index:500}.leaflet-marker-pane{z-index:600}.leaflet-tooltip-pane{z-index:650}.leaflet-popup-pane{z-index:700}
.leaflet-map-pane canvas{z-index:100}.leaflet-map-pane svg{z-index:200}
.leaflet-control{position:relative;z-index:800;pointer-events:auto}.leaflet-top,.leaflet-bottom{position:absolute;z-index:1000;pointer-events:none}
.leaflet-top{top:0}.leaflet-right{right:0}.leaflet-bottom{bottom:0}.leaflet-left{left:0}
.leaflet-control{float:left;clear:both}.leaflet-right .leaflet-control{float:right}.leaflet-top .leaflet-control{margin-top:10px}
.leaflet-bottom .leaflet-control{margin-bottom:10px}.leaflet-left .leaflet-control{margin-left:10px}.leaflet-right .leaflet-control{margin-right:10px}
.leaflet-zoom-animated{transform-origin:0 0}.leaflet-zoom-anim .leaflet-zoom-animated{transition:transform .25s cubic-bezier(0,0,.25,1)}
.leaflet-zoom-anim .leaflet-tile,.leaflet-pan-anim .leaflet-tile{transition:none}.leaflet-zoom-anim .leaflet-zoom-hide{visibility:hidden}
.leaflet-grab{cursor:grab}.leaflet-dragging .leaflet-grab{cursor:grabbing}
.leaflet-marker-icon,.leaflet-marker-shadow,.leaflet-image-layer,.leaflet-tile-container{pointer-events:none}
.leaflet-container{font:12px/1.5 "Helvetica Neue",Arial,Helvetica,sans-serif}.leaflet-container a{color:#0078a8}
.leaflet-bar{box-shadow:0 1px 5px rgba(0,0,0,.65);border-radius:4px}.leaflet-bar a{background:#fff;border-bottom:1px solid #ccc;width:30px;height:30px;line-height:30px;display:block;text-align:center;text-decoration:none;color:#111;font:bold 20px/30px monospace}
.leaflet-bar a:hover{background:#f4f4f4}.leaflet-bar a:first-child{border-radius:4px 4px 0 0}.leaflet-bar a:last-child{border-radius:0 0 4px 4px;border-bottom:0}
.leaflet-container .leaflet-control-attribution{background:rgba(255,255,255,.85);margin:0;padding:0 5px;color:#333;line-height:1.4}.leaflet-control-attribution a{text-decoration:none}
.gnss-telemetry{position:absolute;right:12px;top:12px;z-index:1100;min-width:225px;padding:10px 12px;border:1px solid #526375;border-radius:6px;background:rgba(12,18,28,.92);color:#f4f7fb;font:14px/1.5 system-ui,sans-serif;pointer-events:none}
.gnss-telemetry-title{color:#a9c7e8;font-size:12px;font-weight:700;letter-spacing:.06em}.gnss-telemetry-separator{border-top:1px solid #526375;margin-top:5px;padding-top:5px}
.gnss-map-error{display:none;position:absolute;left:50%;bottom:34px;z-index:1100;transform:translateX(-50%);padding:7px 10px;border-radius:4px;background:rgba(137,28,28,.92);color:#fff;font:13px system-ui,sans-serif;pointer-events:none}.gnss-map-error.visible{display:block}
	.vessel-icon{height:40px;width:40px;filter:drop-shadow(0 1px 2px rgba(0,0,0,.8))}.vessel-arrow{height:40px;width:40px;transform-origin:20px 20px}.vessel-body{fill:#00cceb;stroke:#063946;stroke-width:1.8;stroke-linejoin:round;stroke-linecap:round;fill-rule:evenodd}
	.vessel-dot{display:none;position:absolute;left:13px;top:13px;width:14px;height:14px;border:3px solid #063946;border-radius:50%;background:#00cceb;box-sizing:border-box}.vessel-icon.no-heading .vessel-arrow{display:none}.vessel-icon.no-heading .vessel-dot{display:block}
	`;

const BATTERY_PANEL_CSS = `
.battery-root{height:100%;min-height:1px;position:relative;background:#111827;color:#f9fafb;font-family:system-ui,sans-serif;overflow:hidden}
.battery-stage{position:absolute;left:50%;top:50%;width:520px;height:500px;margin-left:-260px;margin-top:-250px;transform-origin:center center}
.battery-card{width:520px;height:500px;box-sizing:border-box;padding:7px 9px;display:flex;flex-direction:column;align-items:center;gap:6px;border:2px solid #475569;border-radius:10px;background:#162031;box-shadow:0 12px 28px rgba(0,0,0,.34)}
.battery-section-title{font-size:18px;font-weight:900;letter-spacing:.09em;color:#dbe7f5}
.battery-shell{position:relative;align-self:flex-start;width:calc(100% - 17px);height:76px;margin-right:17px;border:5px solid #e5edf5;border-radius:8px;background:#1f2937;box-sizing:border-box;box-shadow:0 8px 20px rgba(0,0,0,.35)}
.battery-shell::after{content:"";position:absolute;right:-17px;top:24px;width:12px;height:26px;border-radius:0 5px 5px 0;background:#e5edf5}
.battery-fill{position:absolute;left:6px;top:6px;bottom:6px;width:0%;border-radius:4px;background:#22c55e;transition:width .2s ease,background-color .2s ease}
.battery-percent{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:44px;font-weight:900;color:#ffffff;text-shadow:0 2px 4px rgba(0,0,0,.75)}
.monitor-body{width:100%;display:grid;grid-template-columns:minmax(0,1fr) 110px;gap:22px;flex:1;min-height:0}
.cell-section,.temperature-section{display:flex;flex-direction:column;align-items:center;gap:6px;min-height:0}
.cell-gauges{width:100%;display:grid;grid-template-columns:repeat(4,1fr);gap:12px;flex:1;min-height:0}
.cell-gauge{display:flex;flex-direction:column;align-items:center;gap:5px}
.cell-name{font-size:18px;font-weight:900;color:#e4edf7}
.cell-track{position:relative;width:48px;height:205px;border:3px solid #cbd5e1;border-radius:8px;background:#263244;overflow:hidden;box-sizing:border-box}
.cell-fill{position:absolute;left:4px;right:4px;bottom:4px;height:0%;border-radius:4px;background:#22c55e;transition:height .2s ease,background-color .2s ease}
.cell-value{font-size:27px;font-weight:900;font-variant-numeric:tabular-nums;color:#ffffff;text-shadow:0 1px 2px rgba(0,0,0,.7)}
.cell-unit{font-size:17px;font-weight:800;color:#c6d3e1}
.temperature-meter{position:relative;width:70px;height:255px;margin-top:1px}
.temperature-tube{position:absolute;left:21px;top:0;width:28px;height:215px;border:4px solid #dbe4ee;border-bottom:0;border-radius:16px 16px 0 0;background:#263244;box-sizing:border-box;overflow:hidden;z-index:2}
.temperature-fill-clip{position:absolute;left:4px;right:4px;bottom:0;height:0%;overflow:hidden;transition:height .25s ease}
.temperature-gradient{position:absolute;left:0;right:0;bottom:0;height:207px;background:linear-gradient(to top,#2563eb 0%,#22d3ee 30%,#facc15 62%,#ef4444 100%)}
.temperature-bulb{position:absolute;left:8px;bottom:0;width:54px;height:54px;border:4px solid #dbe4ee;border-radius:50%;box-sizing:border-box;background:#2563eb;box-shadow:0 4px 10px rgba(0,0,0,.35);transition:background-color .25s ease;z-index:3}
.temperature-bulb::after{content:"";position:absolute;left:12px;top:10px;width:8px;height:8px;border-radius:50%;background:rgba(255,255,255,.42)}
.temperature-value{font-size:29px;font-weight:900;font-variant-numeric:tabular-nums;color:#ffffff;text-shadow:0 1px 2px rgba(0,0,0,.7)}
.temperature-unit{font-size:18px;font-weight:800;color:#c6d3e1}
`;

const BATTERY_TABLE_PANEL_CSS = `
.battery-table-root{height:100%;min-height:1px;position:relative;overflow:hidden;background:#162031;color:#f8fafc;font-family:system-ui,sans-serif;--table-font-size:20px;--table-value-size:24px;--table-unit-size:15px}
.battery-table-stage{position:absolute;inset:0}
.battery-table-card{width:100%;height:100%;box-sizing:border-box;padding:2px 8px;border:2px solid #475569;border-radius:6px;background:#162031}
.battery-table{width:100%;height:100%;border-collapse:collapse;table-layout:fixed;font-variant-numeric:tabular-nums}
.battery-table td{border-bottom:2px solid #3d4b60;font-size:var(--table-font-size);font-weight:800;white-space:nowrap}
.battery-table td:first-child{width:55%}
.battery-table tr:last-child td{border-bottom:0}
.battery-table td:last-child{text-align:right;font-size:var(--table-value-size);font-weight:900;color:#ffffff;text-shadow:0 1px 2px rgba(0,0,0,.7)}
.battery-table-unit{font-size:var(--table-unit-size);font-weight:800;color:#c6d3e1}
`;

const CELL_MIN_VOLTAGE = 3.7;
const CELL_MAX_VOLTAGE = 4.2;
const TEMPERATURE_MIN_C = 10;
const TEMPERATURE_MAX_C = 60;

function format(value, digits, suffix) {
  return typeof value === "number" && Number.isFinite(value) ? `${value.toFixed(digits)}${suffix}` : "--";
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function batteryColor(percent) {
  if (!Number.isFinite(percent)) return "#64748b";
  if (percent <= 20) return "#ef4444";
  if (percent <= 50) return "#eab308";
  return "#22c55e";
}

function cellFillPercent(voltage) {
  if (!Number.isFinite(voltage)) return 0;
  return clamp((voltage - CELL_MIN_VOLTAGE) / (CELL_MAX_VOLTAGE - CELL_MIN_VOLTAGE) * 100, 0, 100);
}

function cellColor(voltage) {
  if (!Number.isFinite(voltage)) return "#64748b";
  if (voltage < CELL_MIN_VOLTAGE || voltage >= CELL_MAX_VOLTAGE) return "#ef4444";
  if (voltage < 3.8) return "#eab308";
  return "#22c55e";
}

function readCellVoltages(data) {
  if (data == undefined || typeof data.length !== "number") return undefined;
  const values = Array.from(data).slice(0, 4).map(Number);
  return values.length === 4 ? values : undefined;
}

function temperatureFillPercent(temperature) {
  if (!Number.isFinite(temperature)) return 0;
  return clamp(
    (temperature - TEMPERATURE_MIN_C) / (TEMPERATURE_MAX_C - TEMPERATURE_MIN_C) * 100,
    0,
    100,
  );
}

function temperatureColor(temperature) {
  if (!Number.isFinite(temperature)) return "#64748b";
  const percent = temperatureFillPercent(temperature);
  if (percent < 30) return "#2563eb";
  if (percent < 60) return "#22d3ee";
  if (percent < 82) return "#facc15";
  return "#ef4444";
}

function normalizeFrame(frame) {
  return typeof frame === "string" ? frame.replace(/^\/+/, "") : "";
}

function normalizeQuaternion(rotation) {
  if (!rotation) return undefined;
  const values = [rotation.x, rotation.y, rotation.z, rotation.w];
  if (!values.every((value) => typeof value === "number" && Number.isFinite(value))) return undefined;
  const magnitude = Math.hypot(...values);
  if (magnitude < 1e-12) return undefined;
  return {x: rotation.x / magnitude, y: rotation.y / magnitude, z: rotation.z / magnitude, w: rotation.w / magnitude};
}

function multiplyQuaternions(a, b) {
  return {
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}

function resolveOrientation(transforms, worldFrame, vesselFrame) {
  const rotations = [];
  const visited = new Set();
  let current = normalizeFrame(vesselFrame);
  const world = normalizeFrame(worldFrame);
  while (current !== world) {
    if (!current || visited.has(current)) return undefined;
    visited.add(current);
    const transform = transforms.get(current);
    if (!transform) return undefined;
    rotations.push(transform.rotation);
    current = transform.parent;
  }
  let result = {x: 0, y: 0, z: 0, w: 1};
  for (let index = rotations.length - 1; index >= 0; index -= 1) {
    result = multiplyQuaternions(result, rotations[index]);
  }
  return normalizeQuaternion(result);
}

function quaternionToBearingDegrees(rotation) {
  if (!rotation) return undefined;
  // map is ENU. Rotate base_link +X into map, then convert east/north to a compass bearing.
  const east = 1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z);
  const north = 2 * (rotation.x * rotation.y + rotation.w * rotation.z);
  if (!Number.isFinite(east) || !Number.isFinite(north) || Math.hypot(east, north) < 1e-9) return undefined;
  return (Math.atan2(east, north) * 180 / Math.PI + 360) % 360;
}

function vesselIcon() {
  const points = [
    [11, 3], [7, 11], [7, 30], [11, 37], [15, 30], [15, 22],
    [25, 22], [25, 30], [29, 37], [33, 30], [33, 11], [29, 3],
    [25, 11], [25, 18], [15, 18], [15, 11],
  ];
  const bodyPath = `${points.map(([x, y], index) => `${index === 0 ? "M" : "L"}${x} ${y}`).join(" ")} Z`;
  return L.divIcon({
    className: "",
    iconSize: [40, 40],
    iconAnchor: [20, 20],
    html: `<div class="vessel-icon no-heading"><svg class="vessel-arrow" viewBox="0 0 40 40" aria-label="Catamaran vessel heading"><path class="vessel-body" d="${bodyPath}"/></svg><div class="vessel-dot"></div></div>`,
  });
}

function initGnssMapTelemetry(context) {
  const root = context.panelElement;
  root.replaceChildren();
  root.className = "gnss-map-root";

  const style = document.createElement("style");
  style.textContent = PANEL_CSS;
  root.appendChild(style);

  const mapElement = document.createElement("div");
  mapElement.className = "gnss-map-canvas";
  root.appendChild(mapElement);

  const legend = document.createElement("div");
  legend.className = "gnss-telemetry";
  root.appendChild(legend);

  const mapError = document.createElement("div");
  mapError.className = "gnss-map-error";
  mapError.textContent = "OpenStreetMapタイルを読み込めません";
  root.appendChild(mapError);

  const map = L.map(mapElement, {zoomControl: true, attributionControl: true}).setView([20, 0], 2);
  const tileLayer = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
  }).addTo(map);
  tileLayer.on("tileerror", () => mapError.classList.add("visible"));
  tileLayer.on("load", () => mapError.classList.remove("visible"));

  const state = {};
  const transforms = new Map();
  let marker;
  let centered = false;

  function updateTransforms(message) {
    for (const stamped of message?.transforms || []) {
      const parent = normalizeFrame(stamped.header?.frame_id);
      const child = normalizeFrame(stamped.child_frame_id);
      const rotation = normalizeQuaternion(stamped.transform?.rotation);
      if (parent && child && rotation) transforms.set(child, {parent, rotation});
    }
  }

  function updateMarker() {
    if (!Number.isFinite(state.latitude) || !Number.isFinite(state.longitude)) return;
    const position = [state.latitude, state.longitude];
    if (!marker) marker = L.marker(position, {icon: vesselIcon(), interactive: false}).addTo(map);
    else marker.setLatLng(position);
    if (!centered) {
      map.setView(position, 16, {animate: false});
      centered = true;
    }

    const orientation = resolveOrientation(transforms, WORLD_FRAME, VESSEL_FRAME);
    state.headingDegrees = quaternionToBearingDegrees(orientation);
    const element = marker.getElement()?.querySelector(".vessel-icon");
    const arrow = element?.querySelector(".vessel-arrow");
    if (element && arrow && Number.isFinite(state.headingDegrees)) {
      element.classList.remove("no-heading");
      arrow.style.transform = `rotate(${state.headingDegrees}deg)`;
    } else if (element) {
      element.classList.add("no-heading");
    }
  }

  function renderLegend() {
    legend.innerHTML = [
      '<div class="gnss-telemetry-title">VESSEL TELEMETRY</div>',
      `<div>LAT&nbsp;&nbsp; ${format(state.latitude, 6, "°")}</div>`,
      `<div>LON&nbsp; ${format(state.longitude, 6, "°")}</div>`,
      `<div>HDG&nbsp; ${format(state.headingDegrees, 1, "°")}</div>`,
      '<div class="gnss-telemetry-separator">',
      `SOG&nbsp;&nbsp; ${format(state.speedMps, 2, " m/s")}</div>`,
      `BAT&nbsp;&nbsp; ${format(state.batteryPercent, 0, " %")}</div>`,
    ].join("");
  }

  context.subscribe([
    {topic: FIX_TOPIC}, {topic: SPEED_TOPIC}, {topic: BATTERY_PERCENT_TOPIC},
    {topic: TF_TOPIC}, {topic: TF_STATIC_TOPIC},
  ]);
  context.watch("currentFrame");
  context.onRender = (renderState, done) => {
    try {
      for (const event of renderState.currentFrame || []) {
        const message = event.message || {};
        if (event.topic === FIX_TOPIC) {
          if (typeof message.latitude === "number") state.latitude = message.latitude;
          if (typeof message.longitude === "number") state.longitude = message.longitude;
        } else if (event.topic === SPEED_TOPIC && typeof message.data === "number") {
          state.speedMps = message.data;
        } else if (event.topic === BATTERY_PERCENT_TOPIC && typeof message.data === "number") {
          state.batteryPercent = message.data;
        } else if (event.topic === TF_TOPIC || event.topic === TF_STATIC_TOPIC) {
          updateTransforms(message);
        }
      }
      updateMarker();
      renderLegend();
    } finally {
      done();
    }
  };

  const resizeObserver = new ResizeObserver(() => map.invalidateSize({animate: false}));
  resizeObserver.observe(root);
  renderLegend();

  return () => {
    resizeObserver.disconnect();
    map.remove();
    root.replaceChildren();
  };
}

function initBatteryStatus(context) {
  const root = context.panelElement;
  root.replaceChildren();
  root.className = "battery-root";

  const style = document.createElement("style");
  style.textContent = BATTERY_PANEL_CSS;
  root.appendChild(style);

  const card = document.createElement("div");
  card.className = "battery-stage";
  card.innerHTML = [
    '<div class="battery-card">',
    '<div class="battery-section-title">REMAINING CHARGE</div>',
    '<div class="battery-shell"><div class="battery-fill"></div><div class="battery-percent">--%</div></div>',
    '<div class="monitor-body">',
    '<div class="cell-section">',
    '<div class="battery-section-title">CELL VOLTAGES</div>',
    '<div class="cell-gauges">',
    ...[1, 2, 3, 4].map((cell) => [
      '<div class="cell-gauge">',
      `<div class="cell-name">CELL ${cell}</div>`,
      '<div class="cell-track"><div class="cell-fill"></div></div>',
      '<div class="cell-value">--<span class="cell-unit"> V</span></div>',
      '</div>',
    ].join("")),
    '</div>',
    '</div>',
    '<div class="temperature-section">',
    '<div class="battery-section-title">TEMPERATURE</div>',
    '<div class="temperature-meter">',
    '<div class="temperature-tube"><div class="temperature-fill-clip"><div class="temperature-gradient"></div></div></div>',
    '<div class="temperature-bulb"></div>',
    '</div>',
    '<div class="temperature-value">--<span class="temperature-unit"> °C</span></div>',
    '</div>',
    '</div>',
    '</div>',
  ].join("");
  root.appendChild(card);

  const fill = card.querySelector(".battery-fill");
  const percentText = card.querySelector(".battery-percent");
  const cellFills = [...card.querySelectorAll(".cell-fill")];
  const cellValues = [...card.querySelectorAll(".cell-value")];
  const temperatureFill = card.querySelector(".temperature-fill-clip");
  const temperatureBulb = card.querySelector(".temperature-bulb");
  const temperatureValue = card.querySelector(".temperature-value");
  let batteryPercent;
  let cellVoltages = [];
  let temperatureC;

  function renderBattery() {
    if (!Number.isFinite(batteryPercent)) {
      fill.style.width = "0%";
      fill.style.backgroundColor = batteryColor(undefined);
      percentText.textContent = "--%";
      return;
    }
    const percent = clamp(batteryPercent, 0, 100);
    fill.style.width = `${percent}%`;
    fill.style.backgroundColor = batteryColor(percent);
    percentText.textContent = `${Math.round(percent)}%`;
  }

  function renderCells() {
    for (let index = 0; index < 4; index += 1) {
      const voltage = cellVoltages[index];
      cellFills[index].style.height = `${cellFillPercent(voltage)}%`;
      cellFills[index].style.backgroundColor = cellColor(voltage);
      cellValues[index].innerHTML = Number.isFinite(voltage)
        ? `${voltage.toFixed(2)}<span class="cell-unit"> V</span>`
        : '--<span class="cell-unit"> V</span>';
    }
  }

  function renderTemperature() {
    temperatureFill.style.height = `${temperatureFillPercent(temperatureC)}%`;
    temperatureBulb.style.backgroundColor = temperatureColor(temperatureC);
    temperatureValue.innerHTML = Number.isFinite(temperatureC)
      ? `${temperatureC.toFixed(2)}<span class="temperature-unit"> °C</span>`
      : '--<span class="temperature-unit"> °C</span>';
  }

  function resizeCard() {
    const scale = Math.max(0.03, Math.min((root.clientWidth - 4) / 520, (root.clientHeight - 4) / 500));
    card.style.transform = `scale(${scale})`;
  }

  context.subscribe([
    {topic: BATTERY_PERCENT_TOPIC},
    {topic: CELL_VOLTAGES_TOPIC},
    {topic: BMS_TEMPERATURE_TOPIC},
  ]);
  context.watch("currentFrame");
  context.onRender = (renderState, done) => {
    try {
      for (const event of renderState.currentFrame || []) {
        const message = event.message || {};
        if (event.topic === BATTERY_PERCENT_TOPIC && typeof message.data === "number") {
          batteryPercent = message.data;
        } else if (event.topic === CELL_VOLTAGES_TOPIC) {
          const values = readCellVoltages(message.data);
          if (values) cellVoltages = values;
        } else if (event.topic === BMS_TEMPERATURE_TOPIC && typeof message.data === "number") {
          temperatureC = message.data;
        }
      }
      renderBattery();
      renderCells();
      renderTemperature();
    } finally {
      done();
    }
  };

  renderBattery();
  renderCells();
  renderTemperature();
  const resizeObserver = new ResizeObserver(resizeCard);
  resizeObserver.observe(root);
  resizeCard();
  return () => {
    resizeObserver.disconnect();
    root.replaceChildren();
  };
}

function initBatteryStatusTable(context) {
  const root = context.panelElement;
  root.replaceChildren();
  root.className = "battery-table-root";

  const style = document.createElement("style");
  style.textContent = BATTERY_TABLE_PANEL_CSS;
  root.appendChild(style);

  const stage = document.createElement("div");
  stage.className = "battery-table-stage";
  stage.innerHTML = [
    '<div class="battery-table-card">',
    '<table class="battery-table">',
    '<tbody>',
    '<tr><td>Remaining charge</td><td data-field="remaining">--<span class="battery-table-unit"> %</span></td></tr>',
    ...[1, 2, 3, 4].map((cell) =>
      `<tr><td>Cell ${cell}</td><td data-field="cell-${cell}">--<span class="battery-table-unit"> V</span></td></tr>`),
    '<tr><td>Temperature</td><td data-field="temperature">--<span class="battery-table-unit"> °C</span></td></tr>',
    '</tbody></table></div>',
  ].join("");
  root.appendChild(stage);

  const remainingElement = stage.querySelector('[data-field="remaining"]');
  const cellElements = [1, 2, 3, 4].map((cell) => stage.querySelector(`[data-field="cell-${cell}"]`));
  const temperatureElement = stage.querySelector('[data-field="temperature"]');
  let batteryPercent;
  let cellVoltages = [];
  let temperatureC;

  function renderTable() {
    remainingElement.innerHTML = Number.isFinite(batteryPercent)
      ? `${Math.round(clamp(batteryPercent, 0, 100))}<span class="battery-table-unit"> %</span>`
      : '--<span class="battery-table-unit"> %</span>';
    for (let index = 0; index < 4; index += 1) {
      const voltage = cellVoltages[index];
      cellElements[index].innerHTML = Number.isFinite(voltage)
        ? `${voltage.toFixed(2)}<span class="battery-table-unit"> V</span>`
        : '--<span class="battery-table-unit"> V</span>';
      cellElements[index].style.color = cellColor(voltage);
    }
    temperatureElement.innerHTML = Number.isFinite(temperatureC)
      ? `${temperatureC.toFixed(2)}<span class="battery-table-unit"> °C</span>`
      : '--<span class="battery-table-unit"> °C</span>';
    remainingElement.style.color = batteryColor(batteryPercent);
    temperatureElement.style.color = temperatureColor(temperatureC);
  }

  function resizeTable() {
    const rowHeight = Math.max(1, (root.clientHeight - 4) / 6);
    const fontSize = Math.max(9, Math.min(rowHeight * 0.52, root.clientWidth * 0.055));
    root.style.setProperty("--table-font-size", `${fontSize}px`);
    root.style.setProperty("--table-value-size", `${fontSize * 1.18}px`);
    root.style.setProperty("--table-unit-size", `${fontSize * 0.7}px`);
  }

  context.subscribe([
    {topic: BATTERY_PERCENT_TOPIC},
    {topic: CELL_VOLTAGES_TOPIC},
    {topic: BMS_TEMPERATURE_TOPIC},
  ]);
  context.watch("currentFrame");
  context.onRender = (renderState, done) => {
    try {
      for (const event of renderState.currentFrame || []) {
        const message = event.message || {};
        if (event.topic === BATTERY_PERCENT_TOPIC && typeof message.data === "number") {
          batteryPercent = message.data;
        } else if (event.topic === CELL_VOLTAGES_TOPIC) {
          const values = readCellVoltages(message.data);
          if (values) cellVoltages = values;
        } else if (event.topic === BMS_TEMPERATURE_TOPIC && typeof message.data === "number") {
          temperatureC = message.data;
        }
      }
      renderTable();
    } finally {
      done();
    }
  };

  renderTable();
  const resizeObserver = new ResizeObserver(resizeTable);
  resizeObserver.observe(root);
  resizeTable();
  return () => {
    resizeObserver.disconnect();
    root.replaceChildren();
  };
}

function activate(extensionContext) {
  extensionContext.registerPanel({name: "gnss-map-telemetry", initPanel: initGnssMapTelemetry});
  extensionContext.registerPanel({name: "battery-status", initPanel: initBatteryStatus});
  extensionContext.registerPanel({name: "battery-status-table", initPanel: initBatteryStatusTable});
}

module.exports = {
  activate,
  multiplyQuaternions,
  normalizeQuaternion,
  quaternionToBearingDegrees,
  resolveOrientation,
  cellFillPercent,
  readCellVoltages,
  temperatureFillPercent,
};
