const FIX_TOPIC = "/sensor/vehicle_gnss/fix/raw";
const SPEED_TOPIC = "/gui/ground_speed_mps";
const BATTERY_PERCENT_TOPIC = "/gui/battery_percent";
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
.vessel-icon{height:34px;width:34px;filter:drop-shadow(0 1px 2px rgba(0,0,0,.8))}.vessel-arrow{height:34px;width:34px;transform-origin:17px 17px}.vessel-arrow path{fill:#00cceb;stroke:#063946;stroke-width:1.5;stroke-linejoin:round}
.vessel-dot{display:none;position:absolute;left:10px;top:10px;width:14px;height:14px;border:3px solid #063946;border-radius:50%;background:#00cceb;box-sizing:border-box}.vessel-icon.no-heading .vessel-arrow{display:none}.vessel-icon.no-heading .vessel-dot{display:block}
`;

function format(value, digits, suffix) {
  return typeof value === "number" && Number.isFinite(value) ? `${value.toFixed(digits)}${suffix}` : "--";
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
  return L.divIcon({
    className: "",
    iconSize: [34, 34],
    iconAnchor: [17, 17],
    html: '<div class="vessel-icon no-heading"><svg class="vessel-arrow" viewBox="0 0 34 34" aria-label="Catamaran vessel heading"><path d="M5 30 L8 7 L12 2 L15 30 Z"/><path d="M19 30 L22 2 L26 7 L29 30 Z"/><path d="M9 15 H25 V20 H9 Z"/><path d="M13 15 L17 9 L21 15 Z"/></svg><div class="vessel-dot"></div></div>',
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

function activate(extensionContext) {
  extensionContext.registerPanel({name: "gnss-map-telemetry", initPanel: initGnssMapTelemetry});
}

module.exports = {
  activate,
  multiplyQuaternions,
  normalizeQuaternion,
  quaternionToBearingDegrees,
  resolveOrientation,
};
