"use strict";

const FIX_TOPIC = "/sensor/vehicle_gnss/fix/raw";
const SPEED_TOPIC = "/gui/ground_speed_mps";

function format(value, digits, suffix) {
  return typeof value === "number" ? `${value.toFixed(digits)}${suffix}` : "--";
}

function buildMapUrl(latitude, longitude) {
  if (typeof latitude !== "number" || typeof longitude !== "number") {
    return "https://www.openstreetmap.org/export/embed.html?layer=mapnik";
  }
  const marker = `${latitude.toFixed(6)}%2C${longitude.toFixed(6)}`;
  return `https://www.openstreetmap.org/export/embed.html?layer=mapnik&marker=${marker}#map=16/${latitude.toFixed(6)}/${longitude.toFixed(6)}`;
}

function initGnssMapTelemetry(context) {
  const root = context.panelElement;
  root.style.cssText = "height:100%;position:relative;overflow:hidden;background:#15202b";

  const map = document.createElement("iframe");
  map.title = "GNSS map";
  map.src = buildMapUrl();
  map.style.cssText = "border:0;height:100%;width:100%;";
  root.appendChild(map);

  const legend = document.createElement("div");
  legend.style.cssText = [
    "position:absolute", "right:12px", "top:12px", "z-index:10", "min-width:225px",
    "padding:10px 12px", "border:1px solid #526375", "border-radius:6px",
    "background:rgba(12,18,28,.92)", "color:#f4f7fb", "font:14px/1.5 system-ui,sans-serif",
  ].join(";");
  root.appendChild(legend);

  const state = {};
  let lastMapPosition = "";
  const renderLegend = () => {
    legend.innerHTML = [
      '<div style="color:#a9c7e8;font-size:12px;font-weight:700;letter-spacing:.06em">VESSEL TELEMETRY</div>',
      `<div>LAT&nbsp;&nbsp; ${format(state.latitude, 6, "°")}</div>`,
      `<div>LON&nbsp; ${format(state.longitude, 6, "°")}</div>`,
      '<div style="border-top:1px solid #526375;margin-top:5px;padding-top:5px">',
      `SOG&nbsp;&nbsp; ${format(state.speedMps, 2, " m/s")}</div>`,
    ].join("");
  };

  context.subscribe([{ topic: FIX_TOPIC }, { topic: SPEED_TOPIC }]);
  context.watch("currentFrame");
  context.onRender = (renderState, done) => {
    for (const event of renderState.currentFrame || []) {
      const message = event.message || {};
      if (event.topic === FIX_TOPIC) {
        if (typeof message.latitude === "number") state.latitude = message.latitude;
        if (typeof message.longitude === "number") state.longitude = message.longitude;
      } else if (event.topic === SPEED_TOPIC && typeof message.data === "number") {
        state.speedMps = message.data;
      }
    }
    renderLegend();
    const currentMapPosition = `${state.latitude},${state.longitude}`;
    if (currentMapPosition !== lastMapPosition && typeof state.latitude === "number" && typeof state.longitude === "number") {
      map.src = buildMapUrl(state.latitude, state.longitude);
      lastMapPosition = currentMapPosition;
    }
    done();
  };
  renderLegend();
  return () => root.replaceChildren();
}

function activate(extensionContext) {
  extensionContext.registerPanel({ name: "gnss-map-telemetry", initPanel: initGnssMapTelemetry });
}

module.exports = { activate };
