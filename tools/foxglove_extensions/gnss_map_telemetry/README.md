# GNSS Map Telemetry

Foxglove custom Map panel with a fixed top-right legend for latitude, longitude,
and speed over ground. It consumes `/sensor/vehicle_gnss/fix/raw` and
`/gui/ground_speed_mps`. The panel uses OpenStreetMap's embeddable map, and can
be panned and zoomed directly in the map area.

Install [gnss-map-telemetry-0.1.1.foxe](gnss-map-telemetry-0.1.1.foxe) by dragging it
into Foxglove, then import `foxglove_setting.json`.
