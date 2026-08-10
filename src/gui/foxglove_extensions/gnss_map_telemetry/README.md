# GNSS Map Telemetry

Foxglove custom Map panel with a fixed top-right legend for latitude, longitude,
base_link heading, and speed over ground. It consumes the consolidated
`/foxglove_log` telemetry published by `foxglove_logger`.
The panel renders OpenStreetMap tiles with its bundled Leaflet 1.9.4 runtime.
After centering on the first valid fix, the map remains under manual pan and zoom
control while the vessel arrow moves and rotates.

Install [gnss-map-telemetry-0.2.3.foxe](gnss-map-telemetry-0.2.3.foxe) by dragging it
into Foxglove, then import `foxglove_setting.json`.

The vessel arrow uses the `HDG` value in `/foxglove_log`.

Leaflet is distributed under the BSD 2-Clause License. Its license text is included
at `dist/vendor/LEAFLET-LICENSE.txt`.

Rebuild the self-contained extension and `.foxe` archive without Node.js or npm:

```bash
./scripts/package.sh
```
