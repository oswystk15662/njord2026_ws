# GNSS Map Telemetry / BMS Monitor

Foxglove custom Map panel with a fixed top-right legend for latitude, longitude,
base_link heading, and speed over ground. It consumes
`/sensor/vehicle_gnss/fix/raw`, `/gui/ground_speed_mps`, `/tf`, and `/tf_static`.
The panel renders OpenStreetMap tiles with its bundled Leaflet 1.9.4 runtime.
After centering on the first valid fix, the map remains under manual pan and zoom
control while the vessel arrow moves and rotates.

The `battery-status` panel is a responsive BMS monitor. It scales the complete
panel to remain visible when its Foxglove panel is resized. It displays remaining
charge from `/gui/battery_percent` and four cell voltages from
`/bms/cell_voltages` (`std_msgs/msg/Float32MultiArray`). Each cell has a gauge and
numeric voltage using a 3.7 V to less-than-4.2 V display range.
It also renders a vertical thermometer from 10 to 60 degrees Celsius using
`/bms/tempereture_c` (`std_msgs/msg/Float32`). The thermometer rises through a
blue-to-red color scale and includes a numeric temperature.

The `battery-status-table` panel subscribes to the same topics and presents the
remaining charge, four cell voltages, and temperature as a large-text two-column
table without gauges.

Install [gnss-map-telemetry-0.2.11.foxe](gnss-map-telemetry-0.2.11.foxe) by dragging it
into Foxglove, then add the `battery-status` panel.

The vessel arrow points along the `base_link` +X axis. Its orientation is resolved
from the `map -> base_link` TF chain, assuming the `map` frame is ENU (+X east,
+Y north). If the chain is unavailable, the panel shows a position dot and `HDG --`.

Leaflet is distributed under the BSD 2-Clause License. Its license text is included
at `dist/vendor/LEAFLET-LICENSE.txt`.

Rebuild the self-contained extension and `.foxe` archive without Node.js or npm:

```bash
./scripts/package.sh
```
