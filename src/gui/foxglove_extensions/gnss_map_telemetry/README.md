# GNSS Map Telemetry

Foxglove custom Map panel with a fixed top-right legend for latitude, longitude,
base_link heading, and speed over ground. It consumes
`/sensor/vehicle_gnss/fix/raw`, `/gui/ground_speed_mps`, `/tf`, and `/tf_static`.
The panel renders OpenStreetMap tiles with its bundled Leaflet 1.9.4 runtime.
After centering on the first valid fix, the map remains under manual pan and zoom
control while the vessel arrow moves and rotates.  It also consumes the
transient-local `/ground_waypoint_markers` `visualization_msgs/msg/MarkerArray`
from the Ground PC's local `waypoint_publisher` configuration and overlays each
configured WP.  This marker list is generated from the installed waypoint YAML;
it does not depend on waypoint-marker traffic from the vessel.

Install [gnss-map-telemetry-0.2.3.foxe](gnss-map-telemetry-0.2.3.foxe) by dragging it
into Foxglove, then import `foxglove_setting.json`.

The cyan pentagon's tip points along the `base_link` +X axis. Its orientation is
resolved on the Ground PC from the `map -> base_link` TF chain, assuming the `map`
frame is ENU (+X east, +Y north). If the chain is unavailable, the panel shows a
position dot and `HDG --`.

Leaflet is distributed under the BSD 2-Clause License. Its license text is included
at `dist/vendor/LEAFLET-LICENSE.txt`.

Rebuild the self-contained extension and `.foxe` archive without Node.js or npm:

```bash
./scripts/package.sh
```
