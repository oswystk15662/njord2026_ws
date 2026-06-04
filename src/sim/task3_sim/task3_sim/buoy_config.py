import math

# Task 3.1 Buoy Configuration
# Red buoys: (2.0, 4.0), (7.0, 4.0), (12.0, 4.0), (12.0, 10.0)
# Green buoys: (2.0, -4.0), (7.0, -4.0), (12.0, -4.0), (14.0, 10.0)
BUOY_DEFINITIONS_3_1 = [
    {'name': 'red_buoy_1', 'center': [2.0, 4.0], 'phase_offset': 0.0},
    {'name': 'red_buoy_2', 'center': [7.0, 4.0], 'phase_offset': math.pi / 3.0},
    {'name': 'red_buoy_3', 'center': [12.0, 4.0], 'phase_offset': 2.0 * math.pi / 3.0},
    {'name': 'red_buoy_gate', 'center': [12.0, 10.0], 'phase_offset': math.pi},
    {'name': 'green_buoy_1', 'center': [2.0, -4.0], 'phase_offset': math.pi / 2.0},
    {'name': 'green_buoy_2', 'center': [7.0, -4.0], 'phase_offset': 5.0 * math.pi / 6.0},
    {'name': 'green_buoy_3', 'center': [12.0, -4.0], 'phase_offset': 7.0 * math.pi / 6.0},
    {'name': 'green_buoy_gate', 'center': [14.0, 10.0], 'phase_offset': 3.0 * math.pi / 2.0},
]

# Task 3.2 Buoy Configuration (Point-Symmetric to 3.1)
# Red: (-2.0, -4.0), (-7.0, -4.0), (-12.0, -4.0), (-12.0, -10.0)
# Green: (-2.0, 4.0), (-7.0, 4.0), (-12.0, 4.0), (-14.0, -10.0)
BUOY_DEFINITIONS_3_2 = [
    {'name': 'red_buoy_1', 'center': [-2.0, -4.0], 'phase_offset': 0.0},
    {'name': 'red_buoy_2', 'center': [-7.0, -4.0], 'phase_offset': math.pi / 3.0},
    {'name': 'red_buoy_3', 'center': [-12.0, -4.0], 'phase_offset': 2.0 * math.pi / 3.0},
    {'name': 'red_buoy_gate', 'center': [-12.0, -10.0], 'phase_offset': math.pi},
    {'name': 'green_buoy_1', 'center': [-2.0, 4.0], 'phase_offset': math.pi / 2.0},
    {'name': 'green_buoy_2', 'center': [-7.0, 4.0], 'phase_offset': 5.0 * math.pi / 6.0},
    {'name': 'green_buoy_3', 'center': [-12.0, 4.0], 'phase_offset': 7.0 * math.pi / 6.0},
    {'name': 'green_buoy_gate', 'center': [-14.0, -10.0], 'phase_offset': 3.0 * math.pi / 2.0},
]
