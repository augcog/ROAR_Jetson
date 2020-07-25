# ROAR: Perception Module

## Usage:
```
import perception.gpd as gpd

gpd_agent = gpd.gpd_sensor(calibration_depth)

# Running code...
	orig_d_frame, d_frame, dets = gpd_agent.output_gpd(d_frame)
# Running code...
```

### Params:
- t=0.089 : threshold for sensing ground, higher t => less sensitive
- del_ang=0.02 : threshold for angle changes, higher del_ang => less sensitive
- fit_type='exp' : choose 'exp' or 'lsq'

### Methods:
- output_gpd: d_frame -> orig_d_frame, filtered_d_frame, detection_bool_matrix

## TODO:
1. Give users ability to access height and roll angle
2. De-roll incoming frames instead of adding roll to model
