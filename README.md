# Extended Kalman Filter

In this project, an Extended Kalman Filter (EKF) is implemented to estimate the real-time position of a moving object with noisy lidar and radar measurements.

## Input data

Given this example of data:

| R    | 8.60363 | 0.0290616 | -2.99903 | 1399637 | 8.6  | 0.25    | 3.00029 | 0    |
| ---- | ------- | --------- | -------- | ------- | ---- | ------- | ------- | ---- |
| L    | 8.45    | 0.25      | 2.903    | 147701  | 0.25 | 3.00027 |         |      |

Each cell represents following variables, respectively:

| radar | meas_rho | meas_phi | meas_rho_dot | timestamp | gt_px | gt_py | gt_vx | gt_vy |
| ----- | -------- | -------- | ------------ | --------- | ----- | ----- | ----- | ----- |
| laser | meas_px  | meas_py  | timestamp    | gt_px     | gt_py | gt_vx | gt_vy |       |

## Output Format

`est_px, est_py, est_vx, est_vy, meas_px, meas_py, gt_px, gt_py, gt_vx, gt_vy`

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   - On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Results

| Input      | Ground truth | RMSE      |
| ---------- | ------------ | --------- |
| position x | 0.11         | 0.0963513 |
| position y | 0.11         | 0.0852071 |
| velocity x | 0.52         | 0.415053  |
| velocity y | 0.52         | 0.431299  |

RMSE -> residual mean square error.

The EKF obtained a great result, which is less than the ground truth.