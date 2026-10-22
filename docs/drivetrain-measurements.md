# Drivetrain measurements

Measured values for the drivetrain, so future students know where the config numbers came from.
Update this file whenever you re-measure.

## Robot

| Item | Value | Source |
|---|---|---|
| Mass (bumpers + battery) | ~60 lb (27.2 kg) | Estimate; weigh on a scale |
| Frame (outside) | 20 in × 20 in | Measured |
| Module centers | 16.5 in apart (±8.25 in) | Frame − 2 × 1.75 in MAXSwerve inset (REV-21-3005 drawing) |
| Bumper-to-bumper | 26.5 in (0.673 m) | Estimate; bumpers not built yet. Update PathPlanner robotWidth/Length when measured |
| MOI | 2.05 kg·m² | Estimate: mass × (L² + W²) / 12 |
| Drive motors | Kraken X60 ×4 | Confirmed |
| Steer motors | NEO 550 ×4 | Confirmed |
| Drive pinion | 14T (4.71:1) | Confirmed |
| Gyro | Pigeon 2, CAN ID 42, roboRIO CAN bus | Confirmed |
| Encoder offsets | REV defaults (-90 / 0 / 180 / 90) | Not yet zeroed with REV alignment tool |

## Tuning

| Item | Value | Date |
|---|---|---|
| Steer P / D | 0.01 / 0 (untuned) | |
| Drive P / D | 0.9 / 0.3 (untuned) | |

## Carpet tests

| Test | Result | Target | Date |
|---|---|---|---|
| Encoder sanity (push 1 m) | | All modules count up, pose x ≈ 1.0 m | |
| Wheel radius (10 spins) | | Within 1–3% of nominal | |
| 10 m straight line | | < 10 cm, < 2° drift | |
| Max speed | | Plateau velocity → `kMaxChassisSpeedMps` | |
| 2 m square | | < 15 cm final error | |
