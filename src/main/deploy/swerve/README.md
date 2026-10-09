# Swerve config

YAGSL builds the drivetrain from the JSON files in `base/`. They were generated at
[config.yagsl.com](https://config.yagsl.com) for our REV MAXSwerve modules.

| Setting | Value |
|---|---|
| Gyro | Pigeon 2, CAN ID 42 |
| Drive motors | Kraken X60 (front), NEO on Spark MAX (back) |
| Steer motors | NEO 550 on Spark MAX |
| Absolute encoders | REV Through Bore, attached to the steer Spark MAX |
| CAN IDs (drive / steer) | FL 2/3, FR 4/5, BL 6/7, BR 8/9 |
| Gearing | 4.71:1 drive, 46.42:1 steer, 3 in wheels |
| Module positions | 10 in from center on each axis |

Still to do on the real robot:

- Tune the PID gains in `modules/pidfproperties.json`. The steer P of 0.01 is likely far too
  low.
- Verify the encoder offsets and inversions using the tuning guide on
  [config.yagsl.com](https://config.yagsl.com).

If you change the drivetrain config, also update `../pathplanner/settings.json` (gearing, wheel
size, module positions, drive motor), since PathPlanner uses it for autos. Robot mass, MOI and
bumper size there are still placeholders.
