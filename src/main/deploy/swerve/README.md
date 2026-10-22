# Swerve config

YAGSL builds the drivetrain from the JSON files in `base/`. They were generated at
[config.yagsl.com](https://config.yagsl.com) for our REV MAXSwerve modules.

| Setting | Value |
|---|---|
| Gyro | Pigeon 2, CAN ID 42 |
| Drive motors | Kraken X60 (TalonFX) on all four modules |
| Steer motors | NEO 550 on Spark MAX |
| Absolute encoders | REV Through Bore, attached to the steer Spark MAX |
| CAN IDs (drive / steer) | FL 2/3, FR 4/5, BL 6/7, BR 8/9 |
| Gearing | 4.71:1 drive, 46.42:1 steer, 3 in wheels |
| Current limits (stator) | 40 A drive, 20 A steer |
| Module positions | 8.25 in from center on each axis (20 in frame, wheels 1.75 in in from the edge per REV drawing REV-21-3005) |

Keep the steer limit at 20 A. A stalled NEO 550 lasts minutes at 20 A but only seconds at 40 A
or more. The drive limit can go up to 60 A once brownouts have been checked.

Speed limits are in `Constants.DriveConstants`: 5.08 m/s module free speed, 4.3 m/s chassis
(replace with the measured top speed).

Still to do on the real robot:

- Tune the PID gains in `modules/pidfproperties.json`. The steer P of 0.01 is likely far too
  low.
- Verify the encoder offsets and inversions using the tuning guide on
  [config.yagsl.com](https://config.yagsl.com).

If you change the drivetrain config, also update `../pathplanner/settings.json` (gearing, wheel
size, module positions, drive motor, current limit, max speed), since PathPlanner uses it for
autos. Mass, MOI and bumper size there are estimates until measured; see
`docs/drivetrain-measurements.md`.
