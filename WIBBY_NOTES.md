# WIBBY swerve work-in-progress (NOT working yet)

Branch for the robot **WIBBY** (4x MAXSwerve, Kraken X60 drive, NEO 550 steer, REV Through Bore
absolute encoders, Pigeon 2 on CAN 42). Based on `feature/03-vision-localization`.

**Known issue: wheel orientation.** The steer modules oscillate / twitch back and forth when
enabled, even with no stick input. Cause not yet confirmed. Suspects: steer motor direction
reversed relative to the Through Bore encoder (positive feedback), steer kP too high, or the
target angle flipping.

## What this branch adds
- `SparkZeroOffsets`: keeps the REV Hardware Client absolute-encoder zero. YAMS writes its own
  zero offset (JSON value, 0) to the steer Spark MAXes on every configure, which wiped it.
- `ModuleZeroing`: dashboard readout `Zeroing/<module>/AdjustedRotations` (0 or 1 = straight).
- `SteerDirectionTest`: dashboard button that checks steer motor vs encoder direction.
- `SwerveModuleTuner` / `SwerveAutoTuner`: live PID tuning and an unverified auto-tune (never run
  on the robot).
- All `absoluteEncoderOffset` in module JSONs are 0 (zero comes from the REV encoder).
- Steer kP 1.5 (was 0.01), drive kP 0.1 / kD 0.03. All untuned.

## Deploy
`./gradlew --stop` then `./gradlew deploy` (roboRIO 10.5.37.2).
