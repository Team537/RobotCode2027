# NEW BASE CHASSIS

Branch for the **new base chassis**. Still swerve, but it does **not** follow the WIBBY setup.

Started from `feature/03-vision-localization` (clean, before any WIBBY-specific swerve work).
The WIBBY hardware work lives separately on `robot/wibby-swerve-wip`.

## Still to configure for this chassis
- Module CAN IDs, motor types and module positions (`src/main/deploy/swerve/base/modules/*.json`)
- Gyro type and CAN ID (`src/main/deploy/swerve/base/swervedrive.json`)
- Gear ratios, wheel diameter, current limits (`physicalproperties.json`)
- PID/feedforward gains (`pidfproperties.json`), absolute-encoder type and zeroing
- Max speeds in `Constants.DriveConstants`
- Vision camera transforms (`subsystems/vision`)

Values currently in the module JSONs are the WIBBY (MAXSwerve, Kraken X60, NEO 550) values and
are placeholders until this chassis is measured.
