# ARCHIVE: YAGSL 2026.3.12 / old Spark MAX firmware snapshot

**Not the main line. Kept only as a fallback.**

## What this is
A snapshot of the swerve drivetrain ported back to last year's stack so it ran against
Spark MAX firmware 25.0.4:

- YAGSL 2026.3.12 (+ ReduxLib 2026.1.1) instead of YAGSL 2026.10.03 + YAMS
- `deploy/swerve/base/*` rewritten in the old YAGSL JSON schema
- `SwerveDriveSubsystem` / `RobotContainer` use the `swervelib` API
- `build.gradle` excludes `.claude-flow`, `.swarm` and `*.db` from the deploy

## Why it exists
The robot showed "No Robot Code" with `Invalid parameter id` from the steer Spark MAXes.
It was first blamed on 25.x firmware rejecting the feedforward params YAGSL 2026.10.03
sends. It later looked like a hardware problem, so this branch is probably unneeded.

## Status
- Builds and starts in sim. Never verified on the robot.
- Module offsets, inversions and drive PID are untested on hardware.
- Old schema has no s/v/a feedforward fields (the YAMS versions are in git history on
  `feature/02-driver-controls`).

## To go back to the main line
Check out `feature/02-driver-controls`. Do not merge this branch.
