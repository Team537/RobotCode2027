# Robot code roadmap — detailed plans

These plans turn the research in [`reports/FRC robot code roadmap 2027.md`](../../reports/FRC%20robot%20code%20roadmap%202027.md) into step-by-step work the team can pick up. Each plan has a goal, why it matters, prerequisites, numbered steps with code, a "done when" checklist, risks, and a rough effort estimate.

Everything is anchored to the code as of commit `262c4c6` (one swerve subsystem, no vision, no autos, no logging). API names were checked against the pinned libraries (YAGSL/YAMS 2026.10.03, PathPlannerLib 2026.1.2, PhotonLib 2026.3.4, Phoenix 6 26.3.0, WPILib 2026.2.1).

## The plans

| # | Plan | Phase | Effort | Depends on |
|---|------|-------|--------|------------|
| 1 | [Drivetrain configuration and validation](01-drivetrain-config.md) | Foundation | 1–2 days + carpet time | — |
| 2 | [Driver controls: input shaping, acceleration limiting, heading hold, Pigeon tilt](02-driver-controls.md) | Foundation | 2–4 days | 1 |
| 3 | [AprilTag vision localization](03-vision-localization.md) | Localization | 1–2 weeks incl. hardware | 1, 5 (logging) |
| 4 | [Drive-to-pose and first autonomous routines](04-drive-to-pose-and-autos.md) | Localization | 1–2 weeks | 1, 3 |
| 5 | [Logging, simulation habit, and CI](05-logging-sim-ci.md) | Foundation | ~1 day each | — |
| 6 | [Superstructure and mechanism skeleton](06-superstructure-skeleton.md) | Architecture | ~1 week | 5 |
| 7 | [Unit tests on robot logic](07-unit-tests.md) | Architecture | hours per test | 5, 6 |
| 8 | [Port to WPILib 2027 / SystemCore](08-wpilib-2027-port.md) | Architecture | ~1 week | all |

## Suggested order

```
Week 1–2   Plan 5 (CI + logging first: ~1 day, then every other plan benefits)
           Plan 1 (config fixes, then carpet tests)
Week 2–3   Plan 2 (controls)                       ← robot drives nicely; stop here if time is short
Week 3–5   Plan 3 (vision)  — order cameras in week 1 so they arrive in time
Week 5–7   Plan 4 (drive-to-pose, first auto)
Dec        Plan 6, Plan 7, then Plan 8 (2027 port) before kickoff
Jan 2027   Fill in the game-specific blanks each plan leaves
```

Plans 1 and 5 can run in parallel on different students. Plan 2 can start as soon as the robot drives straight after Plan 1.

## Two decisions to make early

1. **Mixed drive motors.** `deploy/swerve/base/modules/` has Kraken X60s on the front drives and NEOs on the rear. Plan 1 explains the trade-off; the lowest-risk fix is four matching motors. Decide before tuning.
2. **Camera hardware.** Plan 3 recommends PhotonVision on an Orange Pi 5 with two Arducam OV9281s (~$200 total) or a Limelight 4 (~$489). Order early; lead time is the long pole for Plan 3.

## Conventions used in the plans

- File paths are relative to the repo root.
- Code blocks are meant to compile against the pinned library versions, but treat them as a starting point, not a drop-in: read them, then type them.
- "Carpet test" means on real FRC carpet; hard floors give wrong odometry numbers.
- Numbers marked *(starting point)* come from other teams' published code and must be re-tuned on this robot.
