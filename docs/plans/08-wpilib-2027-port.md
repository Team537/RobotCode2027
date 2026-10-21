# Plan 8 — Port to WPILib 2027 / SystemCore

**Goal:** arrive at kickoff (January 2027) with this project already building and simulating on the WPILib 2027 beta, so week one of build season is spent on the game, not on a package rename.

**Why this is unavoidable:** 2027 is a hard control-system break. The roboRIO is not legal for competition; every team gets a SystemCore in the kit of parts. WPILib 2027 requires Java 25, renames `edu.wpi.first` → `org.wpilib`, adds Commands v3 (v2 stays), removes `robotInit()`, replaces `SmartDashboard`/`Shuffleboard`/`SendableChooser` with a Telemetry/Tunables API and `Selectable`, renames `ChassisSpeeds` → `ChassisVelocities`, and moves the field origin to field center. YAGSL/YAMS already target `2027_alpha7`; PathPlannerLib has a `2027.0.0-alpha`; 3847 has already run a SystemCore robot at an event.

**Effort:** ~1 week in December, plus a second pass at kickoff when the final release lands.

**Prerequisites:** all other plans at least started — the point is to port *the code you'll keep*. Plan 5's CI makes the port safe to attempt on a branch.

**Caveats:** official 2027 beta dates and CTRE/REV library status for SystemCore were not confirmed from primary sources when this was written (Oct 2026). Check the WPILib blog and Chief Delphi before scheduling.

---

## Step 0 — Decisions and prep (before December)

1. **Keep the 2026 branch deployable.** Create `2026-roborio` branch from `main` before porting; the practice chassis keeps running on it. Port on `main` (or a `wpilib-2027` branch merged when stable).
2. **Minimize 2026-only code now.** Everywhere a plan says "keep this behind one small class" (auto chooser, alliance flip, dashboard calls), do that — those are the port's touch points.
3. **Prefer command factories and Triggers** over subclassed `Command`s in new code. Commands v3 keeps the factory style; subclasses port less cleanly.
4. **Watch the vendors.** Each vendordep (Phoenix 6, REVLib, PhotonLib, PathPlanner, YAGSL/YAMS, Studica, ThriftyLib) needs a 2027 build. Make a table in `docs/2027-port.md` and fill it in as each ships. YAGSL/YAMS and PathPlanner already have alphas; Phoenix 6 and REVLib for SystemCore are the ones to watch.

## Step 1 — Toolchain (1 day, when the beta is available)

1. Install the WPILib 2027 beta (VS Code + JDK 25 + tools) alongside 2026.
2. Set `.wpilib/wpilib_preferences.json` `projectYear` to `2027` and let the "Import project" wizard regenerate `build.gradle` (GradleRIO 2027, `sourceCompatibility = 25`).
3. Update every vendordep to its 2027 build. Any that lack one: stub the dependency and note it in `docs/2027-port.md`.
4. `./gradlew build` → expect hundreds of import errors. That's Step 2.

## Step 2 — Mechanical changes (1–2 days)

| Change | What to do |
|---|---|
| `edu.wpi.first.*` → `org.wpilib.*` | Project-wide find/replace on imports; WPILib may ship a migration script — check the changelog first |
| `robotInit()` removed | Already fine — `Robot.java` uses a constructor |
| `ChassisSpeeds` → `ChassisVelocities` | Find/replace, including field names (check the new names in the changelog) |
| `SmartDashboard.putData("Auto Chooser", …)` + `SendableChooser` | Replace with the Telemetry API and `Selectable` in the one method that builds the chooser (`RobotContainer.java:46-47`) |
| Elastic layout | Keys published through the Telemetry API may change names; re-save the layout |
| Epilogue | Expect it to become the Telemetry API; follow the migration note in the changelog |
| `DriverStation`/`Timer`/`HAL` in tests (Plan 7) | Package rename only |
| Units (`MetersPerSecond`, `Seconds`) | Package rename only |

Run `./gradlew spotlessApply` after the rename to fix import order.

## Step 3 — Semantic changes (2–3 days)

### Field origin moves to field center
This is the one that breaks geometry silently.
- Every `FieldConstants` pose, every `AprilTagFieldLayout` usage, every PathPlanner path and `settings.json`, and the alliance-flip helper assume the blue-corner origin today.
- Expect PathPlanner and PhotonLib 2027 to handle their own files; your job is `FieldConstants` and `AllianceFlip`. With a centered origin, flipping becomes a rotation by 180° about the origin (check the WPILib `FlippingUtil`/equivalent for 2027 rather than hand-rolling it).
- Redraw or re-import PathPlanner paths once PathPlanner 2027 ships its migration.

### Commands v3 (optional)
v2 still works in 2027, so don't rewrite everything. Try v3 on one new command (`DriveToPose` is a good candidate) to learn it; migrate more only if it's clearly better.

### CAN ports
2027 introduces a `CANPort` enum; CANivore is allowed as an extra bus through 2027 only, and there is an 18-motor cap. YAGSL's JSON `canbus` fields may change — follow YAGSL's schema-migration page.

### OpMode framework
New in 2027 alongside `TimedRobot`. Stay on `TimedRobot` for the port; evaluate OpModes later.

## Step 4 — Verify (1 day)

1. `./gradlew build` green locally and in CI (update the CI container tag to the 2027 image).
2. All Plan 7 tests pass.
3. Simulate: drive the robot, run `DriveToPose`, run the test-square auto, run the practice mechanism. AdvantageScope 2027 field overlay should show the robot at the right place with the new origin.
4. If a SystemCore is available: deploy, confirm CAN devices enumerate, drive on blocks.
5. Tag the commit `2027-beta-port`.

## Step 5 — Kickoff pass (January)

When the 2027 release lands: bump vendordeps to release builds, re-run Step 4, then start the game-specific work in Plans 4 and 6.

---

## Done when

- [ ] `2026-roborio` branch exists and still deploys to the practice chassis.
- [ ] `main` builds on WPILib 2027 beta with Java 25; CI green.
- [ ] Vendordep status table in `docs/2027-port.md` is complete.
- [ ] Sim runs the drivetrain, `DriveToPose`, an auto and the practice mechanism with the centered field origin.
- [ ] Dashboard rebuilt on the Telemetry API; `SendableChooser` gone.

## Risks

- A vendor (most likely a motor-controller library) ships late. Mitigation: port everything else; keep the hardware-touching classes isolated so they're the only thing waiting.
- Field-origin bugs are silent — an auto that drives to the wrong corner. Test every `FieldConstants` pose in sim against the 2027 field image before trusting it.
- Beta API churn. WPILib's lead said 2027 changes "are not going to change significantly" from the alphas, but expect small renames between beta and release; keep the port on its own branch until it's stable.
