# Plan 7 — Unit tests on robot logic

**Goal:** JUnit tests, run by CI on every PR, for the parts of the code that are pure logic and bite hardest when wrong: state transitions, geometry (alliance flipping, nearest-pose), the acceleration limiter, and heading hold.

**Why:** 1678, 2910 and 581 run `./gradlew test` in CI. Tests don't replace sim or carpet time; they catch the "I refactored the enum and broke the auto" class of bug at zero cost. `build.gradle` already has JUnit 5 configured — only `src/test/java` is missing.

**Effort:** an hour for the harness, then roughly an hour per test class.

**Prerequisites:** Plan 5 (CI), Plan 2 and Plan 6 code to test.

---

## Step 1 — Harness (30 minutes)

Create `src/test/java/frc/robot/TestBase.java`:

```java
package frc.robot;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

/** Initializes the HAL in simulation so WPILib classes (Timer, PIDController, etc.) work in tests. */
public abstract class TestBase {
  @BeforeEach
  void setupHal() {
    assert HAL.initialize(500, 0);
  }

  @AfterEach
  void shutdownHal() {
    HAL.shutdown();
  }
}
```

Run `./gradlew test` to confirm the harness compiles. CI (Plan 5) already runs this.

## Step 2 — Tests worth writing first

### `ChassisAccelLimiterTest` (Plan 2)
- Limiter never changes magnitude by more than `accel × 0.02` per call.
- Direction of a diagonal request is preserved (ratio of vx:vy unchanged while ramping).
- Decel limit applies when slowing; accel limit when speeding up.
- `reset(current)` makes the next output start from `current`.

```java
@Test
void rampsAtMostMaxAccelPerLoop() {
  var limiter = new ChassisAccelLimiter(() -> 2.0, () -> 2.0, () -> 10.0);
  limiter.reset(new ChassisSpeeds());
  var out = limiter.calculate(new ChassisSpeeds(3.0, 4.0, 0));
  assertEquals(2.0 * 0.02, Math.hypot(out.vxMetersPerSecond, out.vyMetersPerSecond), 1e-9);
  assertEquals(3.0 / 4.0, out.vxMetersPerSecond / out.vyMetersPerSecond, 1e-9);  // direction kept
}
```

### `HeadingHoldTest` (Plan 2)
- With rotation input, output omega equals input (no hold).
- After release and the latch delay, output omega corrects toward the latched heading (use a mutable heading supplier and `SimHooks.stepTiming` from `edu.wpi.first.wpilibj.simulation` to advance the `Timer`).

### `AllianceFlipTest` (Plan 4)
- With `DriverStationSim.setAllianceStationId(...)` set to red, `AllianceFlip.apply(bluePose)` mirrors x across the field and flips rotation; blue returns the pose unchanged.
- The nearest-pose selection picks the expected pose for a few robot positions.

### `SuperstructureTest` (Plan 6)
- Requesting `PREPARE_SCORE_HIGH` sets the elevator's wanted state to `HIGH`.
- `SuperState` becomes `READY_TO_SCORE` only once the elevator reports `atGoal()`.
- Build the `Superstructure` with a fake elevator (a subclass or an interface) so no motor controller is constructed in the test.

To make this testable, give `Superstructure` a way to be constructed with a fake — simplest is an interface `ElevatorIO { void setWantedState(…); boolean atGoal(); }` that both the real subsystem and a test stub implement. Keep the real subsystem's hardware construction out of the constructor path that tests use.

### `VisionFilterTest` (Plan 3)
- Pose outside the field margin is rejected.
- Single-tag with ambiguity > 0.4 is rejected.
- Std devs scale up with distance and are `MAX_VALUE` for a single tag beyond 4 m.

Pull the `accept(...)` and `updateStdDevs(...)` logic into a static helper or package-private method so it can be called without a `PhotonCamera`.

## Step 3 — Rules for tests here

- Test *logic*, not hardware. If a test needs a `TalonFX` or `SparkMax`, restructure the code instead.
- One assertion per behavior; a test name reads as a sentence (`rejectsPoseOutsideField`).
- Tests must pass in CI's container, which has no robot — anything that touches a CAN device will fail there.
- Don't test WPILib itself (e.g. that `PIDController` computes P × error).

---

## Done when

- [ ] `src/test/java/frc/robot/TestBase.java` exists and `./gradlew test` passes.
- [ ] Tests for the limiter, heading hold, alliance flip, superstructure transitions, and vision filters.
- [ ] CI runs them on every PR and a failing test blocks merge.

## Risks

- HAL initialization in tests sometimes conflicts when tests run in parallel. Keep `maxParallelForks = 1` (the Gradle default) for the `test` task.
- Simulated timing: `Timer` in tests only advances via `SimHooks.stepTiming`; tests that wait in real time are flaky and slow.
