# Plan 6 — Superstructure and mechanism skeleton

**Goal:** a code structure that's ready for mechanisms before any mechanism exists: one pattern for how a subsystem is written, one place that coordinates them (the superstructure state machine), one place where buttons map to states, and LED/dashboard feedback that follows from state. Then one practice mechanism, built from YAMS and run in sim, to prove the pattern.

**Why:** 254, 1678, 3847 and 6328 all converge on "per-mechanism subsystem + superstructure state machine + Triggers in one place". It's what makes one-button cycles, operator-less automation and state LEDs cheap. Writing it now, with a fake mechanism, means January is "fill in the enums", not "design the architecture".

**Effort:** ~1 week.

**Prerequisites:** Plan 5 (logging and sim). Plan 2's "fragile piece" acceleration limit is the first consumer of superstructure state.

---

## Step 1 — Package layout (30 minutes)

```
frc/robot/
  Robot.java
  RobotContainer.java         ← constructs subsystems, calls Bindings
  Bindings.java               ← the ONLY place buttons → states/commands
  Constants.java
  commands/
    DriveToPose.java          ← Plan 4
  subsystems/
    swervedrive/SwerveDriveSubsystem.java
    vision/Vision.java        ← Plan 3
    superstructure/
      Superstructure.java     ← state machine
      SuperstructureState.java
    leds/Leds.java            ← observes state
    example/ExampleElevator.java   ← practice mechanism (Step 4)
  util/
    ChassisAccelLimiter.java, HeadingHold.java, AllianceFlip.java
```

Move the current `configureBindings()` body into `Bindings.java` as a static `configure(RobotContainer)` method or a class that takes the subsystems. Keep the rule from 3847: *which button fires which state is in one file, and only there.*

## Step 2 — The mechanism pattern (one file per mechanism)

Each mechanism subsystem carries its own config, its hardware (a YAMS `SmartMotorController` wrapper or a YAMS mechanism), a `WantedState` enum (what the superstructure asks for) and a `SystemState` enum (what it's actually doing). Sketch:

```java
@Logged
public class ExampleElevator extends SubsystemBase {
  public enum WantedState { IDLE, STOW, LOW, HIGH, MANUAL }
  public enum SystemState { IDLE, MOVING, AT_GOAL, MANUAL }

  private WantedState m_wanted = WantedState.IDLE;
  private SystemState m_system = SystemState.IDLE;
  private final Elevator m_elevator;   // yams.mechanisms.positional.Elevator

  public ExampleElevator() { /* YAMS config, see Step 4 */ }

  public void setWantedState(WantedState s) { m_wanted = s; }
  public SystemState getSystemState() { return m_system; }
  public boolean atGoal() { return m_system == SystemState.AT_GOAL; }

  @Override
  public void periodic() {
    m_system = handleStateTransition();
    applyState();
    m_elevator.updateTelemetry();
  }

  private SystemState handleStateTransition() {
    return switch (m_wanted) {
      case IDLE -> SystemState.IDLE;
      case MANUAL -> SystemState.MANUAL;
      default -> m_elevator.isNearTarget() ? SystemState.AT_GOAL : SystemState.MOVING;  // check exact YAMS method
    };
  }

  private void applyState() {
    switch (m_wanted) {
      case IDLE -> m_elevator.setVoltage(Volts.zero());
      case STOW -> m_elevator.setHeight(Inches.of(0));
      case LOW -> m_elevator.setHeight(Inches.of(12));
      case HIGH -> m_elevator.setHeight(Inches.of(40));
      case MANUAL -> { /* joystick voltage */ }
    }
  }
}
```

Rules:
- The mechanism never decides *when* to move; the superstructure does.
- `periodic()` is the only place hardware is written. No commands reach into motors.
- Every mechanism has a `MANUAL` state for pit testing.
- Sensors (beam breaks, CANrange, LaserCAN) are read in `periodic()` and exposed as booleans; the superstructure gates on them.

## Step 3 — The superstructure (the state machine)

```java
@Logged
public class Superstructure extends SubsystemBase {
  public enum WantedSuperState { IDLE, STOW, PREPARE_SCORE_LOW, PREPARE_SCORE_HIGH, SCORE, INTAKE, CLIMB }
  public enum SuperState { IDLE, STOWED, MOVING, READY_TO_SCORE, SCORING, INTAKING, HAS_PIECE, CLIMBING }

  private final ExampleElevator m_elevator;
  // private final Intake m_intake; ... (January)
  private WantedSuperState m_wanted = WantedSuperState.IDLE;
  private SuperState m_state = SuperState.IDLE;

  public Superstructure(ExampleElevator elevator) { m_elevator = elevator; }

  public Command request(WantedSuperState s) {
    return runOnce(() -> m_wanted = s).withName("Want " + s);
  }
  public SuperState getState() { return m_state; }
  public boolean holdingFragilePiece() { return m_state == SuperState.HAS_PIECE; }  // Plan 2 consumer

  @Override
  public void periodic() {
    m_state = handleStateTransition();
    applyStates();
  }

  private SuperState handleStateTransition() {
    return switch (m_wanted) {
      case IDLE -> SuperState.IDLE;
      case STOW -> m_elevator.atGoal() ? SuperState.STOWED : SuperState.MOVING;
      case PREPARE_SCORE_LOW, PREPARE_SCORE_HIGH ->
          m_elevator.atGoal() ? SuperState.READY_TO_SCORE : SuperState.MOVING;
      case SCORE -> SuperState.SCORING;
      case INTAKE -> SuperState.INTAKING;   // later: HAS_PIECE when the sensor trips
      case CLIMB -> SuperState.CLIMBING;
    };
  }

  private void applyStates() {
    switch (m_wanted) {
      case IDLE -> m_elevator.setWantedState(ExampleElevator.WantedState.IDLE);
      case STOW, INTAKE -> m_elevator.setWantedState(ExampleElevator.WantedState.STOW);
      case PREPARE_SCORE_LOW -> m_elevator.setWantedState(ExampleElevator.WantedState.LOW);
      case PREPARE_SCORE_HIGH, SCORE -> m_elevator.setWantedState(ExampleElevator.WantedState.HIGH);
      case CLIMB -> { /* January */ }
    }
  }
}
```

In `Bindings.java`:

```java
driver.a().onTrue(superstructure.request(WantedSuperState.PREPARE_SCORE_HIGH));
driver.rightTrigger().onTrue(superstructure.request(WantedSuperState.SCORE))
                     .onFalse(superstructure.request(WantedSuperState.STOW));
new Trigger(superstructure::holdingFragilePiece)       // Plan 2: slow accel while holding
    .whileTrue(Commands.idle());                        // placeholder; limiter reads the boolean directly
```

Triggers *are* the glue — no multi-mechanism command groups needed for the normal cycle. Command groups stay for autos (Plan 4) where sequencing is explicit.

## Step 4 — One practice mechanism in YAMS (2 days)

Use the YAMS mechanism library so you learn its config, sim and telemetry before January.

1. Go to the YAMG generator (yamg.yassrobotics.com) and generate an **Elevator** on a Kraken X60 (or whatever motor you have spare) with a plausible gearing and drum radius. It produces the `SmartMotorControllerConfig` + `ElevatorConfig` boilerplate.
2. Paste into `ExampleElevator`. Set soft limits, a `ProfiledPIDController` or Motion Magic profile, and feedforward (`ElevatorFeedforward` with kG). Use `SysId` routines from the YAMS mechanism to get kS/kV/kG when real hardware exists.
3. Call `m_elevator.simIterate()` in `simulationPeriodic()`.
4. Run sim; in AdvantageScope's Mechanism tab watch the elevator move between `LOW`/`HIGH` from the buttons. Log `WantedState`, `SystemState`, setpoint and measured height.
5. If there's a spare motor and a bench rig, run it for real with the same code.

Even if the 2027 robot has no elevator, the exercise transfers: `Arm`, `Pivot`, `Flywheel` in YAMS share the config pattern.

## Step 5 — State observers: LEDs, dashboard, rumble (1 day)

`Leds` subsystem (CANdle, or PWM addressable via WPILib `AddressableLED`) that reads `Superstructure.getState()` in `periodic()` and shows a color per state. Add vision health (Plan 3) as a stripe. No commands set LEDs; they *observe*.

Elastic (Plan 5) gets a text widget for `Superstructure/State` and `Superstructure/Wanted`. Rumble on `HAS_PIECE` via a Trigger in `Bindings`.

## Step 6 — Per-robot config (optional, 1 hour)

If you keep the 2026 chassis as a practice bot while building the 2027 robot, select constants by roboRIO serial (`RobotController.getSerialNumber()`) as 3847 does: `Constants.robot = switch (serial) { case "…" -> PRACTICE; default -> COMP; }`. Keeps one codebase deployable to both.

---

## Done when

- [ ] `Bindings.java` is the only file with controller bindings.
- [ ] `Superstructure` with `WantedSuperState`/`SuperState` runs and logs its state.
- [ ] `ExampleElevator` built from YAMS, moves in sim between states, visible in AdvantageScope.
- [ ] `Leds` reflects superstructure state; Elastic shows it.
- [ ] Plan 2's fragile-piece accel limit is driven by `Superstructure.holdingFragilePiece()`.

## Risks

- Over-engineering before the game is known. Keep the enums short; they're placeholders.
- YAMS method names (`isNearTarget`, `setHeight`, `setVoltage`) — check against the 2026.10.03 javadoc; YAMG output is the source of truth.
- State machines in `periodic()` must never block or sleep. If a transition needs time, it's a state, not a wait.
