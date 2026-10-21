# Plan 3 — AprilTag vision localization

**Goal:** a field pose that stays correct for a whole match, fused from two AprilTag cameras into the pose estimator that YAMS already runs — so that auto-align and autos (Plan 4) have something to aim with.

**Why:** `SwerveDriveSubsystem.addVisionMeasurement(Pose2d, double)` exists (`SwerveDriveSubsystem.java:139-141`) and `photonlib v2026.3.4` is already a vendordep, but no camera code exists. Odometry alone drifts several cm per cycle of the field; vision resets that drift continuously.

**Effort:** 1–2 weeks including hardware. Code is ~200 lines; calibration and tuning are most of the time.

**Prerequisites:** Plan 1 (odometry under ~15 cm over a square), Plan 5 logging (you need to *see* vision poses to tune them). Cameras ordered early — lead time is the long pole.

---

## Step 1 — Hardware (order in week 1)

**Recommended:** PhotonVision on an Orange Pi 5 with two Arducam OV9281 global-shutter cameras.

| Item | Qty | Approx. cost | Notes |
|---|---|---|---|
| Orange Pi 5 (8 GB) | 1 | ~$99 | PhotonVision's recommended coprocessor; max 2 tag cameras per board |
| Arducam OV9281 (USB, global shutter, ~100° lens) | 2 | ~$40–60 each | Global shutter is required; PhotonVision does not support Logitech or laptop webcams |
| microSD 32 GB+ A2 | 1 | ~$10 | Flash PhotonVision image |
| Regulated 5 V 4 A supply from the PDH/PDP | 1 | ~$20 | Do **not** power the Pi from USB |
| Ethernet to the robot radio/switch | 1 | — | Static IP `10.5.37.11` |

**Alternative:** Limelight 4 (~$489, $660 with Hailo). Less setup, built-in IMU for MegaTag2, no PoE (needs its own power run). Pick this if the team has budget and no mentor comfortable with Linux. The code in Step 4 changes to `LimelightHelpers`, but the fusion logic (Step 5) is the same.

**Mounting (copy the top-team layout):** two front-corner cameras, each yawed ~20° outward and pitched ~25° up from horizontal, as high as the frame allows, looking over the bumpers. Measure the camera-to-robot-center transform from CAD *and* with a tape measure; expect to fudge pitch by a degree or two later.

## Step 2 — Set up the coprocessor (one evening)

1. Flash the PhotonVision 2026 image for Orange Pi 5. Hostname `photonvision`, static IP `10.5.37.11`, subnet `255.255.255.0`.
2. In the PhotonVision web UI, give each camera a stable name (`front-left`, `front-right`). USB enumeration can swap on boot; PhotonVision pins by USB port, so label the ports and never move the plugs.
3. Set the AprilTag pipeline: 1280×720, 36h11 tags, decimate 2, exposure as low as the gym allows (global shutter tolerates low exposure), target ~30 FPS per camera.
4. **Field layout:** 2026 has two variants — Welded (default) and AndyMark. Set the same one in PhotonVision's settings *and* in robot code (Step 4). Check which one your event uses.
5. Turn on "Multi-Target" in the pipeline; coprocessor multi-tag is the most accurate mode.

## Step 3 — Calibrate every camera at its running resolution (1 hour per camera)

This is the step teams skip and regret.

1. Print PhotonVision's ChArUco board (not a checkerboard) on rigid, flat material. Measure the actual square size after printing.
2. In the Calibration tab, pick 1280×720 (the resolution the pipeline will run at).
3. Take at least 12 shots: near, far, every corner of the frame, tilted. 20+ is better.
4. Accept only if mean reprojection error < 1 px. Otherwise redo.
5. Export the calibration JSON and commit it under `docs/vision/calibration/` so it can be re-imported after a re-flash.

## Step 4 — Robot code

### 4a. Expose the 3-arg `addVisionMeasurement`
YAMS `SwerveDrive` has `addVisionMeasurement(Pose2d, double, Matrix<N3,N1>)` (verified). Add a pass-through in `SwerveDriveSubsystem` next to the 2-arg one:

```java
public void addVisionMeasurement(
    Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
  m_drive.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
}

/** YAMS's Field2d, so vision poses are drawn on the same field as odometry. */
public Field2d getField2d() {
  return m_drive.getField2d();
}
```

### 4b. `Vision` subsystem
Create `src/main/java/frc/robot/subsystems/vision/Vision.java`. This is a port of YAGSL-Example's `Vision.java` with the parameter type changed to our subsystem and the field layout updated.

```java
package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  // Check the exact enum name in your WPILib version: k2026RebuiltWelded / k2026RebuiltAndyMark.
  private static final AprilTagFieldLayout kLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  private static final double kMaxSingleTagDistanceMeters = 4.0;
  private static final double kMaxSingleTagAmbiguity = 0.4;
  private static final double kFieldBorderMarginMeters = 0.5;

  /** One entry per camera. Transforms are robot-center → camera; measure them. */
  private enum Cameras {
    FRONT_LEFT(
        "front-left",
        new Translation3d(Units.inchesToMeters(11), Units.inchesToMeters(9), Units.inchesToMeters(8)),
        new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(20))),
    FRONT_RIGHT(
        "front-right",
        new Translation3d(Units.inchesToMeters(11), Units.inchesToMeters(-9), Units.inchesToMeters(8)),
        new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(-20)));

    final PhotonCamera camera;
    final PhotonPoseEstimator estimator;
    Matrix<N3, N1> curStdDevs = kSingleTagStdDevs;

    Cameras(String name, Translation3d t, Rotation3d r) {
      camera = new PhotonCamera(name);
      estimator =
          new PhotonPoseEstimator(
              kLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(t, r));
      estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
  }

  private final SwerveDriveSubsystem m_swerve;
  private int m_rejected = 0; // log this (Plan 5)

  public Vision(SwerveDriveSubsystem swerve) {
    m_swerve = swerve;
  }

  @Override
  public void periodic() {
    for (Cameras cam : Cameras.values()) {
      for (PhotonPipelineResult result : cam.camera.getAllUnreadResults()) {
        Optional<EstimatedRobotPose> est = cam.estimator.update(result);
        if (est.isEmpty()) continue;
        EstimatedRobotPose pose = est.get();
        if (!accept(pose, result)) {
          m_rejected++;
          continue;
        }
        updateStdDevs(cam, pose, result.getTargets());
        m_swerve
            .getField2d()
            .getObject(cam.name())
            .setPose(pose.estimatedPose.toPose2d());
        m_swerve.addVisionMeasurement(
            pose.estimatedPose.toPose2d(), pose.timestampSeconds, cam.curStdDevs);
      }
    }
  }

  private boolean accept(EstimatedRobotPose pose, PhotonPipelineResult result) {
    Pose2d p = pose.estimatedPose.toPose2d();
    if (result.getTargets().isEmpty()) return false;
    // Single tag: reject high ambiguity.
    if (pose.targetsUsed.size() == 1
        && pose.targetsUsed.get(0).getPoseAmbiguity() > kMaxSingleTagAmbiguity) return false;
    // Off the field.
    if (p.getX() < -kFieldBorderMarginMeters
        || p.getX() > kLayout.getFieldLength() + kFieldBorderMarginMeters
        || p.getY() < -kFieldBorderMarginMeters
        || p.getY() > kLayout.getFieldWidth() + kFieldBorderMarginMeters) return false;
    // Floating or buried.
    if (Math.abs(pose.estimatedPose.getZ()) > 0.3) return false;
    return true;
  }

  /** PhotonVision/YAGSL heuristic: base std devs by tag count, scaled by distance². */
  private void updateStdDevs(
      Cameras cam, EstimatedRobotPose pose, List<PhotonTrackedTarget> targets) {
    int numTags = 0;
    double avgDist = 0;
    for (PhotonTrackedTarget t : targets) {
      var tagPose = kLayout.getTagPose(t.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation()
              .getDistance(pose.estimatedPose.toPose2d().getTranslation());
    }
    if (numTags == 0) {
      cam.curStdDevs = kSingleTagStdDevs;
      return;
    }
    avgDist /= numTags;
    Matrix<N3, N1> std = numTags > 1 ? kMultiTagStdDevs : kSingleTagStdDevs;
    if (numTags == 1 && avgDist > kMaxSingleTagDistanceMeters) {
      std = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      std = std.times(1 + (avgDist * avgDist / 30));
    }
    cam.curStdDevs = std;
  }
}
```

Construct it in `RobotContainer` after the swerve: `private final Vision m_vision = new Vision(m_swerve);`. Its `periodic()` runs automatically.

### 4c. What stays fixed
YAMS builds its `SwerveDrivePoseEstimator` with the default constructor, so *odometry* std devs are (0.1, 0.1, 0.1) and `SwerveDriveConfig` has no setter for them. That's fine — all the tuning happens on the vision side.

## Step 5 — Tune the std devs with data, not vibes

1. Put the robot still at 1 m, 2 m, 3 m, 4 m from a tag. Log the vision-only pose (the `Field2d` objects from 4b, or a dedicated log key) for 10 s at each.
2. Compute the standard deviation of x, y, θ at each distance. Typical: a few cm at 1 m, ~10 cm at 3–4 m.
3. If your measured noise is much better than `(4,4,8)×(1+d²/30)`, lower the base values — but stay above measured noise. Trusting vision too much makes the pose jump.
4. Only consider 6328's formula (`0.01·d^1.2/n²`) once you've got 1600×1200 cameras, < 0.5 px calibration error, and strict rejection. Not before.
5. Rotation: leave θ std dev high (we use 8 / 1 above; Limelight uses 9999999). The gyro is better than vision at heading.

## Step 6 — Field tests

| Test | Pass criterion |
|---|---|
| Stand still facing a tag, drive away 5 m by hand with vision off, turn vision on | Pose snaps back within 2 s, no oscillation |
| Drive a lap of the field at speed | `Field2d` robot pose stays on the robot; rejected-count stays low |
| Spin in place at full rotation speed | No pose jumps (if there are, add a gyro-rate rejection: skip frames when `|ω|` > 360°/s) |
| Power-cycle the Pi mid-match simulation | Code doesn't crash; vision resumes |

## Step 7 — Pre-match checklist (put in `docs/checklists/vision.md`)

- [ ] Field layout variant matches the event (Welded/AndyMark) on both Pi and robot
- [ ] Both camera streams live in the PhotonVision UI; names match code
- [ ] Exposure correct for this venue (OV9281 exposure bug: toggle once)
- [ ] `Vision/rejected` counter not climbing while stationary with a tag in view
- [ ] Robot pose on the dashboard field matches where the robot actually is

---

## Done when

- [ ] Two calibrated cameras, calibration files committed.
- [ ] `Vision` subsystem fusing both cameras with distance-scaled std devs and rejection filters.
- [ ] Measured stationary noise table in `docs/vision/noise.md`.
- [ ] All Step 6 tests pass.
- [ ] Pre-match checklist written.

## Risks

- **Layout mismatch** between Pi and robot is the #1 failure. Make it a checklist item.
- PhotonVision 2025 reliability: ~1 in 50 boot or stream failures reported. The checklist catches these; the code must tolerate a dead camera (it does — `getAllUnreadResults()` just returns nothing).
- Timestamps: `EstimatedRobotPose.timestampSeconds` is already FPGA time. Don't pass `Timer.getFPGATimestamp()` instead.
- If continuous fusion is unstable despite tuning, 2910's fallback works: don't fuse continuously; hard-reset odometry from vision only at the start of an auto-align.
- Simulation: PhotonLib's `VisionSystemSim` can simulate cameras; YAGSL-Example's `Vision.java` shows how. Worth adding once the real thing works.
