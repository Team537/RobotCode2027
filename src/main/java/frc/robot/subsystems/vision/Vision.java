// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Fuses AprilTag poses from the PhotonVision cameras into the drivetrain's pose estimator, so the
 * robot's field position stays correct for a whole match.
 *
 * <p>Each camera result is turned into a robot pose (multi-tag on the coprocessor when two or more
 * tags are visible, otherwise the single tag with the lowest ambiguity), checked against rejection
 * filters, and added with a trust level that drops with distance to the tags.
 *
 * <p>A dead or unplugged camera just produces no results, so the robot keeps driving on odometry.
 */
public class Vision extends SubsystemBase {
  private final SwerveDriveSubsystem m_swerve;
  private final AprilTagFieldLayout m_layout;
  private final List<Camera> m_cameras;
  private final VisionSystemSim m_sim;

  /** One physical camera and its pose estimator. */
  private static final class Camera {
    final String name;
    final PhotonCamera camera;
    final PhotonPoseEstimator estimator;
    final Transform3d robotToCamera;
    int accepted = 0;
    int rejected = 0;

    Camera(String name, Transform3d robotToCamera, AprilTagFieldLayout layout) {
      this.name = name;
      this.camera = new PhotonCamera(name);
      this.estimator = new PhotonPoseEstimator(layout, robotToCamera);
      this.robotToCamera = robotToCamera;
    }
  }

  /**
   * Creates the vision subsystem.
   *
   * @param swerve Drivetrain whose pose estimator receives the vision poses.
   */
  public Vision(SwerveDriveSubsystem swerve) {
    m_swerve = swerve;
    m_layout = AprilTagFieldLayout.loadField(VisionConstants.kFieldLayout);
    m_cameras =
        List.of(
            new Camera(VisionConstants.kFrontLeftName, VisionConstants.kRobotToFrontLeft, m_layout),
            new Camera(
                VisionConstants.kFrontRightName, VisionConstants.kRobotToFrontRight, m_layout));

    m_sim = RobotBase.isSimulation() ? createSim() : null;
  }

  @Override
  public void periodic() {
    for (Camera cam : m_cameras) {
      for (PhotonPipelineResult result : cam.camera.getAllUnreadResults()) {
        process(cam, result);
      }
      SmartDashboard.putBoolean("Vision/" + cam.name + "/connected", cam.camera.isConnected());
      SmartDashboard.putNumber("Vision/" + cam.name + "/accepted", cam.accepted);
      SmartDashboard.putNumber("Vision/" + cam.name + "/rejected", cam.rejected);
    }
  }

  private void process(Camera cam, PhotonPipelineResult result) {
    if (!result.hasTargets()) {
      return;
    }
    Optional<EstimatedRobotPose> estimate = cam.estimator.estimateCoprocMultiTagPose(result);
    if (estimate.isEmpty()) {
      estimate = cam.estimator.estimateLowestAmbiguityPose(result);
    }
    if (estimate.isEmpty()) {
      return;
    }

    EstimatedRobotPose pose = estimate.get();
    Optional<Matrix<N3, N1>> stdDevs = stdDevsFor(pose);
    if (stdDevs.isEmpty() || !isPlausible(pose)) {
      cam.rejected++;
      return;
    }

    cam.accepted++;
    Pose2d pose2d = pose.estimatedPose.toPose2d();
    m_swerve.getField2d().getObject("Vision " + cam.name).setPose(pose2d);
    // timestampSeconds is already in FPGA time, which is what the pose estimator expects.
    m_swerve.addVisionMeasurement(pose2d, pose.timestampSeconds, stdDevs.get());
  }

  /** Rejects poses that can't be right: off the field, or floating above or sunk into the floor. */
  private boolean isPlausible(EstimatedRobotPose pose) {
    Pose3d p = pose.estimatedPose;
    double margin = VisionConstants.kFieldBorderMarginMeters;
    return p.getX() > -margin
        && p.getX() < m_layout.getFieldLength() + margin
        && p.getY() > -margin
        && p.getY() < m_layout.getFieldWidth() + margin
        && Math.abs(p.getZ()) < VisionConstants.kMaxHeightErrorMeters;
  }

  /**
   * How much to trust a pose, based on how many tags were used and how far away they are. Returns
   * empty for single-tag poses that are too ambiguous or too far away to use at all.
   */
  private Optional<Matrix<N3, N1>> stdDevsFor(EstimatedRobotPose pose) {
    int tagCount = 0;
    double totalDistance = 0;
    for (PhotonTrackedTarget target : pose.targetsUsed) {
      Optional<Pose3d> tagPose = m_layout.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) {
        continue;
      }
      tagCount++;
      totalDistance +=
          tagPose
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(pose.estimatedPose.toPose2d().getTranslation());
    }
    if (tagCount == 0) {
      return Optional.empty();
    }

    double avgDistance = totalDistance / tagCount;
    double[] base;
    if (tagCount == 1) {
      PhotonTrackedTarget only = pose.targetsUsed.get(0);
      if (only.getPoseAmbiguity() > VisionConstants.kMaxSingleTagAmbiguity
          || avgDistance > VisionConstants.kMaxSingleTagDistanceMeters) {
        return Optional.empty();
      }
      base = VisionConstants.kSingleTagStdDevs;
    } else {
      base = VisionConstants.kMultiTagStdDevs;
    }

    // Noise grows roughly with distance squared.
    double scale = 1 + (avgDistance * avgDistance / 30);
    return Optional.of(VecBuilder.fill(base[0] * scale, base[1] * scale, base[2] * scale));
  }

  // ---- Simulation ----

  private VisionSystemSim createSim() {
    VisionSystemSim sim = new VisionSystemSim("main");
    sim.addAprilTags(m_layout);

    // Roughly an Arducam OV9281 at the resolution the pipeline will run.
    SimCameraProperties props = new SimCameraProperties();
    props.setCalibration(1280, 720, Rotation2d.fromDegrees(80));
    props.setCalibError(0.35, 0.10);
    props.setFPS(30);
    props.setAvgLatencyMs(35);
    props.setLatencyStdDevMs(5);

    for (Camera cam : m_cameras) {
      PhotonCameraSim camSim = new PhotonCameraSim(cam.camera, props, m_layout);
      sim.addCamera(camSim, cam.robotToCamera);
    }
    SmartDashboard.putData("Vision/Sim Field", sim.getDebugField());
    return sim;
  }

  @Override
  public void simulationPeriodic() {
    // Use the simulator's true robot position, not the estimate, so vision acts like real cameras.
    m_sim.update(m_swerve.getSimPose());
  }
}
