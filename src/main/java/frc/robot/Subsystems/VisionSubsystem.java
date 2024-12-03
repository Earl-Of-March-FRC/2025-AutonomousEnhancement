// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera(NetworkTableInstance.getDefault(), VisionConstants.CAMERA_NAME); // VisionConstants.CAMERA_NAME
  private final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  private final Transform3d cameraToRobot = new Transform3d(VisionConstants.CAMERA_TO_ROBOT_X, VisionConstants.CAMERA_TO_ROBOT_Y, VisionConstants.CAMERA_TO_ROBOT_Z, new Rotation3d());

  private final VisionSystemSim visionSim = new VisionSystemSim("Simulation");
  private final SimCameraProperties cameraSimProperties = new SimCameraProperties();
  private final PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraSimProperties);

  private PhotonTrackedTarget currentTarget = null;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    cameraSimProperties.setCalibration(640, 480, new Rotation2d(100));
    cameraSimProperties.setCalibError(0.25, 0.08);
    cameraSimProperties.setFPS(30);
    cameraSimProperties.setAvgLatencyMs(35);
    cameraSimProperties.setLatencyStdDevMs(5);

    visionSim.addAprilTags(tagLayout);
    visionSim.addCamera(cameraSim, cameraToRobot);

    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);
  }

  @Override
  public void periodic() {

    //////////////////////////////////////////////////////////////////////
    // PhotonVision
    // currentTarget = getTarget();

    if (currentTarget != null) {
      SmartDashboard.putNumber("Vision_Target_ID", currentTarget.getFiducialId());
      SmartDashboard.putNumber("Vision_Target_Yaw", currentTarget.getYaw());
      SmartDashboard.putNumber("Vision_Target_Pitch", currentTarget.getPitch());
      SmartDashboard.putNumber("Vision_Target_Area", currentTarget.getArea());
      SmartDashboard.putNumber("Vision_Target_Skew", currentTarget.getSkew());
      // Pose3d estimatedPose = estimateRobotPose();
      // if (estimatedPose != null) {
      //   simulatedField.setRobotPose(estimatedPose.toPose2d());
      //   SmartDashboard.putData("PoseFromAprilTag", simulatedField);
      // }
    }
    //////////////////////////////////////////////////////////////////////
  }

  @Override
  public void simulationPeriodic() {
    if (currentTarget != null) {
      visionSim.update(estimateRobotPose());
    }
  }

  /**
   * Get the latest unread result.
   * @return Latest result.
   */
  private PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  /**
   * Get the latest best apriltag target.
   * @return Best target, null if not found.
   */
  public PhotonTrackedTarget getTarget() {
    return getLatestResult().hasTargets() ? getLatestResult().getBestTarget() : null;
  }

  public boolean hasTarget() {
    PhotonPipelineResult result = getLatestResult(); 
    
    return result.hasTargets();
  }

  /**
   * Estimates the robot's position based on a tracked april tag
   * @return {@code Pose3d} containing an estimate robot position. Returns null if position could not be estimated.
   */
  public Pose3d estimateRobotPose() {
    PhotonTrackedTarget target = getTarget();

    if (target == null || target.getFiducialId() == -1) {
      return null;
    }

    Optional<Pose3d> tagPose = tagLayout.getTagPose(target.getFiducialId());
    if (tagPose.isEmpty()) {
      return null;
    }

    return PhotonUtils.estimateFieldToRobotAprilTag(
      target.getBestCameraToTarget(),
      tagPose.get(),
      cameraToRobot
    );
  }
}
