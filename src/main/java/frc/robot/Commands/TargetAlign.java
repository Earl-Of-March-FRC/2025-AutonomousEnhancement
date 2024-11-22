// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class TargetAlign extends Command {
  private final DrivetrainSubsystem driveSub;
  private final VisionSubsystem visionSub;

  /** Creates a new TankDriveCmd. */
  public TargetAlign(
    DrivetrainSubsystem driveSub,
    VisionSubsystem visionSub
) {
    this.driveSub = driveSub;
    this.visionSub = visionSub;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Auto start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = 0, right = 0, yaw;

    PhotonTrackedTarget target = visionSub.getTarget();
    if (target != null) {
      yaw = target.getYaw();

      if (yaw > 0.1) {
        right = 0.1;
        left = 0;
        // System.out.println("left");
      } else if (yaw < -0.1) {
        // System.out.println("Right");
        left = 0.1;
        right = 0;
      } else {
        // System.out.println("Forward");
        right = 0.25;
        left = 0.25;
      }
    }

    driveSub.tankDrive(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
