// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ArmCommands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class AlignWithAprilTag extends Command {
  private final ArmSubsystem armSub;
  private final VisionSubsystem visionSub;
  
  private final PIDController controller = new PIDController(0.04, 0, 0);

  /** Creates a new AlignWithAprilTag. */
  public AlignWithAprilTag(
    ArmSubsystem armSub,
    VisionSubsystem visionSub
  ) {
    this.armSub = armSub;
    this.visionSub = visionSub;
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonTrackedTarget target = visionSub.getTarget();
    if (target == null) {
      System.out.println("Arm not moving, no target found.");
      armSub.setSpeed(0);
      return;
    }

    armSub.setSpeed(
      controller.calculate(target.getPitch(), 0)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
