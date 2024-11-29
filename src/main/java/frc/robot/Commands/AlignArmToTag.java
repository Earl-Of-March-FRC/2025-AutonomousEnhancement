// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class AlignArmToTag extends Command {
  private ArmSubsystem armSubsystem;
  private VisionSubsystem visionSubsystem;
  private PIDController controller = new PIDController(Constants.ArmConstants.PIDController.PROPOTIONAL,Constants.ArmConstants.PIDController.INTEGRAL,Constants.ArmConstants.PIDController.DERIVATIVE);
   
  /** Creates a new AlignArmToTag. */
  public AlignArmToTag(ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {
    this.armSubsystem = armSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonTrackedTarget tag = visionSubsystem.getTarget();

    if (tag != null) {
      armSubsystem.setSpeed(
        controller.calculate(tag.getPitch(),1)
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
