// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class OneStickCmd extends Command {
  private DrivetrainSubsystem dSubsystem;
  private DoubleSupplier leftAxis, rightAxis;

  /** Creates a new TankDriveCmd. */
  public OneStickCmd(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier leftAxis, DoubleSupplier rightAxis) {
    this.dSubsystem = drivetrainSubsystem;
    this.leftAxis = leftAxis;
    this.rightAxis = rightAxis;
    addRequirements(dSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dSubsystem.allStickDrive(leftAxis.getAsDouble(), rightAxis.getAsDouble());
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
