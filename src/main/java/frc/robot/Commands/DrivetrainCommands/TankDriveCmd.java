// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DrivetrainCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class TankDriveCmd extends Command {
  private final DrivetrainSubsystem driveSub;
  private final DoubleSupplier leftInput, rightInput;

  /** Creates a new TankDriveCmd. */
  public TankDriveCmd(
    DrivetrainSubsystem driveSub,
    DoubleSupplier leftInput, DoubleSupplier rightInput
  ) {
    this.driveSub = driveSub;
    this.leftInput = leftInput;
    this.rightInput = rightInput;
    addRequirements(driveSub);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSub.tankDrive(leftInput.getAsDouble(),rightInput.getAsDouble());
  }
}
