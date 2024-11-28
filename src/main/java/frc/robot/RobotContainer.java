// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DrivetrainCommands.TankDriveCmd;
import frc.robot.Commands.DrivetrainCommands.TargetAlign;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class RobotContainer { 
  private final CommandXboxController driver = new CommandXboxController(DriverConstants.DRIVER_PORT);

  private final VisionSubsystem visionSub = new VisionSubsystem();
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();

  public RobotContainer() {
    driveSub.setDefaultCommand(
      new TankDriveCmd(driveSub,() -> inputToDrivetrainSpeed(driver.getRawAxis(DriverConstants.LEFT_AXIS)), () -> inputToDrivetrainSpeed(driver.getRawAxis(DriverConstants.RIGHT_AXIS)))
    );

    configureBindings();
  }

  public void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No auto routine.");
  }

  /**
   * Runs an input through a square root function.
   * @param input Raw input between -1.0 and 1.0
   * @return Clamped and calculated output
   */
  private double inputToDrivetrainSpeed(double input) {
    return MathUtil.clamp(Math.signum(input)*(Math.sqrt(Math.abs(input))), -DrivetrainConstants.MAX_MOTOR_SPEED, DrivetrainConstants.MAX_MOTOR_SPEED);
  }
}
